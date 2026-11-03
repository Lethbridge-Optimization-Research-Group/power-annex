"""
    ExtremeScenarioGenerator

Generate demand-side operating scenarios from a PowerModels reference dictionary.

This first version is impact-based: it describes how demand changes at each bus.
It does not model the real-world event that caused the change, and it does not
sample from a probability distribution.

Each generated scenario contains a deep copy of the original network reference
(`ref`) with its load `pd` and `qd` values modified. The input reference is not
changed.
"""
module ExtremeScenarioGenerator

export DemandScenario,
       generate_demand_scenario,
       generate_baseline_scenario,
       generate_global_scenarios,
       generate_localized_scenario,
       generate_uneven_scenario

"""
    DemandScenario

A generated demand scenario.

Fields:
- `id`: unique, human-readable scenario identifier.
- `category`: scenario category, such as `:baseline`, `:global_high`,
  `:localized_extreme`, or `:uneven_demand`.
- `description`: plain-language description of the scenario.
- `bus_multipliers`: effective demand multiplier applied at each loaded bus.
- `ref`: copied PowerModels reference dictionary containing the modified loads.

Both active demand (`pd`) and reactive demand (`qd`) are scaled by the same
bus-level multiplier. This preserves each load's original power factor.
"""
struct DemandScenario{R}
    id::String
    category::Symbol
    description::String
    bus_multipliers::Dict{Int, Float64}
    ref::R
end

function _validated_multiplier(value::Real, label::AbstractString)
    multiplier = Float64(value)
    if !isfinite(multiplier) || multiplier <= 0.0
        throw(ArgumentError("$label must be a finite number greater than 0; got $value"))
    end
    return multiplier
end

function _load_bus_id(load, load_id)
    haskey(load, "load_bus") ||
        throw(ArgumentError("Load $load_id does not contain the \"load_bus\" field."))

    bus_value = load["load_bus"]
    if !(bus_value isa Real) || !isfinite(bus_value) || !isinteger(bus_value)
        throw(ArgumentError("Load $load_id has an invalid load_bus value: $bus_value"))
    end
    return Int(bus_value)
end

function _validate_reference(ref)
    haskey(ref, :load) ||
        throw(ArgumentError("The reference dictionary must contain ref[:load]."))

    isempty(ref[:load]) &&
        throw(ArgumentError("ref[:load] is empty; no demand scenarios can be generated."))

    for (load_id, load) in ref[:load]
        for field in ("pd", "qd")
            haskey(load, field) ||
                throw(ArgumentError("Load $load_id does not contain the \"$field\" field."))
            value = load[field]
            if !(value isa Real) || !isfinite(value)
                throw(ArgumentError("Load $load_id has an invalid $field value: $value"))
            end
        end
        _load_bus_id(load, load_id)
    end

    return nothing
end

"""
    generate_demand_scenario(ref; id, category, description,
                             default_multiplier=1.0,
                             bus_multipliers=Dict())

General-purpose scenario constructor.

`default_multiplier` is applied to every load unless its bus appears in
`bus_multipliers`. A bus-specific multiplier overrides the default.

This function can represent a combined scenario as well. For example, use a
default multiplier greater than 1 to increase demand system-wide, then give a
larger multiplier to selected buses to represent a more severe local increase.
"""
function generate_demand_scenario(
    ref;
    id::AbstractString,
    category::Symbol,
    description::AbstractString,
    default_multiplier::Real = 1.0,
    bus_multipliers::AbstractDict = Dict{Int, Float64}(),
)
    _validate_reference(ref)

    default_factor = _validated_multiplier(default_multiplier, "default_multiplier")

    normalized_bus_multipliers = Dict{Int, Float64}()
    for (bus, value) in bus_multipliers
        if !(bus isa Integer)
            throw(ArgumentError("Bus IDs in bus_multipliers must be integers; got $bus"))
        end
        bus_id = Int(bus)
        normalized_bus_multipliers[bus_id] =
            _validated_multiplier(value, "Multiplier for bus $bus_id")
    end

    loaded_bus_ids = Set{Int}()
    for (load_id, load) in ref[:load]
        push!(loaded_bus_ids, _load_bus_id(load, load_id))
    end

    unknown_bus_ids = setdiff(Set(keys(normalized_bus_multipliers)), loaded_bus_ids)
    if !isempty(unknown_bus_ids)
        unknown_list = join(sort!(collect(unknown_bus_ids)), ", ")
        throw(ArgumentError("No loads were found at bus ID(s): $unknown_list"))
    end

    # Work on a deep copy so that generating scenarios never changes the base case.
    scenario_ref = deepcopy(ref)
    effective_multipliers = Dict{Int, Float64}()

    for (load_id, load) in scenario_ref[:load]
        bus_id = _load_bus_id(load, load_id)
        multiplier = get(normalized_bus_multipliers, bus_id, default_factor)
        effective_multipliers[bus_id] = multiplier

        load["pd"] = Float64(load["pd"]) * multiplier
        load["qd"] = Float64(load["qd"]) * multiplier
    end

    return DemandScenario(
        String(id),
        category,
        String(description),
        effective_multipliers,
        scenario_ref,
    )
end

"""
    generate_baseline_scenario(ref)

Return an unchanged copy of the base case as a baseline scenario.
"""
function generate_baseline_scenario(ref)
    return generate_demand_scenario(
        ref;
        id = "baseline",
        category = :baseline,
        description = "Original demand; no scaling is applied.",
    )
end

"""
    generate_global_scenarios(ref; multipliers=[1.10, 1.20, 1.30])

Generate one global high-demand scenario for each supplied multiplier.
Every loaded bus receives the same multiplier.

The multipliers are stress-test settings, not probabilities or random samples.
"""
function generate_global_scenarios(
    ref;
    multipliers::AbstractVector{<:Real} = [1.10, 1.20, 1.30],
)
    scenarios = DemandScenario[]

    for (index, value) in enumerate(multipliers)
        multiplier = _validated_multiplier(value, "multipliers[$index]")
        multiplier_label = replace(string(multiplier), "." => "_")

        push!(
            scenarios,
            generate_demand_scenario(
                ref;
                id = "global_$(index)_$(multiplier_label)x",
                category = :global_high,
                description = "All loaded buses have demand multiplied by $multiplier.",
                default_multiplier = multiplier,
            ),
        )
    end

    return scenarios
end

"""
    generate_localized_scenario(ref; bus_ids, multiplier, id="localized_extreme")

Increase demand only at the selected buses. Loads at every other bus remain
unchanged. All loads attached to a selected bus receive the same multiplier.
"""
function generate_localized_scenario(
    ref;
    bus_ids::AbstractVector{<:Integer},
    multiplier::Real,
    id::AbstractString = "localized_extreme",
)
    isempty(bus_ids) && throw(ArgumentError("bus_ids cannot be empty."))

    factor = _validated_multiplier(multiplier, "multiplier")
    selected_buses = unique(Int.(bus_ids))
    bus_factors = Dict(bus_id => factor for bus_id in selected_buses)

    return generate_demand_scenario(
        ref;
        id = id,
        category = :localized_extreme,
        description = "Demand at bus/buses $(join(selected_buses, ", ")) is multiplied by $factor; other buses are unchanged.",
        bus_multipliers = bus_factors,
    )
end

"""
    generate_uneven_scenario(ref; bus_multipliers, id="uneven_demand")

Apply different demand multipliers at different buses. Buses omitted from
`bus_multipliers` remain at their original demand.

Example:

    generate_uneven_scenario(ref;
        bus_multipliers = Dict(4 => 1.2, 7 => 1.8, 12 => 1.4))
"""
function generate_uneven_scenario(
    ref;
    bus_multipliers::AbstractDict,
    id::AbstractString = "uneven_demand",
)
    isempty(bus_multipliers) &&
        throw(ArgumentError("bus_multipliers cannot be empty."))

    return generate_demand_scenario(
        ref;
        id = id,
        category = :uneven_demand,
        description = "Different demand multipliers are applied at selected buses.",
        bus_multipliers = bus_multipliers,
    )
end

end # module ExtremeScenarioGenerator
