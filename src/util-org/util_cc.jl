## Bismillah
###############################################################################
# File: util_cc.jl
# Author: Shidratul Muntaha
# Research: Active-Power Uncertainty in AC Chance-Constrained Optimal Power Flow
# Purpose: Reusable helpers for case data nd uncertainty dictionaries.
###############################################################################

using PowerModels
using Ipopt
using JuMP
using Distributions

###############################################################################
# 1. Read and Process the Case
###############################################################################

# Return both data for the baseline solve and ref for our new model.
function read_case_cc(filename)
    data = PowerModels.parse_file(filename)
    PowerModels.standardize_cost_terms!(data, order = 2)
    PowerModels.calc_thermal_limits!(data)
    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
    return data, ref
end

###############################################################################
# 2. Initialize the JuMP Model
###############################################################################

# Variables and constraints are added in the solver file.
function init_ac_cc()
    model = Model(Ipopt.Optimizer)
    set_optimizer_attribute(model, "print_level", 0)
    return model
end

###############################################################################
# 3. Read a Nonnegative Power Value
###############################################################################

# Used for sigma and power tolerances, both in p.u.
function read_sigma(message)
    print(message)
    sigma = parse(Float64, strip(readline()))

    isfinite(sigma) && sigma >= 0 ||
        error("Sigma must be finite and nonnegative.")

    return sigma
end

###############################################################################
# 4. Construct the Uncertainty Dictionary
###############################################################################

# Only selected IDs get entries; generator means come from the baseline.
function create_uncertainty(ref, baseline_solution, uncertain_load_ids, uncertain_gen_ids)
    uncertainty = Dict(
        :load => Dict(),
        :gen => Dict()
    )

    # Selected uncertain loads.
    for id in uncertain_load_ids
        load = ref[:load][id]
        println("\nLoad $id — enter sigma in p.u.")

        uncertainty[:load][id] = Dict(
            "load_bus" => load["load_bus"],
            "exp_pd" => load["pd"],
         
            "sigma_pd" => read_sigma("Sigma Pd: "),
            
        )
    end

    # Selected uncertain generators.
    for id in uncertain_gen_ids
        gen = ref[:gen][id]
        solution = baseline_solution["gen"][string(id)]
        println("\nGenerator $id — enter sigma in p.u.")

        uncertainty[:gen][id] = Dict(
            "gen_bus" => gen["gen_bus"],
            "exp_pg" => solution["pg"],
          
            "sigma_pg" => read_sigma("Sigma Pg: "),
           
        )
    end
    return uncertainty
end

###############################################################################
# 5. Combine Independent Standard Deviations
###############################################################################

# Root of the sum of squared sigmas; independence is assumed.
function combined_sigma(sigmas)
    return sqrt(sum(sigma^2 for sigma in sigmas; init = 0.0))
end

###############################################################################
# 6. Calculate Truncated-Normal Quantiles
###############################################################################

# mu and sigma describe the underlying Normal, before truncation.
# low and high are its allowed net-injection range, in p.u.
# The returned values are the active-power bounds a1 and a2.
# This keeps the current bus-level truncated-Normal approximation.
function normal_bounds(mu, sigma, low, high, tail_minus, tail_plus)
    d = truncated(Normal(mu, sigma), low, high)

    return quantile(d, tail_minus),
           quantile(d, 1 - tail_plus)
end

###############################################################################
# 7. Group Selected Components by Bus
###############################################################################

# Bus constraints need the selected load and generator IDs at each bus.
function group_uncertain_components(ref, uncertainty)
    uncertain_buses = Dict()

    for bus_id in keys(ref[:bus])
        # Find selected uncertain loads at this bus.
        load_ids = [
            id for id in ref[:bus_loads][bus_id]
            if haskey(uncertainty[:load], id)
        ]

        # Find selected uncertain generators at this bus.
        gen_ids = [
            id for id in ref[:bus_gens][bus_id]
            if haskey(uncertainty[:gen], id)
        ]

        # Keep buses with at least one uncertain component.
        if !isempty(load_ids) || !isempty(gen_ids)
            uncertain_buses[bus_id] = Dict(
                :load => load_ids,
                :gen => gen_ids
            )
        end
    end
    return uncertain_buses
end

###############################################################################
# 8. Prepare Bus Uncertainty Parameters
###############################################################################

# Aggregate selected load means and independent active-power error sigmas.
function prepare_bus_uncertainty(uncertainty, uncertain_buses)
    bus_uncertainty = Dict()

    for bus_id in keys(uncertain_buses)
        load_ids = uncertain_buses[bus_id][:load]
        gen_ids = uncertain_buses[bus_id][:gen]

        l3oads = [uncertainty[:load][id] for id in load_ids]
        gens = [uncertainty[:gen][id] for id in gen_ids]

        # Selected uncertain load means.
        mu_pd = sum(l["exp_pd"] for l in loads; init = 0.0)
      

        # Combined sigma: assumes independent uncertainties.
        sigma_p = combined_sigma(vcat(
            [g["sigma_pg"] for g in gens],
            [l["sigma_pd"] for l in loads]
        ))

       

        bus_uncertainty[bus_id] = Dict(
            "gen_ids" => gen_ids,
            "mu_pd" => mu_pd,
            "sigma_p" => sigma_p,
           
        )
    end
    return bus_uncertainty
end

###############################################################################
# 9. Read Truncation Limits and Power Tolerances
###############################################################################

# Limits bound the distribution; tolerances bound the allowed power interval.
function read_limits()
    print("Lower truncation limit (p.u.): ")
    low = parse(Float64, strip(readline()))

    print("Upper truncation limit (p.u.): ")
    high = parse(Float64, strip(readline()))

    low < high || error("Lower limit must be below upper limit.")
    return low, high
end

function read_tolerances(required)
    println("Minimum total tolerance: ", required)

    while true
        minus = read_sigma("Epsilon minus (p.u.): ")
        plus = read_sigma("Epsilon plus (p.u.): ")

        minus + plus >= required && return minus, plus
        println("Total is too small. Enter again.")
    end
end
