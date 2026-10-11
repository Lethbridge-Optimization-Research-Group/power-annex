## Bismillah
###############################################################################
# File: test1.jl
# Author: Shidratul Muntaha
# Research: Active-Power Uncertainty in AC Chance-Constrained OPF
# Purpose: Read experiment inputs, call the solver, and print results.
###############################################################################

# Load the helper functions first, then the solver function.
include(joinpath(@__DIR__, "..", "util-org", "util_cc.jl"))
include(joinpath(@__DIR__, "..", "basemodels-org", "solve_ac_cc.jl"))

###############################################################################
# 1. Load and Process System Data
###############################################################################

# Keep the same case as your original experiment.
file_name = "/Users/shidratulmuntaha/power-annex/.github/workflows/cases/case118.m"
data, ref = read_case_cc(file_name)



###############################################################################
# 2. Solve the Baseline AC-OPF
###############################################################################

# Solve the case before adding uncertainty.
baseline = PowerModels.solve_ac_opf(data, Ipopt.Optimizer)

println("Loads: ", length(ref[:load]))
println("Generators: ", length(ref[:gen]))
# Check whether the solve succeeded.

status = baseline["termination_status"]
println("Baseline status: ", status)

if !(status in (JuMP.MOI.OPTIMAL, JuMP.MOI.LOCALLY_SOLVED))
    error("Baseline solve failed. Check the solver output.")
end

# Save the cost and generator/voltage results.
baseline_cost = baseline["objective"]
baseline_solution = baseline["solution"]

println("Baseline cost: ", baseline_cost)

###############################################################################
# 3. Select Uncertain Loads and Generators
###############################################################################

# Show the available component IDs.
println("Available load IDs: ", sort(collect(keys(ref[:load]))))
println("Available generator IDs: ", sort(collect(keys(ref[:gen]))))

# Enter IDs separated by commas. Press Enter to select none.
print("Which LOAD IDs should be uncertain? ")
load_input = strip(readline())

uncertain_load_ids = isempty(load_input) ? Int[] :
    unique(parse.(Int, strip.(split(load_input, ","))))

print("Which GENERATOR IDs should be uncertain? ")
gen_input = strip(readline())

uncertain_gen_ids = isempty(gen_input) ? Int[] :
    unique(parse.(Int, strip.(split(gen_input, ","))))

# Check that the selected IDs exist.
for id in uncertain_load_ids
    haskey(ref[:load], id) || error("Load ID $id does not exist.")
end

for id in uncertain_gen_ids
    haskey(ref[:gen], id) || error("Generator ID $id does not exist.")
end

println("Selected loads: ", uncertain_load_ids)
println("Selected generators: ", uncertain_gen_ids)

###############################################################################
# 4. Create the Uncertainty Dictionaries
###############################################################################

# This helper asks for each selected component's active-power sigma.
uncertainty = create_uncertainty(
    ref, baseline_solution, uncertain_load_ids, uncertain_gen_ids
)
println("\nUncertainty dictionaries created.")

###############################################################################
# 5. Check the Uncertainty Dictionaries
###############################################################################

println("\nUNCERTAIN LOADS")

for id in sort(collect(keys(uncertainty[:load])))
    entry = uncertainty[:load][id]

    println("\nLoad $id, Bus ", entry["load_bus"])
    println("Pd: mean = ", entry["exp_pd"],
            ", sigma = ", entry["sigma_pd"])
  
end

println("\nUNCERTAIN GENERATORS")

for id in sort(collect(keys(uncertainty[:gen])))
    entry = uncertainty[:gen][id]

    println("\nGenerator $id, Bus ", entry["gen_bus"])
    println("Pg: mean = ", entry["exp_pg"],
            ", sigma = ", entry["sigma_pg"])
    
        
end

###############################################################################
# 6. Choose Confidence and Tail Split
###############################################################################

print("Confidence percentage, e.g. 95: ")
confidence_percent = parse(Float64, strip(readline()))

print("Percentage of violation probability in lower tail, e.g. 20: ")
lower_share = parse(Float64, strip(readline()))

0 < confidence_percent < 100 ||
    error("Confidence must be between 0 and 100.")

0 < lower_share < 100 ||
    error("Lower-tail share must be between 0 and 100.")

confidence = confidence_percent / 100
alpha = 1 - confidence

tail_minus = alpha * lower_share / 100
tail_plus = alpha - tail_minus

println("Alpha: ", alpha)
println("Lower tail: ", tail_minus)
println("Upper tail: ", tail_plus)

###############################################################################
# 9. Group Uncertain Components by Bus
###############################################################################

uncertain_buses = group_uncertain_components(ref, uncertainty)

# Check the grouping.
for bus_id in sort(collect(keys(uncertain_buses)))
    println(
        "Bus ", bus_id,
        ": uncertain loads = ", uncertain_buses[bus_id][:load],
        ", uncertain generators = ", uncertain_buses[bus_id][:gen]
    )
end

###############################################################################
# 10. Prepare Bus Active-Power Uncertainty Parameters
###############################################################################

bus_uncertainty = prepare_bus_uncertainty(uncertainty, uncertain_buses)

###############################################################################
# 16. Calculate Truncated Bounds and Enter Tolerances
###############################################################################

bus_bounds = Dict()
bus_epsilon = Dict()

for i in sort(collect(keys(bus_uncertainty)))
    info = bus_uncertainty[i]

    fixed_loads = [
        ref[:load][id] for id in ref[:bus_loads][i]
        if !haskey(uncertainty[:load], id)
    ]

    # Underlying Normal parameters: baseline generation minus mean demand.
    mu_p = sum(
        baseline_solution["gen"][string(g)]["pg"]
        for g in ref[:bus_gens][i]; init = 0.0
    ) - info["mu_pd"] -
        sum(l["pd"] for l in fixed_loads; init = 0.0)

    

    println("\nBus $i — ACTIVE net injection")
    low_p, high_p = read_limits()
    a1, a2 = normal_bounds(
        mu_p, info["sigma_p"], low_p, high_p, tail_minus, tail_plus
    )
    
    p_minus, p_plus = read_tolerances(a2 - a1)

    
    bus_bounds[i] = Dict(
        "a1" => a1, "a2" => a2,
        
    )

    bus_epsilon[i] = Dict(
        "p_minus" => p_minus, "p_plus" => p_plus,
       
    )
end

###############################################################################
# 18. Initialize and Solve the Model
###############################################################################

# Use a fresh model for this experiment.
model = init_ac_cc()
result = solve_model_ccac!(model, ref, uncertainty, bus_bounds, bus_epsilon)

###############################################################################
# 19. Print the Results
###############################################################################

println("Status: ", result[:status])
println("Objective: ", result[:objective])

if result[:objective] !== nothing
    println("Baseline cost: ", baseline_cost)
    println("New model cost: ", result[:objective])
    println("Cost difference: ", result[:objective] - baseline_cost)
else
    println("No successful solution; cost comparison unavailable.")
end
