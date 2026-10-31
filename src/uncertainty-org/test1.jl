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
###############################################################################
###############################################################################
# 2A. Find Closest and Furthest Buses with Both Load and Generation
###############################################################################

# Ask for the study bus.
print("Study bus ID: ")
study_bus = parse(Int, strip(readline()))

haskey(ref[:bus], study_bus) || error("Bus ID does not exist.")
println("Loads: ", ref[:bus_loads][study_bus])
println("Generators: ", ref[:bus_gens][study_bus])
# Calculate shortest distances using active AC branch connections.

distance = Dict(study_bus => 0)#Creates a dictionary that stores each bus’s distance from the study bus.

queue = [study_bus] #start search from the selected bus 

for i in queue
    for branch in values(ref[:branch])
#br_status == 1 means the line is active.
        branch["br_status"] == 1 || continue

        from = branch["f_bus"]
        to = branch["t_bus"]

        # Find the bus connected to bus i.
        if from == i
            neighbour = to
        elseif to == i
            neighbour = from
        else
            continue
        end

        # Record each bus when first reached.
##discover new neighbours, record their distance, and add them to the queue(BFS)

        if !haskey(distance, neighbour)
            distance[neighbour] = distance[i] + 1
            push!(queue, neighbour)
        end
    end
end

# Keep other reachable buses containing both loads and generators.
eligible_buses = [
    i for i in keys(distance)
    if i != study_bus &&
       !isempty(ref[:bus_loads][i]) &&
       !isempty(ref[:bus_gens][i])
]

isempty(eligible_buses) &&
    error("No other reachable bus contains both a load and generator.")

# Find the nearest and furthest eligible distances.
nearest = minimum(distance[i] for i in eligible_buses)
furthest = maximum(distance[i] for i in eligible_buses)

closest_buses = sort([
    i for i in eligible_buses if distance[i] == nearest
])

furthest_buses = sort([
    i for i in eligible_buses if distance[i] == furthest
])

# Display all candidates, including ties.
println("\nStudy bus: ", study_bus)
println("Closest eligible buses: ", closest_buses,
        " — ", nearest, " connections")
println("Furthest eligible buses: ", furthest_buses,
        " — ", furthest, " connections")

# Show component IDs at the candidate buses.
for bus_id in union(closest_buses, furthest_buses)
    println("Bus $bus_id: loads = ", ref[:bus_loads][bus_id],
            ", generators = ", ref[:bus_gens][bus_id])
end

# Select any number of closest eligible buses.
print("Choose closest buses, separated by commas: ")
near_buses = unique(parse.(Int, strip.(split(readline(), ","))))

all(b -> b in closest_buses, near_buses) ||
    error("Choose buses from the displayed closest list.")

# Select any number of furthest eligible buses.
print("Choose furthest buses, separated by commas: ")
far_buses = unique(parse.(Int, strip.(split(readline(), ","))))

all(b -> b in furthest_buses, far_buses) ||
    error("Choose buses from the displayed furthest list.")

comparison_buses = unique(vcat(near_buses, far_buses))

println("Comparison buses: ", comparison_buses)

###############################################################################
# 2B. Save Baseline Values at the Comparison Buses
###############################################################################

baseline_pg = Dict()
baseline_vm = Dict()
baseline_va = Dict()

for bus_id in comparison_buses
    # Total active generation at the bus, in p.u.
    baseline_pg[bus_id] = sum(
        baseline_solution["gen"][string(g)]["pg"]
        for g in ref[:bus_gens][bus_id]; init = 0.0
    )

    # Voltage magnitude in p.u. and voltage angle in radians.
    bus = baseline_solution["bus"][string(bus_id)]
    baseline_vm[bus_id] = bus["vm"]
    baseline_va[bus_id] = bus["va"]

    println("\nBus $bus_id baseline:")
    println("Pg: ", baseline_pg[bus_id])
    println("Voltage magnitude: ", baseline_vm[bus_id])
    println("Voltage angle: ", baseline_va[bus_id])
end


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
# 3A. Save the Selected Generator's Baseline Contribution
###############################################################################

# Check that exactly one generator was selected.
length(uncertain_gen_ids) == 1 ||
    error("Select exactly one generator for this experiment.")

# Get its ID and information from the case.
g = only(uncertain_gen_ids)
gen = ref[:gen][g]

# Get its active-power output from the baseline AC-OPF solution.
# We will use this value as the renewable's underlying Normal mean.
pg_star = baseline_solution["gen"][string(g)]["pg"]

# Read its quadratic cost coefficients: c2*Pg^2 + c1*Pg + c0.
c2, c1, c0 = gen["cost"]

# Calculate how much this generator contributed to the baseline cost.
generator_cost = c2 * pg_star^2 + c1 * pg_star + c0

# Subtract its contribution because we assume zero renewable operating cost.
# This adjusts the comparison value; it does not change the baseline solution.
baseline_other_cost = baseline_cost - generator_cost

# Display the selected generator and the adjusted baseline cost.
println("Selected generator: ", g)
println("Generator bus: ", gen["gen_bus"])
println("Baseline Pg: ", pg_star)
println("Selected generator cost: ", generator_cost)
println("Baseline cost excluding this generator: ", baseline_other_cost)

###############################################################################
# 3B. Check the Selected Load Location
###############################################################################
##selected one load and that both the load and generator are at your study bus.

# Start with exactly one uncertain load.
length(uncertain_load_ids) == 1 ||
    error("Select exactly one load for this experiment.")

# Get the selected load and its bus.
d = only(uncertain_load_ids)
load_bus = ref[:load][d]["load_bus"]
gen_bus = ref[:gen][g]["gen_bus"]

# For this first experiment, both components must share a bus.
load_bus == gen_bus ||
    error("Choose a load and generator at the same bus.")

# Both selected components must be at the chosen study bus.
gen_bus == study_bus ||
    error("Choose a load and generator at Bus $study_bus.")
    
println("Study bus: ", gen_bus)
println("Selected load: ", d)
println("Selected generator: ", g)


###############################################################################

###############################################################################
# 3C. Choose Uncertainty Levels

load_sigma_percent = read_sigma("Load sigma (% of demand): ")
sigma_pd = (load_sigma_percent / 100) * abs(ref[:load][d]["pd"])

sigma_percentages = 10.0:10.0:50.0
costs = Float64[]

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


# 4. Create the Uncertainty Dictionaries
###############################################################################

for gen_sigma_percent in sigma_percentages
    sigma_pg = (gen_sigma_percent / 100) * abs(pg_star)

    println("\nGeneration sigma: $gen_sigma_percent%")


uncertainty = create_uncertainty(
    ref, baseline_solution,
    uncertain_load_ids, uncertain_gen_ids,
    sigma_pd, sigma_pg
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
###############################################################################
###############################################################################
###############################################################################
# 11. Calculate Net-Injection Bounds
###############################################################################

sigma_pg > 0 || error("Choose a positive generator sigma.")
sigma_pd > 0 || error("Choose a positive load sigma.")

# Renewable output stays between zero and capacity.
pmax = ref[:gen][g]["pmax"]
gen_distribution = truncated(
    Normal(pg_star, sigma_pg), 0.0, pmax
)

# Keep the existing Normal demand assumption.
load_distribution = truncated(
    Normal(ref[:load][d]["pd"], sigma_pd),
    0.0, Inf
)

# Include both generation and demand uncertainty.
a1 = net_quantile(gen_distribution, load_distribution, tail_minus)
a2 = net_quantile(gen_distribution, load_distribution, 1 - tail_plus)

bus_bounds = Dict(
    study_bus => Dict("a1" => a1, "a2" => a2)
)

# Fixed active and reactive power tolerances, in p.u.
epsilon = 1.0

bus_epsilon = Dict(
    study_bus => Dict(
        "p_minus" => epsilon,
        "p_plus"  => epsilon,
        "q_minus" => epsilon,
        "q_plus"  => epsilon
    )
)

println("Active net-injection bounds: ", (a1, a2))
###############################################################################
# 18. Initialize and Solve the Model
###############################################################################

# Use a fresh model for this experiment.
model = init_ac_cc()
result = solve_model_ccac!(model, ref, uncertainty, bus_bounds, bus_epsilon)


push!(costs, isnothing(result[:objective]) ? NaN : result[:objective])
###############################################################################
###############################################################################
# 19. Print Cost and Comparison-Bus Results
###############################################################################

println("Status: ", result[:status])

if result[:objective] !== nothing
    println("Baseline cost excluding renewable: ", baseline_other_cost)
    println("New model cost: ", result[:objective])
    println("Cost difference: ",
            result[:objective] - baseline_other_cost)

    for bus_id in comparison_buses
        # Total active generation at this comparison bus.
        new_pg = sum(
            result[:pg][g] for g in ref[:bus_gens][bus_id];
            init = 0.0
        )

        println("\nBus $bus_id:")
        println("Pg: ", new_pg)
        println("Voltage magnitude: ", result[:vm][bus_id])
        println("Voltage angle: ", result[:va][bus_id])

        println("Change in Pg: ", new_pg - baseline_pg[bus_id])
        println("Change in voltage magnitude: ",
                result[:vm][bus_id] - baseline_vm[bus_id])
        println("Change in voltage angle: ",
                result[:va][bus_id] - baseline_va[bus_id])
    end
else
    println("No successful solution; comparison unavailable.")
end



###############################################################################
end # Close generation sigma loop

include(joinpath(@__DIR__, "/Users/shidratulmuntaha/power-annex/src/uncertainty-org/B-plot.jl"))