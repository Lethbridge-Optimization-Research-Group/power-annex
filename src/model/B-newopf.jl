##Bismillah
###############################################################################
# B-newopf.jl
# General AC Chance-Constrained Optimal Power Flow
###############################################################################


###############################################################################
# 0. Initialization
###############################################################################

# Load Julia packages.
using PowerModels
using Ipopt
using JuMP
using Distributions


###############################################################################
# 1. Load and Process System Data
###############################################################################

# Select the power-system case.
file_name = "/Users/shidratulmuntaha/power-annex/.github/workflows/cases/case118.m"

# Read the case file.
data = PowerModels.parse_file(file_name)

# Convert linear generation-cost functions to quadratic form when needed.
PowerModels.standardize_cost_terms!(data, order = 2)

# Add reasonable thermal limits to branches that do not contain them.
PowerModels.calc_thermal_limits!(data)

# Build the PowerModels reference dictionary.
ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]

# Check the available components.
println("Loads: ", length(ref[:load]))
println("Generators: ", length(ref[:gen]))

###############################################################################
# 2. Solve the Baseline AC-OPF
###############################################################################

# Solve the case before adding uncertainty.
baseline = PowerModels.solve_ac_opf(data, Ipopt.Optimizer)

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

# Read and check a standard deviation.
function read_sigma(message)
    print(message)
    sigma = parse(Float64, strip(readline()))

    isfinite(sigma) && sigma >= 0 ||
        error("Sigma must be finite and nonnegative.")

    return sigma
end

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
        "exp_qd" => load["qd"],
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
        "exp_qg" => solution["qg"],
        "sigma_pg" => read_sigma("Sigma Pg: "),
       
    )
end

println("\nUncertainty dictionaries created.")

##here each load or generator represent its own uncertainty

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
###############################################################################
# 6. Choose Confidence and Tail Split
###############################################################################
###probabilities outside the lower and upper bounds

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
# 7. Define the Combined Sigma Function
###############################################################################

# Assumes the supplied uncertainties are independent.
function combined_sigma(sigmas)
    return sqrt(sum(sigma^2 for sigma in sigmas; init = 0.0))
end
###############################################################################
# 8. Define the Quantile-Bound Function
###############################################################################
## trancated normal Distributions
###############################################################################
# 8. Define Truncated-Normal Bounds
###############################################################################

###############################################################################
# 8. Define Truncated-Normal Bounds
###############################################################################

function normal_bounds(mu, sigma, low, high, tail_minus, tail_plus)
    d = truncated(Normal(mu, sigma), low, high)

    return quantile(d, tail_minus),
           quantile(d, 1 - tail_plus)
end
###############################################################################
# 9. Group Uncertain Components by Bus
###############################################################################

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

# Check the grouping.
for bus_id in sort(collect(keys(uncertain_buses)))
    println(
        "Bus ", bus_id,
        ": uncertain loads = ", uncertain_buses[bus_id][:load],
        ", uncertain generators = ", uncertain_buses[bus_id][:gen]
    )
end
###############################################################################
# 10. Prepare Bus Uncertainty Parameters
###############################################################################
##organizing those existing values by bus, because the OPF power-balance constraints are bus-based


bus_uncertainty = Dict()

for bus_id in keys(uncertain_buses)
    load_ids = uncertain_buses[bus_id][:load]
    gen_ids = uncertain_buses[bus_id][:gen]

    loads = [uncertainty[:load][id] for id in load_ids]
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
###############################################################################
# 11. Create the AC-CC Model and Variables
###############################################################################

model = Model(Ipopt.Optimizer)
set_optimizer_attribute(model, "print_level", 0)

# Bus voltage angles and magnitudes.
@variable(model, va[i in keys(ref[:bus])])

@variable(model,
    ref[:bus][i]["vmin"] <= vm[i in keys(ref[:bus])] <=
    ref[:bus][i]["vmax"], start = 1.0
)

# Generator active and reactive power.
@variable(model,
    ref[:gen][i]["pmin"] <= pg[i in keys(ref[:gen])] <=
    ref[:gen][i]["pmax"]
)

@variable(model,
    ref[:gen][i]["qmin"] <= qg[i in keys(ref[:gen])] <=
    ref[:gen][i]["qmax"]
)

# Active and reactive power flows at both ends of AC branches.
@variable(model,
    -ref[:branch][l]["rate_a"] <= p[(l,i,j) in ref[:arcs]] <=
    ref[:branch][l]["rate_a"]
)

@variable(model,
    -ref[:branch][l]["rate_a"] <= q[(l,i,j) in ref[:arcs]] <=
    ref[:branch][l]["rate_a"]
)

# HVDC terminal power flows.
@variable(model, p_dc[a in ref[:arcs_dc]])
@variable(model, q_dc[a in ref[:arcs_dc]])

###############################################################################
# 12. Add HVDC and Reference-Bus Constraints
###############################################################################

for (id, line) in ref[:dcline]
    f_idx = (id, line["f_bus"], line["t_bus"])
    t_idx = (id, line["t_bus"], line["f_bus"])

    # From-terminal limits.
    set_lower_bound(p_dc[f_idx], line["pminf"])
    set_upper_bound(p_dc[f_idx], line["pmaxf"])
    set_lower_bound(q_dc[f_idx], line["qminf"])
    set_upper_bound(q_dc[f_idx], line["qmaxf"])

    # To-terminal limits.
    set_lower_bound(p_dc[t_idx], line["pmint"])
    set_upper_bound(p_dc[t_idx], line["pmaxt"])
    set_lower_bound(q_dc[t_idx], line["qmint"])
    set_upper_bound(q_dc[t_idx], line["qmaxt"])

    # Active-power losses.
    @constraint(model,
        (1 - line["loss1"]) * p_dc[f_idx] +
        p_dc[t_idx] - line["loss0"] == 0
    )
end

# Reference-bus voltage angles.
for bus_id in keys(ref[:ref_buses])
    @constraint(model, va[bus_id] == 0)
end
###############################################################################
# 13. Add the Objective Function
###############################################################################

# Identify the from-terminal arc of each HVDC line.
from_idx = Dict(arc[1] => arc for arc in ref[:arcs_from_dc])

# Minimize generation and HVDC operating costs.
@objective(model, Min,
    sum(
        gen["cost"][1] * pg[id]^2 +
        gen["cost"][2] * pg[id] +
        gen["cost"][3]
        for (id, gen) in ref[:gen]
    ) +
    sum(
        line["cost"][1] * p_dc[from_idx[id]]^2 +
        line["cost"][2] * p_dc[from_idx[id]] +
        line["cost"][3]
        for (id, line) in ref[:dcline]
    )
)
###############################################################################
# 14. Add AC Branch Equations and Limits
###############################################################################

for (id, branch) in ref[:branch]
    i = branch["f_bus"]
    j = branch["t_bus"]

    f_idx = (id, i, j)
    t_idx = (id, j, i)

    # Branch and transformer parameters.
    g, b = PowerModels.calc_branch_y(branch)
    tr, ti = PowerModels.calc_branch_t(branch)
    tm = branch["tap"]^2

    g_fr = branch["g_fr"]
    b_fr = branch["b_fr"]
    g_to = branch["g_to"]
    b_to = branch["b_to"]

    # From-terminal active and reactive flows.
    @constraint(model,
        p[f_idx] == (g + g_fr)/tm * vm[i]^2 +
        (-g*tr + b*ti)/tm * vm[i]*vm[j]*cos(va[i] - va[j]) +
        (-b*tr - g*ti)/tm * vm[i]*vm[j]*sin(va[i] - va[j])
    )

    @constraint(model,
        q[f_idx] == -(b + b_fr)/tm * vm[i]^2 -
        (-b*tr - g*ti)/tm * vm[i]*vm[j]*cos(va[i] - va[j]) +
        (-g*tr + b*ti)/tm * vm[i]*vm[j]*sin(va[i] - va[j])
    )

    # To-terminal active and reactive flows.
    @constraint(model,
        p[t_idx] == (g + g_to) * vm[j]^2 +
        (-g*tr - b*ti)/tm * vm[j]*vm[i]*cos(va[j] - va[i]) +
        (-b*tr + g*ti)/tm * vm[j]*vm[i]*sin(va[j] - va[i])
    )

    @constraint(model,
    q[t_idx] == -(b + b_to) * vm[j]^2 -
    (-b*tr + g*ti)/tm * vm[j]*vm[i]*cos(va[j] - va[i]) +
    (-g*tr - b*ti)/tm * vm[j]*vm[i]*sin(va[j] - va[i])
)

    # Voltage-angle difference limits.
    @constraint(model, va[i] - va[j] <= branch["angmax"])
    @constraint(model, va[i] - va[j] >= branch["angmin"])

    # Apparent-power limits at both terminals.
    @constraint(model,
        p[f_idx]^2 + q[f_idx]^2 <= branch["rate_a"]^2
    )

    @constraint(model,
        p[t_idx]^2 + q[t_idx]^2 <= branch["rate_a"]^2
    )
end

###############################################################################
# 15. Add Deterministic Bus Power Balances
###############################################################################

for i in keys(ref[:bus])
    # Uncertain buses will be handled separately.
    haskey(uncertain_buses, i) && continue

    loads = [ref[:load][id] for id in ref[:bus_loads][i]]
    shunts = [ref[:shunt][id] for id in ref[:bus_shunts][i]]

    # Active power balance.
    @constraint(model,
        sum(p[a] for a in ref[:bus_arcs][i]; init = 0.0) +
        sum(p_dc[a] for a in ref[:bus_arcs_dc][i]; init = 0.0) ==
        sum(pg[g] for g in ref[:bus_gens][i]; init = 0.0) -
        sum(l["pd"] for l in loads; init = 0.0) -
        sum(s["gs"] for s in shunts; init = 0.0) * vm[i]^2
    )

    # Reactive power balance.
    @constraint(model,
        sum(q[a] for a in ref[:bus_arcs][i]; init = 0.0) +
        sum(q_dc[a] for a in ref[:bus_arcs_dc][i]; init = 0.0) ==
        sum(qg[g] for g in ref[:bus_gens][i]; init = 0.0) -
        sum(l["qd"] for l in loads; init = 0.0) +
        sum(s["bs"] for s in shunts; init = 0.0) * vm[i]^2
    )
end
###############################################################################
# 16. Enter and Check Power Tolerances
###############################################################################
###############################################################################
# 16. Calculate Truncated Bounds and Enter Tolerances
###############################################################################

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
###############################################################################
# 18. Add Proposed Inequalities with Fixed Baseline Means
###############################################################################

# Bus totals use baseline generator means and case/uncertainty load means.
# Variances include only selected uncertain components, assumed independent.
# These inequalities replace balances at uncertain buses as in the proposal.
###############################################################################
# 18. Add the Proposed Inequalities
###############################################################################

for i in keys(uncertain_buses)
    bounds = bus_bounds[i]
    eps = bus_epsilon[i]
   

    Pg = @expression(model,
        sum(pg[g] for g in ref[:bus_gens][i]; init = 0.0)
    )

   


    @constraint(model, Pg - eps["p_minus"] <= bounds["a1"])
    @constraint(model, Pg + eps["p_plus"] >= bounds["a2"])
    
end

# Show Ipopt's diagnostic output.
set_optimizer_attribute(model, "print_level", 5)

optimize!(model)

cc_status = termination_status(model)
println("Status: ", cc_status)

if cc_status in (JuMP.MOI.OPTIMAL, JuMP.MOI.LOCALLY_SOLVED)
    cc_cost = objective_value(model)

    println("Baseline cost: ", baseline_cost)
    println("New model cost: ", cc_cost)
    println("Cost difference: ", cc_cost - baseline_cost)

else
    println("No successful solution; cost comparison unavailable.")
end