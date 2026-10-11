#### AC Optimal Power Flow ####

# This file provides a pedagogical example of modeling the AC Optimal Power
# Flow problem using the Julia Mathematical Programming package (JuMP) and the
# PowerModels package for data parsing.

# This file can be run by calling `include("ac-opf.jl")` from the Julia REPL or
# by calling `julia ac-opf.jl` in Julia v1.

# Developed by Line Roald (@lroald) and Carleton Coffrin (@ccoffrin)


###############################################################################
# 0. Initialization
###############################################################################

# Load Julia Packages
#--------------------
using PowerModels
using Ipopt
using JuMP
using Distributions


# Load System Data
# ----------------
powermodels_path = joinpath(dirname(pathof(PowerModels)), "..")

file_name = "/Users/shidratulmuntaha/power-annex/.github/workflows/cases/case39.m"
# note: change this string to modify the network data that will be loaded

# load the data file
data = PowerModels.parse_file(file_name)

# Add zeros to turn linear objective functions into quadratic ones
# so that additional parameter checks are not required
PowerModels.standardize_cost_terms!(data, order=2)

# Adds reasonable rate_a values to branches without them
PowerModels.calc_thermal_limits!(data)

# ============================================================
# STEP 1: NORMAL AC-OPF BASELINE
# ============================================================

baseline = PowerModels.solve_ac_opf(data, Ipopt.Optimizer)

baseline_status = baseline["termination_status"]
baseline_cost = baseline["objective"]

println("==============================================")
println("BASELINE AC-OPF")
println("Termination status = ", baseline_status)
println("Baseline cost C* = ", baseline_cost)
println("==============================================")

# use build_ref to filter out inactive components
ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]

# Reactive demand at Bus 39
bus_cc = 39

qd39 = sum(
    ref[:load][l]["qd"]
    for l in ref[:bus_loads][bus_cc]
)

println("Reactive demand Qd at Bus 39 = ", qd39)
# note: ref contains all the relevant system parameters needed to build the OPF model
# When we introduce constraints and variable bounds below, we use the parameters in ref.

# Reactive power uncertainty
mu_qd = qd39
sigma_qd = 1.0

# Confidence level
alpha = 0.95
alpha_prime = 1.0 - alpha

# Normal quantile
z = quantile(
    Normal(0, 1),
    1.0 - alpha_prime / 2
)

# Reactive power confidence limits
b1 = mu_qd - z * sigma_qd
b2 = mu_qd + z * sigma_qd

println("mu_Qd = ", mu_qd)
println("sigma_Qd = ", sigma_qd)
println("z = ", z)
println("b1 = ", b1)
println("b2 = ", b2)


## “For Bus 39, the case gives a reactive demand of 2.5 p.u., so I used that as μ Qd​.I assumed σQd=1 as we discussed. With a 95% confidence level, the normal quantile is 1.96. Therefore, I calculated b1=0.54 and b2=4.46. So the 95% confidence interval for the uncertain reactive demand is approximately 0.54 to 4.46 p.u.

# Step 3B: Choose epsilon-minus
epsilon_minus = 0.005 * abs(mu_qd)

println("epsilon_minus = ", epsilon_minus)

# Step 3B: Choose epsilon-plus
epsilon_plus = max(
    0.0,
    b2 - b1 - epsilon_minus
)

println("epsilon_plus = ", epsilon_plus)

###############################################################################
# 1. Building the Optimal Power Flow Model
###############################################################################

# Initialize a JuMP Optimization Model
#-------------------------------------
model = Model(Ipopt.Optimizer)

set_optimizer_attribute(model, "print_level", 0)
# note: print_level changes the amount of solver information printed to the terminal


# Add Optimization and State Variables
# ------------------------------------

# Add voltage angles va for each bus
@variable(model, va[i in keys(ref[:bus])])
# note: [i in keys(ref[:bus])] adds one `va` variable for each bus in the network

# Add voltage angles vm for each bus
@variable(model, ref[:bus][i]["vmin"] <= vm[i in keys(ref[:bus])] <= ref[:bus][i]["vmax"], start=1.0)
# note: this vairable also includes the voltage magnitude limits and a starting value

# Add active power generation variable pg for each generator (including limits)
@variable(model, ref[:gen][i]["pmin"] <= pg[i in keys(ref[:gen])] <= ref[:gen][i]["pmax"])
# Add reactive power generation variable qg for each generator (including limits)
@variable(model, ref[:gen][i]["qmin"] <= qg[i in keys(ref[:gen])] <= ref[:gen][i]["qmax"])

# Add power flow variables p to represent the active power flow for each branch
@variable(model, -ref[:branch][l]["rate_a"] <= p[(l,i,j) in ref[:arcs]] <= ref[:branch][l]["rate_a"])
# Add power flow variables q to represent the reactive power flow for each branch
@variable(model, -ref[:branch][l]["rate_a"] <= q[(l,i,j) in ref[:arcs]] <= ref[:branch][l]["rate_a"])
# note: ref[:arcs] includes both the from (i,j) and the to (j,i) sides of a branch

# Add power flow variables p_dc to represent the active power flow for each HVDC line
@variable(model, p_dc[a in ref[:arcs_dc]])
# Add power flow variables q_dc to represent the reactive power flow at each HVDC terminal
@variable(model, q_dc[a in ref[:arcs_dc]])

for (l,dcline) in ref[:dcline]
    f_idx = (l, dcline["f_bus"], dcline["t_bus"])
    t_idx = (l, dcline["t_bus"], dcline["f_bus"])

    JuMP.set_lower_bound(p_dc[f_idx], dcline["pminf"])
    JuMP.set_upper_bound(p_dc[f_idx], dcline["pmaxf"])
    JuMP.set_lower_bound(q_dc[f_idx], dcline["qminf"])
    JuMP.set_upper_bound(q_dc[f_idx], dcline["qmaxf"])

    JuMP.set_lower_bound(p_dc[t_idx], dcline["pmint"])
    JuMP.set_upper_bound(p_dc[t_idx], dcline["pmaxt"])
    JuMP.set_lower_bound(q_dc[t_idx], dcline["qmint"])
    JuMP.set_upper_bound(q_dc[t_idx], dcline["qmaxt"])
end


# Add Objective Function
# ----------------------

# index representing which side the HVDC line is starting
from_idx = Dict(arc[1] => arc for arc in ref[:arcs_from_dc])

# Minimize the cost of active power generation and cost of HVDC line usage
# assumes costs are given as quadratic functions
@objective(model, Min,
    sum(gen["cost"][1]*pg[i]^2 + gen["cost"][2]*pg[i] + gen["cost"][3] for (i,gen) in ref[:gen]) +
    sum(dcline["cost"][1]*p_dc[from_idx[i]]^2 + dcline["cost"][2]*p_dc[from_idx[i]] + dcline["cost"][3] for (i,dcline) in ref[:dcline])
)


# Add Constraints
# ---------------

# Fix the voltage angle to zero at the reference bus
for (i,bus) in ref[:ref_buses]
    @constraint(model, va[i] == 0)
end

# Nodal power balance constraints
for (i,bus) in ref[:bus]
    # Build a list of the loads and shunt elements connected to the bus i
    bus_loads = [ref[:load][l] for l in ref[:bus_loads][i]]
    bus_shunts = [ref[:shunt][s] for s in ref[:bus_shunts][i]]

    # Active power balance at node i
    @constraint(model,
        sum(p[a] for a in ref[:bus_arcs][i]) +                  # sum of active power flow on lines from bus i +
        sum(p_dc[a_dc] for a_dc in ref[:bus_arcs_dc][i]) ==     # sum of active power flow on HVDC lines from bus i =
        sum(pg[g] for g in ref[:bus_gens][i]) -                 # sum of active power generation at bus i -
        sum(load["pd"] for load in bus_loads) -                 # sum of active load consumption at bus i -
        sum(shunt["gs"] for shunt in bus_shunts)*vm[i]^2        # sum of active shunt element injections at bus i
    )

    # Reactive power balance at node i
    # Reactive power balance / chance constraint
# Reactive power balance / chance constraint
if i == bus_cc

    # Reactive generation at Bus 39
    qg_bus39 = sum(qg[g] for g in ref[:bus_gens][i]; init=0.0)

    # Reactive power flowing through AC branches
    qflow_bus39 = sum(q[a] for a in ref[:bus_arcs][i]; init=0.0)

    # Reactive power flowing through DC lines
    qdc_bus39 = sum(q_dc[a_dc] for a_dc in ref[:bus_arcs_dc][i]; init=0.0)

    # Reactive shunt contribution
    qshunt_bus39 =
        sum(shunt["bs"] for shunt in bus_shunts; init=0.0) * vm[i]^2

    # Qg - Qtrans
    qtrans_bus39 =
        qflow_bus39 + qdc_bus39 - qshunt_bus39

    # Lower chance constraint
    @constraint(model,
        qg_bus39 - qtrans_bus39 - epsilon_minus <= b1
    )

    # Upper chance constraint
    @constraint(model,
        qg_bus39 - qtrans_bus39 + epsilon_plus >= b2
    )

else

    # Original reactive power balance
    @constraint(model,
        sum(q[a] for a in ref[:bus_arcs][i]) +
        sum(q_dc[a_dc] for a_dc in ref[:bus_arcs_dc][i]) ==
        sum(qg[g] for g in ref[:bus_gens][i]) -
        sum(load["qd"] for load in bus_loads) +
        sum(shunt["bs"] for shunt in bus_shunts) * vm[i]^2
    )

end
end
# Branch power flow physics and limit constraints
for (i,branch) in ref[:branch]
    # Build the from variable id of the i-th branch, which is a tuple given by (branch id, from bus, to bus)
    f_idx = (i, branch["f_bus"], branch["t_bus"])
    # Build the to variable id of the i-th branch, which is a tuple given by (branch id, to bus, from bus)
    t_idx = (i, branch["t_bus"], branch["f_bus"])
    # note: it is necessary to distinguish between the from and to sides of a branch due to power losses

    p_fr = p[f_idx]                     # p_fr is a reference to the optimization variable p[f_idx]
    q_fr = q[f_idx]                     # q_fr is a reference to the optimization variable q[f_idx]
    p_to = p[t_idx]                     # p_to is a reference to the optimization variable p[t_idx]
    q_to = q[t_idx]                     # q_to is a reference to the optimization variable q[t_idx]
    # note: adding constraints to p_fr is equivalent to adding constraints to p[f_idx], and so on

    vm_fr = vm[branch["f_bus"]]         # vm_fr is a reference to the optimization variable vm on the from side of the branch
    vm_to = vm[branch["t_bus"]]         # vm_to is a reference to the optimization variable vm on the to side of the branch
    va_fr = va[branch["f_bus"]]         # va_fr is a reference to the optimization variable va on the from side of the branch
    va_to = va[branch["t_bus"]]         # va_fr is a reference to the optimization variable va on the to side of the branch

    # Compute the branch parameters and transformer ratios from the data
    g, b = PowerModels.calc_branch_y(branch)
    tr, ti = PowerModels.calc_branch_t(branch)
    g_fr = branch["g_fr"]
    b_fr = branch["b_fr"]
    g_to = branch["g_to"]
    b_to = branch["b_to"]
    tm = branch["tap"]^2
    # note: tap is assumed to be 1.0 on non-transformer branches


    # AC Power Flow Constraints

    # From side of the branch flow
    @constraint(model, p_fr ==  (g+g_fr)/tm*vm_fr^2 + (-g*tr+b*ti)/tm*(vm_fr*vm_to*cos(va_fr-va_to)) + (-b*tr-g*ti)/tm*(vm_fr*vm_to*sin(va_fr-va_to)) )
    @constraint(model, q_fr == -(b+b_fr)/tm*vm_fr^2 - (-b*tr-g*ti)/tm*(vm_fr*vm_to*cos(va_fr-va_to)) + (-g*tr+b*ti)/tm*(vm_fr*vm_to*sin(va_fr-va_to)) )

    # To side of the branch flow
    @constraint(model, p_to ==  (g+g_to)*vm_to^2 + (-g*tr-b*ti)/tm*(vm_to*vm_fr*cos(va_to-va_fr)) + (-b*tr+g*ti)/tm*(vm_to*vm_fr*sin(va_to-va_fr)) )
    @constraint(model, q_to == -(b+b_to)*vm_to^2 - (-b*tr+g*ti)/tm*(vm_to*vm_fr*cos(va_fr-va_to)) + (-g*tr-b*ti)/tm*(vm_to*vm_fr*sin(va_to-va_fr)) )

    # Voltage angle difference limit
    @constraint(model, va_fr - va_to <= branch["angmax"])
    @constraint(model, va_fr - va_to >= branch["angmin"])

    # Apparent power limit, from side and to side
    @constraint(model, p_fr^2 + q_fr^2 <= branch["rate_a"]^2)
    @constraint(model, p_to^2 + q_to^2 <= branch["rate_a"]^2)
end

# HVDC line constraints
for (i,dcline) in ref[:dcline]
    # Build the from variable id of the i-th HVDC line, which is a tuple given by (hvdc line id, from bus, to bus)
    f_idx = (i, dcline["f_bus"], dcline["t_bus"])
    # Build the to variable id of the i-th HVDC line, which is a tuple given by (hvdc line id, to bus, from bus)
    t_idx = (i, dcline["t_bus"], dcline["f_bus"])   # index of the ith HVDC line which is a tuple given by (line number, to bus, from bus)
    # note: it is necessary to distinguish between the from and to sides of a HVDC line due to power losses

    # Constraint defining the power flow and losses over the HVDC line
    @constraint(model, (1-dcline["loss1"])*p_dc[f_idx] + (p_dc[t_idx] - dcline["loss0"]) == 0)
end



###############################################################################
# 3. Solve the Optimal Power Flow Model and Review the Results
###############################################################################

# Solve the optimization problem
optimize!(model)

# Check that the solver terminated without an error
println("The solver termination status is $(termination_status(model))")

# Check the value of the objective function
cost = objective_value(model)
println("The cost of generation is $(cost).")


# Check the value of an optimization variable
# Example: Active power generated at generator 1
pg1 = value(pg[1])
println("The active power generated at generator 1 is $(pg1*ref[:baseMVA]) MW.")
# note: the optimization model is in per unit, so the baseMVA value is used to restore the physical units
