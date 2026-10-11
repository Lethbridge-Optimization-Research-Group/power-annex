###############################################################################
# ACTIVE POWER CHANCE-CONSTRAINED AC-OPF
# IEEE 39-BUS SYSTEM
###############################################################################

using PowerModels
using Ipopt
using JuMP
using Distributions


###############################################################################
# 0. INITIALIZATION
###############################################################################

file_name = "/Users/shidratulmuntaha/power-annex/.github/workflows/cases/case39.m"

# Load case
data = PowerModels.parse_file(file_name)

# Make all generator cost functions quadratic
PowerModels.standardize_cost_terms!(data, order=2)

# Add thermal limits
PowerModels.calc_thermal_limits!(data)


###############################################################################
# 1. BUILD REFERENCE
###############################################################################

ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]


###############################################################################
# 2. BASELINE AC-OPF
###############################################################################

# Create the model
model = Model(Ipopt.Optimizer)

set_optimizer_attribute(model, "print_level", 0)


###############################################################################
# 3. VARIABLES
###############################################################################

# Voltage angle
@variable(model, va[i in keys(ref[:bus])])

# Voltage magnitude
@variable(
    model,
    ref[:bus][i]["vmin"] <= vm[i in keys(ref[:bus])] <= ref[:bus][i]["vmax"],
    start=1.0
)

# Active generation
@variable(
    model,
    ref[:gen][i]["pmin"] <= pg[i in keys(ref[:gen])] <= ref[:gen][i]["pmax"]
)

# Reactive generation
@variable(
    model,
    ref[:gen][i]["qmin"] <= qg[i in keys(ref[:gen])] <= ref[:gen][i]["qmax"]
)

# Active power flow
@variable(
    model,
    -ref[:branch][l]["rate_a"] <=
    p[(l,i,j) in ref[:arcs]] <=
    ref[:branch][l]["rate_a"]
)

# Reactive power flow
@variable(
    model,
    -ref[:branch][l]["rate_a"] <=
    q[(l,i,j) in ref[:arcs]] <=
    ref[:branch][l]["rate_a"]
)

# HVDC active power flow
@variable(model, p_dc[a in ref[:arcs_dc]])

# HVDC reactive power flow
@variable(model, q_dc[a in ref[:arcs_dc]])


###############################################################################
# 4. HVDC LIMITS
###############################################################################

for (l, dcline) in ref[:dcline]

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


###############################################################################
# 5. OBJECTIVE FUNCTION
###############################################################################

from_idx = Dict(
    arc[1] => arc
    for arc in ref[:arcs_from_dc]
)

@objective(
    model,
    Min,

    sum(
        gen["cost"][1] * pg[i]^2 +
        gen["cost"][2] * pg[i] +
        gen["cost"][3]
        for (i, gen) in ref[:gen]
    )

    +

    sum(
        dcline["cost"][1] * p_dc[from_idx[i]]^2 +
        dcline["cost"][2] * p_dc[from_idx[i]] +
        dcline["cost"][3]
        for (i, dcline) in ref[:dcline]
    )
)


###############################################################################
# 6. REFERENCE BUS
###############################################################################

for (i, bus) in ref[:ref_buses]

    @constraint(model, va[i] == 0)

end


###############################################################################
# 7. NORMAL ACTIVE AND REACTIVE POWER BALANCE
###############################################################################

for (i, bus) in ref[:bus]

    # Loads connected to bus i
    bus_loads = [
        ref[:load][l]
        for l in ref[:bus_loads][i]
    ]

    # Shunts connected to bus i
    bus_shunts = [
        ref[:shunt][s]
        for s in ref[:bus_shunts][i]
    ]


    ###########################################################################
    # ACTIVE POWER BALANCE
    #
    # This remains at EVERY bus, including Bus 39.
    ###########################################################################

    @constraint(
        model,

        sum(
            p[a]
            for a in ref[:bus_arcs][i]
        )

        +

        sum(
            p_dc[a_dc]
            for a_dc in ref[:bus_arcs_dc][i]
        )

        ==

        sum(
            pg[g]
            for g in ref[:bus_gens][i]
        )

        -

        sum(
            load["pd"]
            for load in bus_loads
        )

        -

        sum(
            shunt["gs"]
            for shunt in bus_shunts
        ) * vm[i]^2
    )


    ###########################################################################
    # REACTIVE POWER BALANCE
    #
    # Keep normal reactive power balance because this code is for
    # ACTIVE power chance constraint.
    ###########################################################################

    @constraint(
        model,

        sum(
            q[a]
            for a in ref[:bus_arcs][i]
        )

        +

        sum(
            q_dc[a_dc]
            for a_dc in ref[:bus_arcs_dc][i]
        )

        ==

        sum(
            qg[g]
            for g in ref[:bus_gens][i]
        )

        -

        sum(
            load["qd"]
            for load in bus_loads
        )

        +

        sum(
            shunt["bs"]
            for shunt in bus_shunts
        ) * vm[i]^2
    )

end


###############################################################################
# 8. BRANCH POWER FLOW EQUATIONS
###############################################################################

for (i, branch) in ref[:branch]

    # From side
    f_idx = (
        i,
        branch["f_bus"],
        branch["t_bus"]
    )

    # To side
    t_idx = (
        i,
        branch["t_bus"],
        branch["f_bus"]
    )


    # Branch variables
    p_fr = p[f_idx]
    q_fr = q[f_idx]

    p_to = p[t_idx]
    q_to = q[t_idx]


    # Voltage variables
    vm_fr = vm[branch["f_bus"]]
    vm_to = vm[branch["t_bus"]]

    va_fr = va[branch["f_bus"]]
    va_to = va[branch["t_bus"]]


    # Branch parameters
    g, b = PowerModels.calc_branch_y(branch)

    tr, ti = PowerModels.calc_branch_t(branch)

    g_fr = branch["g_fr"]
    b_fr = branch["b_fr"]

    g_to = branch["g_to"]
    b_to = branch["b_to"]

    tm = branch["tap"]^2


    ###########################################################################
    # FROM SIDE
    ###########################################################################

    @constraint(
        model,

        p_fr ==

        (g + g_fr) / tm * vm_fr^2

        +

        (-g * tr + b * ti) / tm *
        (
            vm_fr * vm_to *
            cos(va_fr - va_to)
        )

        +

        (-b * tr - g * ti) / tm *
        (
            vm_fr * vm_to *
            sin(va_fr - va_to)
        )
    )


    @constraint(
        model,

        q_fr ==

        -(b + b_fr) / tm * vm_fr^2

        -

        (-b * tr - g * ti) / tm *
        (
            vm_fr * vm_to *
            cos(va_fr - va_to)
        )

        +

        (-g * tr + b * ti) / tm *
        (
            vm_fr * vm_to *
            sin(va_fr - va_to)
        )
    )


    ###########################################################################
    # TO SIDE
    ###########################################################################

    @constraint(
        model,

        p_to ==

        (g + g_to) * vm_to^2

        +

        (-g * tr - b * ti) / tm *
        (
            vm_to * vm_fr *
            cos(va_to - va_fr)
        )

        +

        (-b * tr + g * ti) / tm *
        (
            vm_to * vm_fr *
            sin(va_to - va_fr)
        )
    )


    @constraint(
        model,

        q_to ==

        -(b + b_to) * vm_to^2

        -

        (-b * tr + g * ti) / tm *
        (
            vm_to * vm_fr *
            cos(va_fr - va_to)
        )

        +

        (-g * tr - b * ti) / tm *
        (
            vm_to * vm_fr *
            sin(va_fr - va_to)
        )
    )


    ###########################################################################
    # VOLTAGE ANGLE LIMIT
    ###########################################################################

    @constraint(
        model,
        va_fr - va_to <= branch["angmax"]
    )

    @constraint(
        model,
        va_fr - va_to >= branch["angmin"]
    )


    ###########################################################################
    # THERMAL LIMIT
    ###########################################################################

    @constraint(
        model,
        p_fr^2 + q_fr^2 <= branch["rate_a"]^2
    )

    @constraint(
        model,
        p_to^2 + q_to^2 <= branch["rate_a"]^2
    )

end


###############################################################################
# 9. HVDC LINE CONSTRAINTS
###############################################################################

for (i, dcline) in ref[:dcline]

    f_idx = (
        i,
        dcline["f_bus"],
        dcline["t_bus"]
    )

    t_idx = (
        i,
        dcline["t_bus"],
        dcline["f_bus"]
    )


    @constraint(
        model,

        (1 - dcline["loss1"]) * p_dc[f_idx]
        +
        (p_dc[t_idx] - dcline["loss0"])
        == 0
    )

end


###############################################################################
# 10. SOLVE BASELINE
###############################################################################

optimize!(model)

baseline_status = termination_status(model)
baseline_cost = objective_value(model)


println()
println("==============================================")
println("BASELINE AC-OPF")
println("Termination status = ", baseline_status)
println("Baseline cost C* = ", baseline_cost)
println("==============================================")


###############################################################################
# 11. BUS 39 ACTIVE POWER
###############################################################################

bus_cc = 39


# Generators at Bus 39
gens_39 = ref[:bus_gens][bus_cc]


# Baseline active generation at Bus 39
Pg_star = sum(
    value(pg[g])
    for g in gens_39;
    init = 0.0
)


# Active demand at Bus 39
Pd_39 = sum(
    ref[:load][l]["pd"]
    for l in ref[:bus_loads][bus_cc];
    init = 0.0
)


###############################################################################
# 12. ACTIVE POWER UNCERTAINTY
###############################################################################

# Keep the same units as the case data/model
mu_Pg = Pg_star
mu_Pd = Pd_39

sigma_Pg = 1.0
sigma_Pd = 1.0


# Random variable
#
# X = Pg - Pd
#

mu_X = mu_Pg - mu_Pd

sigma_X = sqrt(
    sigma_Pg^2 +
    sigma_Pd^2
)


###############################################################################
# 13. 95% CONFIDENCE INTERVAL
###############################################################################

alpha = 0.95

alpha_prime = 1.0 - alpha

z = quantile(
    Normal(0, 1),
    1.0 - alpha_prime / 2
)


# Confidence limits
a1 = mu_X - z * sigma_X

a2 = mu_X + z * sigma_X


###############################################################################
# 14. CONVERT X LIMITS TO Pg LIMITS
###############################################################################

# a1 <= Pg - mu_Pd <= a2
#
# Therefore:
#
# mu_Pd + a1 <= Pg <= mu_Pd + a2

Pg_lower = mu_Pd + a1

Pg_upper = mu_Pd + a2


###############################################################################
# 15. EPSILON
###############################################################################
##we assumed a symmetric normal distribution

epsilon_minus = z * sigma_X

epsilon_plus = z * sigma_X


###############################################################################
# 16. PRINT CHANCE-CONSTRAINT INFORMATION
###############################################################################

println("Active demand Pd at Bus 39 = ", Pd_39)

println("mu_Pg = ", mu_Pg)

println("mu_Pd = ", mu_Pd)

println("sigma_Pg = ", sigma_Pg)

println("sigma_Pd = ", sigma_Pd)

println("mu_X = ", mu_X)

println("sigma_X = ", sigma_X)

println("z = ", z)

println("a1 = ", a1)

println("a2 = ", a2)

println("Pg lower limit = ", Pg_lower)

println("Pg upper limit = ", Pg_upper)

println("epsilon_minus = ", epsilon_minus)

println("epsilon_plus = ", epsilon_plus)


###############################################################################
# 17. ADD ACTIVE POWER CHANCE CONSTRAINT
###############################################################################

Pg_bus39 = sum(
    pg[g]
    for g in gens_39;
    init = 0.0
)


# Lower bound
@constraint(
    model,
    Pg_bus39 >= Pg_lower
)


# Upper bound
@constraint(
    model,
    Pg_bus39 <= Pg_upper
)


###############################################################################
# 18. SOLVE CHANCE-CONSTRAINED AC-OPF
###############################################################################

optimize!(model)


###############################################################################
# 19. FINAL RESULTS
###############################################################################

cc_status = termination_status(model)

cc_cost = objective_value(model)


# Final Pg at Bus 39
Pg_final = sum(
    value(pg[g])
    for g in gens_39;
    init = 0.0
)


# Final X
X_final = Pg_final - mu_Pd


# Cost difference
delta_C = cc_cost - baseline_cost


# Percentage cost change
percent_change =
    100 * delta_C / baseline_cost


###############################################################################
# 20. PRINT FINAL OUTPUT
###############################################################################

println()
println("==============================================")
println("CHANCE-CONSTRAINED AC-OPF")

println("Termination status = ", cc_status)

println("Chance-constrained cost C_CC = ", cc_cost)

println("Final active generation Pg at Bus 39 = ", Pg_final)

println("Required Pg lower bound = ", Pg_lower)

println("Required Pg upper bound = ", Pg_upper)

println("Final X = Pg - mu_Pd = ", X_final)

println("Cost difference ΔC = ", delta_C)

println("Cost change (%) = ", percent_change, "%")


###############################################################################
# 21. CHECK CHANCE CONSTRAINT
###############################################################################

if Pg_final >= Pg_lower && Pg_final <= Pg_upper

    println("Chance constraint is SATISFIED.")

else

    println("Chance constraint is NOT SATISFIED.")

end

println("==============================================")