## Bismillah
###############################################################################
# File: solve_ac_cc.jl
# Author: Shidratul Muntaha
# Research: Active-Power Uncertainty in AC Chance-Constrained OPF
# Purpose: Build and solve the current proposed model; return status and cost.
###############################################################################

using PowerModels
using Ipopt
using JuMP

# Include util_cc.jl before calling this function.
# The caller supplies a fresh model and the prepared bounds and tolerances.
# Baseline comparison and user inputs belong in the test file.
function solve_model_ccac!(model, ref, uncertainty, bus_bounds, bus_epsilon)
    uncertain_buses = group_uncertain_components(ref, uncertainty)

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
        loads = [ref[:load][id] for id in ref[:bus_loads][i]]
        shunts = [ref[:shunt][id] for id in ref[:bus_shunts][i]]

        # Only the active balance is replaced at uncertain buses.
        if !haskey(uncertain_buses, i)
            # Active power balance.
            @constraint(model,
                sum(p[a] for a in ref[:bus_arcs][i]; init = 0.0) +
                sum(p_dc[a] for a in ref[:bus_arcs_dc][i]; init = 0.0) ==
                sum(pg[g] for g in ref[:bus_gens][i]; init = 0.0) -
                sum(l["pd"] for l in loads; init = 0.0) -
                sum(s["gs"] for s in shunts; init = 0.0) * vm[i]^2
            )
        end

        # Deterministic reactive balance at every bus; shunts remain included.
        @constraint(model,
            sum(q[a] for a in ref[:bus_arcs][i]; init = 0.0) +
            sum(q_dc[a] for a in ref[:bus_arcs_dc][i]; init = 0.0) ==
            sum(qg[g] for g in ref[:bus_gens][i]; init = 0.0) -
            sum(l["qd"] for l in loads; init = 0.0) +
            sum(s["bs"] for s in shunts; init = 0.0) * vm[i]^2
        )
    end

    ###############################################################################
    # 18. Add the Proposed Active-Power Inequalities and Solve
    ###############################################################################

    for i in keys(uncertain_buses)
        bounds = bus_bounds[i]
        eps = bus_epsilon[i]
       

        Pg = @expression(model,
            sum(pg[g] for g in ref[:bus_gens][i]; init = 0.0)
        )

       
###33change

        @constraint(model, Pg - eps["p_minus"] <= bounds["a1"])
        @constraint(model, Pg + eps["p_plus"] >= bounds["a2"])
        
    end

    # Show Ipopt's diagnostic output.
    set_optimizer_attribute(model, "print_level", 5)

    optimize!(model)

    ###############################################################################
    # 19. Return the Solution Dictionary
    ###############################################################################

    # Store an objective only when the solver reports success.
    result = Dict{Symbol, Any}(
        :status => termination_status(model),
        :objective => nothing
    )

    if result[:status] in (JuMP.MOI.OPTIMAL, JuMP.MOI.LOCALLY_SOLVED)
        result[:objective] = objective_value(model)
    end

    return result
end
