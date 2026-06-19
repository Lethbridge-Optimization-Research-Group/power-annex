function set_model_variables!(power_flow_model::AbstractMPOPFModel, factory::DCMPOPFSearchFactory)
    model = power_flow_model.model
    T = power_flow_model.time_periods
    ref = PowerModels.build_ref(power_flow_model.data)[:it][:pm][:nw][0]
    bus_data = ref[:bus]
    gen_data = ref[:gen]
    ramp_data = power_flow_model.ramping_data["ramp_limits"]
    
    @variable(model, va[t in 1:T, i in keys(bus_data)])
    @variable(model, gen_data[i]["pmin"] <= pg[t in 1:T, i in keys(gen_data)] <= gen_data[i]["pmax"])
    # @variable(model, -branch_data[l]["rate_a"] <= p[t in 1:T, (l,i,j) in ref[:arcs]] <= branch_data[l]["rate_a"])
    @variable(model, -ref[:branch][l]["rate_a"] <= p[1:T,(l,i,j) in ref[:arcs_from]] <= ref[:branch][l]["rate_a"])
    @variable(model, 0 <= ramp_up[t in 2:T, g in keys(gen_data)] <= ramp_data[g])
    @variable(model, 0 <= ramp_down[t in 2:T, g in keys(gen_data)] <= ramp_data[g])
end

function set_model_objective_function!(power_flow_model::AbstractMPOPFModel, factory::DCMPOPFSearchFactory)
    model = power_flow_model.model
    data = power_flow_model.data
    T = power_flow_model.time_periods
    ramping_data = power_flow_model.ramping_data
    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
    pg = model[:pg]
    ramp_up = model[:ramp_up]
    ramp_down = model[:ramp_down]
    
    @objective(model, Min,
    sum(sum(ref[:gen][g]["cost"][1]*pg[t,g]^2 + ref[:gen][g]["cost"][2]*pg[t,g] + ref[:gen][g]["cost"][3] for g in keys(ref[:gen])) for t in 1:T) +
    sum(ramping_data["costs"][g] * (ramp_up[t, g] + ramp_down[t, g]) for g in keys(ref[:gen]) for t in 2:T) 
    )
end

function set_model_constraints!(power_flow_model::AbstractMPOPFModel, factory::DCMPOPFSearchFactory)
    model = power_flow_model.model
    data = power_flow_model.data
    T = power_flow_model.time_periods
    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
    gen_data = ref[:gen]
    load_data = ref[:load]
    va = model[:va]
    p = model[:p]
    pg = model[:pg]
    demands = power_flow_model.demands
    ramp_up = model[:ramp_up]
    ramp_down = model[:ramp_down]

    p_expr = Dict(t => Dict() for t in 1:T)

    for t in 1:T
        for (l, i, j) in ref[:arcs_from]
            p_expr[t][(l, i, j)] = 1.0 * p[t, (l, i, j)]
            p_expr[t][(l, j, i)] = -1.0 * p[t, (l, i, j)]
        end
    end

    for t in 1:T
        # Reference bus constraint
        for (i, bus) in ref[:ref_buses]
            @constraint(model, va[t,i] == 0)
        end

        # Bus power balance constraint
        for (i, bus) in ref[:bus]
            bus_loads = [load_data[l] for l in ref[:bus_loads][i]]
            bus_shunts = [ref[:shunt][s] for s in ref[:bus_shunts][i]]
            
            @constraint(model,
                sum(p_expr[t][a] for a in ref[:bus_arcs][i]) <=
                sum(pg[t, g] for g in ref[:bus_gens][i]) -
                sum(get(demands[t], i, 0)) - #epsilon
                sum(shunt["gs"] for shunt in bus_shunts) * 1.0^2
            )
        end

        # Branch power flow constraints
        for (i, branch) in ref[:branch]
            f_idx = (i, branch["f_bus"], branch["t_bus"])
            p_fr = p[t,f_idx]
            va_fr = va[t,branch["f_bus"]]
            va_to = va[t,branch["t_bus"]]
            
            g, b = PowerModels.calc_branch_y(branch)
            
            @constraint(model, p_fr == -b*(va_fr - va_to))
            @constraint(model, va_fr - va_to <= branch["angmax"])
            @constraint(model, va_fr - va_to >= branch["angmin"])
        end
    end

    # Generator ramping constraints
    for g in keys(gen_data)
        for t in 2:T
            @constraint(model, ramp_up[t, g] >= pg[t, g] - pg[t-1, g])
            @constraint(model, ramp_down[t, g] >= pg[t-1, g] - pg[t, g])
        end
    end
end