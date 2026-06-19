function set_model_variables!(power_flow_model::AbstractMPOPFModel, factory::ACMPOPFSearchFactory)
    model = power_flow_model.model
    T = power_flow_model.time_periods
    ref = PowerModels.build_ref(power_flow_model.data)[:it][:pm][:nw][0]
    bus_data = ref[:bus]
    gen_data = ref[:gen]
    branch_data = ref[:branch]
    ramp_data = power_flow_model.ramping_data["ramp_limits"]
    
    @variable(model, va[t in 1:T, i in keys(bus_data)])
    @variable(model, bus_data[i]["vmin"] <= vm[t in 1:T, i in keys(bus_data)] <= bus_data[i]["vmax"], start=1.0)
    @variable(model, gen_data[i]["pmin"] <= pg[t in 1:T, i in keys(gen_data)] <= gen_data[i]["pmax"])
    @variable(model, gen_data[i]["qmin"] <= qg[t in 1:T, i in keys(gen_data)] <= gen_data[i]["qmax"])
    @variable(model, -branch_data[l]["rate_a"] <= p[t in 1:T, (l,i,j) in ref[:arcs]] <= branch_data[l]["rate_a"])
    @variable(model, -branch_data[l]["rate_a"] <= q[t in 1:T, (l,i,j) in ref[:arcs]] <= branch_data[l]["rate_a"])
    @variable(model, 0 <= ramp_up[t in 2:T, g in keys(gen_data)] <= ramp_data[g])
    @variable(model, 0 <= ramp_down[t in 2:T, g in keys(gen_data)] <= ramp_data[g])
end

function set_model_objective_function!(power_flow_model::AbstractMPOPFModel, factory::ACMPOPFSearchFactory)
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

function set_model_constraints!(power_flow_model::AbstractMPOPFModel, factory::ACMPOPFSearchFactory)
    model = power_flow_model.model
    data = power_flow_model.data
    T = power_flow_model.time_periods
    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]

    gen_data = ref[:gen]
    load_data = ref[:load]

    ramping_data = power_flow_model.ramping_data
    va = model[:va]
    p = model[:p]
    q = model[:q]
    pg = model[:pg]
    qg = model[:qg]
    vm = model[:vm]
	active_demands = power_flow_model.active_demands
    reactive_demands = power_flow_model.reactive_demands
    ramp_up = model[:ramp_up]
    ramp_down = model[:ramp_down]


    
	# power balance constraints for time periods
    for t in 1:T

		# 
        for (i, bus) in ref[:ref_buses]
            @constraint(model, va[t,i] == 0)
        end

		# 
        for (i, bus) in ref[:bus]
            bus_loads = [load_data[l] for l in ref[:bus_loads][i]]
            bus_shunts = [ref[:shunt][s] for s in ref[:bus_shunts][i]]

            active_bus_demand = get(active_demands[t], i, 0.0)
            reactive_bus_demand = get(reactive_demands[t], i, 0.0)

            @constraint(model,
                sum(p[t,a] for a in ref[:bus_arcs][i]) <=
                sum(pg[t, g] for g in ref[:bus_gens][i]) -
                active_bus_demand -
                sum(shunt["gs"] for shunt in bus_shunts)*vm[t,i]^2
            )

            @constraint(model,
                sum(q[t,a] for a in ref[:bus_arcs][i]) <=
                sum(qg[t, g] for g in ref[:bus_gens][i]) -
                reactive_bus_demand +
                sum(shunt["bs"] for shunt in bus_shunts)*vm[t,i]^2
            )
        end

		# active ( p ) and reactive ( q ) power constraints
        for (i, branch) in ref[:branch]
            f_idx = (i, branch["f_bus"], branch["t_bus"])
            t_idx = (i, branch["t_bus"], branch["f_bus"])

            p_to = p[t,t_idx]
            q_to = q[t,t_idx]
            p_fr = p[t,f_idx]
            q_fr = q[t,f_idx]

            va_fr = va[t,branch["f_bus"]]
            va_to = va[t,branch["t_bus"]]

            vm_fr = vm[t,branch["f_bus"]]
            vm_to = vm[t,branch["t_bus"]]

            g, b = PowerModels.calc_branch_y(branch)
            tr, ti = PowerModels.calc_branch_t(branch)
            ttm = tr^2 + ti^2

            g_fr = branch["g_fr"]
            b_fr = branch["b_fr"]
            g_to = branch["g_to"]
            b_to = branch["b_to"]

            @constraint(model, p_fr ==  (g+g_fr)/ttm*vm_fr^2 + (-g*tr+b*ti)/ttm*(vm_fr*vm_to*cos(va_fr-va_to)) + (-b*tr-g*ti)/ttm*(vm_fr*vm_to*sin(va_fr-va_to)) )
            @constraint(model, q_fr == -(b+b_fr)/ttm*vm_fr^2 - (-b*tr-g*ti)/ttm*(vm_fr*vm_to*cos(va_fr-va_to)) + (-g*tr+b*ti)/ttm*(vm_fr*vm_to*sin(va_fr-va_to)) )

            @constraint(model, p_to ==  (g+g_to)*vm_to^2 + (-g*tr-b*ti)/ttm*(vm_to*vm_fr*cos(va_to-va_fr)) + (-b*tr+g*ti)/ttm*(vm_to*vm_fr*sin(va_to-va_fr)) )
            @constraint(model, q_to == -(b+b_to)*vm_to^2 - (-b*tr+g*ti)/ttm*(vm_to*vm_fr*cos(va_to-va_fr)) + (-g*tr-b*ti)/ttm*(vm_to*vm_fr*sin(va_to-va_fr)) )

            @constraint(model, va_fr - va_to <= branch["angmax"])
            @constraint(model, va_fr - va_to >= branch["angmin"])

            @constraint(model, p_fr^2 + q_fr^2 <= branch["rate_a"]^2)
            @constraint(model, p_to^2 + q_to^2 <= branch["rate_a"]^2)
        end
    end

	# ramp up and ramp down constraints
    for g in keys(gen_data)
        for t in 2:T
            @constraint(model, ramp_up[t, g] >= pg[t, g] - pg[t-1, g])
            @constraint(model, ramp_down[t, g] >= pg[t-1, g] - pg[t, g])
        end
    end
end