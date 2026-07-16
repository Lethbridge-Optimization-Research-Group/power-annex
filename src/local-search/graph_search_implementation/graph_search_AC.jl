using Graphs, MetaGraphs, Gurobi, JuMP

#=
Refer to test_feasibility_AC when looking to modify how solutions are tested for feasability.
line 512 - Experiment on reactive power
=#

#=
- return actual models used in final solution
    - compare P, Q, V, theta from opt. model and solved model (plot)
- trial run with Qd staying the same across time periods
- double check how -Qd is handled
=#

#=
- uncomment reactive fixing constraint
- reintroduce checking ramp limits before adding edges
- double check logic for additional reactive demand scenarios
=#

"""
    AC_graph_search(data, factory, active_demands, reactive_demands, ramping_data, time_periods; max_it)

Create an AC graph model that iteratively adjusts generator values 
in order to form a solution.

# Arguments
- `data::Dict{String, Any}` : Powermodels parsed Matpower case data
- `factory::ACMPOPFSearchFactory` : Model factory for creating JuMP AC-OPF models
- `active_demands::Vector{Dict{Int64, Float64}}` : Active power demands for each time period
- `reactive_demands::Vector{Dict{Int64, Float64}}` : Reactive power demands for each time period
- `ramping_data::Dict{String, Any}` : Ramping costs and limits for each generator
- `time_periods::Int` : Number of time periods 
- `max_it::Int64` : A kwarg to specify the desired max iteration count (default 40)

# Returns 
- `info::Dict{Symbol, Any}` : Final model solution information and associated data.

Access with `info[:parameter]`
# Keys
`
:time
:graph
:path
:cost
:solution
:cost_history
:violations
:generation_cost
:ramping_cost
`
"""
function AC_graph_search(data::Dict{String, Any}, factory::ACMPOPFSearchFactory, active_demands::Vector{Dict{Int64, Float64}}, 
    reactive_demands::Vector{Dict{Int64, Float64}}, ramping_data::Dict{String, Any}, time_periods::Int64; 
    max_it::Int64 = 40)
    
    iteration = 1
    max_iterations = max_it
    
    start_time = time()

    search_parameters = Dict(
        :iteration => iteration,
        :cost_history => Vector{Float64}(),
        :total_active_demand => nothing,
        :total_reactive_demand => nothing,
        :total_active_generation => nothing,
        :total_reactive_generation => nothing,
        :data => data,
        :ramping_data => ramping_data
    )

    violations = Dict(
        :min_active_demand_not_met => 0,
        :min_reactive_demand_not_met => 0,
        :active_pmin_pmax_out_of_bounds => 0,
        :reactive_qmin_qmax_out_of_bounds => 0,
        :voltage_violations => 0,
        :infeasible_model => 0
    )

    # Find peak demand period considering both active and reactive power
    highest_demand = find_largest_time_period_AC(time_periods, active_demands, reactive_demands)
    largest_model = build_and_optimize_largest_period_AC(factory, active_demands[highest_demand], 
                                                        reactive_demands[highest_demand], ramping_data)
    status = termination_status(largest_model.model)
    println("Peak period optimization status: $status")
    
    if status != MOI.LOCALLY_SOLVED && status != MOI.OPTIMAL 
        error("Largest time period infeasible")
    end

    # Extract both active and reactive power values
    active_values = [value(largest_model.model[:pg][key]) for key in keys(largest_model.model[:pg])]
    reactive_values = [value(largest_model.model[:qg][key]) for key in keys(largest_model.model[:qg])]
    
    baseline_active_values = Dict(zip(largest_model.model[:pg].axes[2], active_values))
    baseline_reactive_values = Dict(zip(largest_model.model[:qg].axes[2], reactive_values))

    # Calculate total demands for search parameters
    search_parameters[:total_active_demand] = [sum(values(active_demands[i])) for i in 1:time_periods]
    search_parameters[:total_reactive_demand] = [sum(values(reactive_demands[i])) for i in 1:time_periods]
    
    # Initialize generation totals
    baseline_total_active = sum(values(baseline_active_values))
    baseline_total_reactive = sum(values(baseline_reactive_values))
    search_parameters[:total_active_generation] = fill(baseline_total_active, time_periods)
    search_parameters[:total_reactive_generation] = fill(baseline_total_reactive, time_periods)

    # Generate initial scenarios with both P and Q
    initial_scenarios_raw = generate_new_scenarios_subset_AC(baseline_active_values, baseline_reactive_values,
                                                           search_parameters, 1)

    scenarios, scenario_violations = test_scenarios_AC(data, factory, active_demands[highest_demand], 
                                                      reactive_demands[highest_demand], ramping_data, 
                                                      initial_scenarios_raw)
    
    # Update violation counts
    for (key, value) in scenario_violations
        violations[key] += value
    end

    graph = build_initial_graph_AC(scenarios, time_periods)
    add_weighted_edges_AC!(graph, time_periods, ramping_data)

    feasibility = false
    path = nothing
    
    while !feasibility
        path = shortest_path_AC(graph, time_periods)

        if path == false || path === nothing
            error("No feasible path found in the graph. The problem may be infeasible.")
        end

        infeasible_nodes = test_feasibility_AC(factory, path, graph, active_demands, reactive_demands, ramping_data)
        violations[:infeasible_model] += length(infeasible_nodes)

        if isempty(infeasible_nodes) 
            feasibility = true
        else
            # Remove infeasible nodes and rebuild connections
            for node in sort(infeasible_nodes, rev=true)
                rem_vertex!(graph, node)
            end
            add_weighted_edges_AC!(graph, time_periods, ramping_data)
        end
    end

    path_results = calculate_path_cost_AC(path, graph)
    
    best_graph = graph
    best_path = path
    best_cost = path_results[:total_cost]
    best_solution = extract_solution_AC(best_graph, best_path)
    
    # Update search parameters
    push!(search_parameters[:cost_history], best_cost)
    search_parameters[:total_active_generation] = [sum(values(best_solution[i][:active_generator_values])) for i in 1:time_periods]
    search_parameters[:total_reactive_generation] = [sum(values(best_solution[i][:reactive_generator_values])) for i in 1:time_periods]
    
    generation_cost = 0.0
    ramping_cost = 0.0

    # Main optimization loop
    while iteration < max_iterations
        current_active_values = Vector{Dict{Int64, Float64}}()
        current_reactive_values = Vector{Dict{Int64, Float64}}()
        new_scenarios = Vector{Vector{Any}}()

        # Extract current solution values
        for i in 1:time_periods
            active_vals = Dict()
            reactive_vals = Dict()
            
            for (gen_id, val) in best_solution[i][:active_generator_values]
                active_vals[gen_id] = val
            end
            for (gen_id, val) in best_solution[i][:reactive_generator_values]
                reactive_vals[gen_id] = val
            end
            
            push!(current_active_values, active_vals)
            push!(current_reactive_values, reactive_vals)
            push!(new_scenarios, Vector{Any}())
        end

        # Generate new scenarios for each time period
        for i in 1:time_periods
            scenarios_for_period =  generate_new_scenarios_subset_AC(current_active_values[i], 
                                                                   current_reactive_values[i],
                                                                   search_parameters, i)

            tested_scenarios, scenario_violations = test_scenarios_AC(data, factory, 
                                                                     active_demands[i], 
                                                                     reactive_demands[i],
                                                                     ramping_data, 
                                                                     scenarios_for_period)
            
            # Update violation counts
            for (key, value) in scenario_violations
                violations[key] += value
            end
            
            new_scenarios[i] = tested_scenarios
        end

        new_graph = build_new_graph_AC(new_scenarios, time_periods)
        add_weighted_edges_AC!(new_graph, time_periods, ramping_data)

        feasibility = false

        # Run shortest path, test feasibility only of the nodes in the path, then remove ones that don't work and rerun
        while !feasibility
            path = shortest_path_AC(new_graph, time_periods)

            if path == false || path === nothing
                error("No feasible path found in the graph. The problem may be infeasible.")
            end

            infeasible_nodes = test_feasibility_AC(factory, path, new_graph, active_demands, 
                                                  reactive_demands, ramping_data)
            violations[:infeasible_model] += length(infeasible_nodes)

            if isempty(infeasible_nodes) 
                feasibility = true
            else
                for node in infeasible_nodes
                    rem_vertex!(new_graph, node)
                end
            end
        end
        
        path_results = calculate_path_cost_AC(path, new_graph)

        if path_results[:total_cost] < best_cost
            best_graph = new_graph
            best_cost = path_results[:total_cost]
            best_path = path
            best_solution = extract_solution_AC(best_graph, best_path)
            generation_cost = path_results[:generation_cost]
            ramping_cost = path_results[:ramping_cost]
        end

        push!(search_parameters[:cost_history], best_cost)
        iteration += 1

        # Convergence check
        if iteration > 10
            recent_costs = search_parameters[:cost_history][end - 10:end-1]
            improvement = best_cost / maximum(recent_costs)
            
            popfirst!(search_parameters[:cost_history]) # Keep only the last 10 to save space

            if improvement > 0.999
                println("No improvement, stopping at $iteration iterations")
                break
            end
        end
    end

    info = Dict(
        :graph => best_graph,
        :path => best_path,
        :cost => best_cost,
        :solution => best_solution,
        :cost_history => search_parameters[:cost_history],
        :violations => violations,
        :generation_cost => generation_cost,
        :ramping_cost => ramping_cost,
        :time => time() - start_time
    )

    return info
end

"""
    test_scenarios_AC(data, factory, active_demand, reactive_demand, ramping_data, random_scenarios)

Validate proposed AC generator scenarios by checking simple demand bounds. Return only those scenarios that are valid,
along with their associated costs.

# Arguments
- `data::Dict{String, Any}`: PowerModels data
- `factory::ACMPOPFSearchFactory`: Model factory
- `active_demand::Dict{Int64, Float64}`: Active demand for a specific time period
- `reactive_demand::Dict{Int64, Float64}`: Reactive demand for a specific time period
- `ramping_data::Dict{String, Any}`: Ramping info
- `random_scenarios::Vector{Tuple{Dict{Int64, Float64}, Dict{Int64, Float64}}}`: AC scenarios to test

# Returns
- `Vector{Tuple{Dict{Int64, Float64}, Dict{Int64, Float64}, Float64}}`: Valid scenarios with P, Q, and costs
- `Dict{Symbol, Int64}`: Violation counts
"""
function test_scenarios_AC(data, factory, active_demand, reactive_demand, ramping_data, random_scenarios)
    violations = Dict(
        :min_active_demand_not_met => 0,
        :min_reactive_demand_not_met => 0,
        :active_pmin_pmax_out_of_bounds => 0,
        :reactive_qmin_qmax_out_of_bounds => 0,
        :voltage_violations => 0
    )

    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
    gen_data = ref[:gen]
    minimum_active_demand = sum(values(active_demand))
    minimum_reactive_demand = sum(values(reactive_demand))

    valid_scenarios = []
    
    for (active_scenario, reactive_scenario) in random_scenarios
        scenario_valid = true
        
        # Test minimum demand constraints
        if sum(values(active_scenario)) < minimum_active_demand
            println("Active demand not met, skipping scenario")
            violations[:min_active_demand_not_met] += 1
            scenario_valid = false
        end
        
        # For reactive power, we need to consider that demand can be negative and generators can supply/absorb
        # Check if total reactive capability can meet the demand (considering both positive and negative)

        total_reactive_generation = sum(values(reactive_scenario))
        if abs(total_reactive_generation) < abs(minimum_reactive_demand) && 
           sign(total_reactive_generation) != sign(minimum_reactive_demand) &&
           abs(minimum_reactive_demand) > 0.001  # Only check if reactive demand is significant
            println("Reactive demand not met, skipping scenario (gen: $total_reactive_generation, demand: $minimum_reactive_demand)")
            violations[:min_reactive_demand_not_met] += 1
            scenario_valid = false
        end

        # Test generator bounds
        if scenario_valid
            for (gen_id, p_value) in active_scenario
                pmin = data["gen"][string(gen_id)]["pmin"]
                pmax = data["gen"][string(gen_id)]["pmax"]
                
                if !((pmin - 0.001 <= p_value) && (p_value <= pmax + 0.001))
                    println("Active power bounds violated for generator $gen_id, (P = $p_value, pmin = $pmin, pmax = $pmax)")
                    violations[:active_pmin_pmax_out_of_bounds] += 1
                    scenario_valid = false
                    break
                end
            end
        end
        
        if scenario_valid
            for (gen_id, q_value) in reactive_scenario
                qmin = data["gen"][string(gen_id)]["qmin"]
                qmax = data["gen"][string(gen_id)]["qmax"]
                
                if !((qmin - 0.001 <= q_value) && (q_value <= qmax + 0.001))
                    println("Reactive power bounds violated for generator $gen_id, (Q = $q_value, qmin = $qmin, qmax = $qmax)")
                    violations[:reactive_qmin_qmax_out_of_bounds] += 1
                    scenario_valid = false
                    break
                end
            end
        end
        
        # Calculate cost if scenario is valid
        if scenario_valid
            calculated_cost = 0.0
            for (gen_id, p_value) in active_scenario
                # Include reactive power cost if available
                q_value = reactive_scenario[gen_id]
                
                # Standard quadratic cost for active power
                calculated_cost += gen_data[gen_id]["cost"][1] * p_value^2 +
                                  gen_data[gen_id]["cost"][2] * p_value +
                                  gen_data[gen_id]["cost"][3]
                
                # Add reactive power cost if specified in generator data
                if haskey(gen_data[gen_id], "qcost") && !isempty(gen_data[gen_id]["qcost"])
                    calculated_cost += gen_data[gen_id]["qcost"][1] * q_value^2 +
                                      gen_data[gen_id]["qcost"][2] * q_value +
                                      gen_data[gen_id]["qcost"][3]
                end
            end
            
            push!(valid_scenarios, (active_scenario, reactive_scenario, calculated_cost))
        end
    end

    return valid_scenarios, violations
end

"""
    generate_new_scenarios_subset_AC(current_active, current_reactive, search_parameters, time_period; 
                                    scenarios_to_generate=15, method_choice=1, up_probability=0.3)

Given a current best-scenario time period, generate a subset of new scenarios (for the same time period) by randomly modifying
the given active values of generators. This is a key part of the local search portion of our graph search algorithm, 
where nearby scenarios are created to explore the solution space.

- Note: Currently, reactive generation exploration is disabled. The slack bus should in theory be able to meet reactive
demands without generator interference.
- Note 2: After running a small simulation, indeed disabling the reactive modification still allows optimality to be approached

# Arguments
- `current_active::Dict{Int64, Float64}`: Current active power values for generators in the given time period
- `current_reactive::Dict{Int64, Float64}`: Current reactive power values for generators in the given time period
- `search_parameters::Dict{Symbol, Any}`: Dictionary containing the global data dictionary for our search model
- `time_period::Int64`: The current time period for which to generate new scenarios
- `scenarios_to_generate::Int64`: Number of new scenarios to generate (default 15)
- `subset_percentage::Float64`: Percentage of generators to modify in each new scenario (default 0.3, currently unused)
- `method_choice::Int64`: Parameter passed into delta_AC which dictates how the maximum delta is calculated
    - 1: Stochastic variation (default)
    - 2: Dynamic Gap variation
    - 3: Temporal Smoothing (not implemented, defaults to stochastic)
- `up_probability::Float64`: Probability of increasing generator values rather than decreasing (default 0.3)

# Returns
- `new_scenarios::Vector{Tuple{Dict{Int64, Float64}, Dict{Int64, Float64}}}`: 
A vector of tuples, one for each new scenario specified to generate. Each tuple contains two dictionaries with all generators'
new active and reactive powers.
"""
function generate_new_scenarios_subset_AC(current_active::Dict{Int64, Float64}, current_reactive::Dict{Int64, Float64}, search_parameters::Dict{Symbol, Any},
                                         time_period::Int64; 
                                         scenarios_to_generate::Int64=15,
                                         subset_percentage::Float64=0.3, 
                                         method_choice::Int64=1,
                                         up_probability::Float64=0.3)
    
    data = search_parameters[:data]
    core_generators = 0.2 # Generators that are commonly modified across scenarios
    auxiliary_generators = 0.1 # Subset of new generators that will be different per-scenario

    all_generators = collect(keys(current_active))
    n_generators = length(all_generators)
    
    n_to_modify_core = max(1, round(Int, n_generators * core_generators))
    n_to_modify_auxiliary = max(1, round(Int, n_generators * auxiliary_generators))
    
    core_generators_to_modify = rand(all_generators, n_to_modify_core)
    
    random_scenarios = Vector{Tuple{Dict{Int64, Float64}, Dict{Int64, Float64}}}()

    for scenario_idx in 1:scenarios_to_generate
        new_active = copy(current_active)
        new_reactive = copy(current_reactive)
        # this set may intersect with our core generators???
        auxiliary_generators_to_modify = rand(all_generators, n_to_modify_auxiliary)

        variation_percent = delta_AC(search_parameters, method_choice, time_period)
        
        # Modify core generators
        for gen_id in core_generators_to_modify
            # Modify active power
            current_p = current_active[gen_id]
            max_p_variation = current_p * variation_percent
            p_variation = rand() * max_p_variation
            
            new_p = if rand() < up_probability
                current_p + p_variation
            else
                current_p - p_variation
            end
            
            pmin = data["gen"][string(gen_id)]["pmin"]
            pmax = data["gen"][string(gen_id)]["pmax"]
            new_active[gen_id] = clamp(new_p, pmin, pmax)
            
            # Modify reactive power - ensure we can meet reactive demand
            # current_q = current_reactive[gen_id]
            # qmin = data["gen"][string(gen_id)]["qmin"]
            # qmax = data["gen"][string(gen_id)]["qmax"]
            
            # # Use larger variation for reactive power and bias toward demand requirements
            # max_q_variation = max(abs(current_q) * variation_percent, abs(qmax - qmin) * 0.1)
            # q_variation = rand() * max_q_variation
            
            # # Get reactive demand info to bias generation
            # reactive_demand_total = get(search_parameters, :total_reactive_demand, [0.0])[time_period]
            # reactive_gen_total = get(search_parameters, :total_reactive_generation, [0.0])[time_period]
            
            # # Bias toward meeting reactive demand
            # reactive_bias = 0.5
            # if abs(reactive_demand_total) > 0.001
            #     if reactive_gen_total < reactive_demand_total
            #         # Need more positive reactive power
            #         up_probability_q = up_probability + reactive_bias
            #     else
            #         # Need less reactive power
            #         up_probability_q = up_probability - reactive_bias
            #     end
            # else
            #     up_probability_q = 0.5  # No bias if no significant reactive demand
            # end
            
            # new_q = if rand() < clamp(up_probability_q, 0.1, 0.9)
            #     current_q + q_variation
            # else
            #     current_q - q_variation
            # end
            
            # new_reactive[gen_id] = clamp(new_q, qmin, qmax)
        end

        # Modify auxiliary generators
        for gen_id in auxiliary_generators_to_modify
            # Similar process for auxiliary generators
            current_p = current_active[gen_id]
            max_p_variation = current_p * variation_percent
            p_variation = rand() * max_p_variation
            
            new_p = if rand() < up_probability
                current_p + p_variation
            else
                current_p - p_variation
            end
            
            pmin = data["gen"][string(gen_id)]["pmin"]
            pmax = data["gen"][string(gen_id)]["pmax"]
            new_active[gen_id] = clamp(new_p, pmin, pmax)
            
            # Reactive power variation with demand bias
            #     current_q = current_reactive[gen_id]
            #     qmin = data["gen"][string(gen_id)]["qmin"]
            #     qmax = data["gen"][string(gen_id)]["qmax"]

            #     max_q_variation = max(abs(current_q) * variation_percent, abs(qmax - qmin) * 0.05)
            #     q_variation = rand() * max_q_variation

            #     # Apply same reactive demand bias
            #     reactive_demand_total = get(search_parameters, :total_reactive_demand, [0.0])[time_period]
            #     reactive_gen_total = get(search_parameters, :total_reactive_generation, [0.0])[time_period]

            #     reactive_bias = 0.3  # Less bias for auxiliary generators
            #     if abs(reactive_demand_total) > 0.001
            #         if reactive_gen_total < reactive_demand_total
            #             up_probability_q = up_probability + reactive_bias
            #         else
            #             up_probability_q = up_probability - reactive_bias
            #         end
            #     else
            #         up_probability_q = 0.5
            #     end

            #     new_q = if rand() < clamp(up_probability_q, 0.1, 0.9)
            #         current_q + q_variation
            #     else
            #         current_q - q_variation
            #     end

            #     new_reactive[gen_id] = clamp(new_q, qmin, qmax)
        end
        
        push!(random_scenarios, (new_active, new_reactive))
    end
    
    # Always include current solution
    push!(random_scenarios, (current_active, current_reactive))
    
    # Generate additional scenarios specifically to meet reactive demand if needed
    # reactive_demand_total = get(search_parameters, :total_reactive_demand, [0.0])[time_period]
    # reactive_gen_total = sum(values(current_reactive))
    
    # if abs(reactive_demand_total) > 0.001 && abs(reactive_gen_total - reactive_demand_total) > abs(reactive_demand_total) * 0.1
    #     println("Generating additional scenarios to meet reactive demand (current: $reactive_gen_total, needed: $reactive_demand_total)")
        
    #     for extra_idx in 1:5  # Generate a few extra scenarios
    #         new_active = copy(current_active)
    #         new_reactive = copy(current_reactive)
            
    #         # Adjust reactive power to better match demand
    #         reactive_shortfall = reactive_demand_total - reactive_gen_total
    #         reactive_adjustment_per_gen = reactive_shortfall / length(all_generators)
            
    #         for gen_id in all_generators
    #             qmin = data["gen"][string(gen_id)]["qmin"]
    #             qmax = data["gen"][string(gen_id)]["qmax"]
                
    #             # Add some randomness to the adjustment
    #             adjustment = reactive_adjustment_per_gen * (0.5 + rand())
    #             new_q = current_reactive[gen_id] + adjustment
    #             new_reactive[gen_id] = clamp(new_q, qmin, qmax)
    #         end
            
    #         push!(random_scenarios, (new_active, new_reactive))
    #     end
    # end
    println(random_scenarios[1])
    
    return random_scenarios
end

"""
    delta_ac(scenario_idx, search_parameters, method_choice, time_period)

Calculate variation factor for AC scenarios considering both active and reactive power demands.

# Arguments
- `search_parameters::Dict`: Helpful dictionary containing symbols that point to global data such as the iteration or our ref dictionary
-  `method_choice::Int64`: Choose how to generate local search variations.
    - 1 for stochastic/0.05
    - 2 for dynamic gap.
    - 3 for Temporal Smoothing (not implemented, defaults to stochastic)
- `time_period::Int64`: Current scenario's time period (for accessing hourly demands)

# Returns
- `delta::Float64`: A small floating point number that influences by what percent generators get modified by
"""
function delta_AC(search_parameters::Dict, method_choice::Int64, time_period::Int64)
    #= 
    time_periods = length(search_parameters[:total_active_generation]) this line is unnused,
    but could be implemented for error checking if decided it is necessary.
    =#

    if search_parameters[:iteration] < 5
        factor = 0.05
    else
        factor = 0.3
    end
    
    if method_choice == 1
        delta = 0.05
    elseif method_choice == 2
        # Consider both active and reactive power mismatches
        active_demand = search_parameters[:total_active_demand][time_period]
        reactive_demand = search_parameters[:total_reactive_demand][time_period]
        active_generation = search_parameters[:total_active_generation][time_period]
        reactive_generation = search_parameters[:total_reactive_generation][time_period]
        
        active_ratio = active_generation / active_demand
        reactive_ratio = abs(reactive_generation) / max(abs(reactive_demand), 0.01)  # Avoid division by zero
        
        # Combine both ratios with weighting
        combined_ratio = 0.7 * abs(active_ratio - 1.0) + 0.3 * abs(reactive_ratio - 1.0)
        delta = combined_ratio * factor
    else
        delta = 0.05
    end
    
    return delta
end

"""
    find_largest_time_period_ac(time_periods, active_demands, reactive_demands)

Find the time period with the highest combined active and reactive demand.

# Arguments
- `time_periods::Int64`: Number of time periods total
- `active_demands::Vector{Dict{Int64, Float64}}`: All active demands over all time periods
- `reactive_demands::Vector{Dict{Int64, Float64}}`: All reactive demands over all time periods

# Returns
- `largest_index::Int64`: A single integer representing the time period with the greatest demand
"""
function find_largest_time_period_AC(time_periods, active_demands, reactive_demands)
    largest_index = -1
    largest_combined = 0

    for t in 1:time_periods
        active_sum = sum(values(active_demands[t]))
        reactive_sum = abs(sum(values(reactive_demands[t])))
        
        # Combine active and reactive with weighting (active power typically more significant)
        combined_demand = active_sum + 0.3 * reactive_sum
        
        if combined_demand > largest_combined
            largest_combined = combined_demand
            largest_index = t
        end
    end

    return largest_index
end

"""
    build_and_optimize_largest_period_ac(factory, active_demand, reactive_demand, ramping_data)

Build and optimize an AC power flow model for the peak demand period, setting a baseline for
    generators, line flow and demands that we can build off of.

# Arguments
- `factory::AbstractMPOPFModelFactory`: The desired model factory, typically MPOPFSearchFactory
- `active_demand::Dict{Int64, Float64}`: Demands for generators in the largest time period.
- `reactive_demand::Dict{Int64, Float64}`: Same as above but for the imaginary portion
- `ramping_data::Dict{String, Any}`: Dictionary containing ramp limits, ids and costs for generators in the Matpower case

# Returns
- `model::AbstractMPOPFModel`: An optimized model created from the provided factory and ramping/demand data
"""
function build_and_optimize_largest_period_AC(factory, active_demand, reactive_demand, ramping_data)
    # active_demand here is put into a container to create a 1 time period vector for model creation.
    # don't pass active_demand or reactive_demand directly or create_search_model may access invalid space.

    demands = [active_demand]  # Adjust based on your factory interface
    reactive_demands = [reactive_demand]
    
    model = create_search_model(factory, 1, ramping_data, demands, reactive_demands)
    optimize!(model.model)

    return model
end

"""
    test_feasibility_AC(factory, path, graph, active_demands, reactive_demands, ramping_data)

Test the feasibility of each node in a path by solving an AC power flow model.

# Arguments
- `factory::AbstractMPOPFModelFactory`: The model's factory for use in making and optimizing the scenario path we desire
- `path::Vector{Int64}`: An array of integer nodes representing a particular shortest path in our scenario graph
- `graph::MetaDiGraph`: A graph defined using MetaGraphs package containing our scenario nodes and edges
- `active_demands::Vector{Dict{Int64, Float64}}`: All active demands across all time periods
- `reactive_demands::Vector{Dict{Int64, Float64}}`: All reactive demands across all time periods
- `ramping_data::Dict{String, Any}: A dictionary containing ramp limits, ids and costs for generators.`

# Returns
- `infeasible_nodes::Vector{Int64}`: A vector containing any nodes which make our current path infeasible.
"""
function test_feasibility_AC(factory::AbstractMPOPFModelFactory, path::Vector{Int64}, graph::MetaDiGraph, active_demands::Vector{Dict{Int64, Float64}}, reactive_demands::Vector{Dict{Int64, Float64}}, ramping_data::Dict{String, Any})
    infeasible_nodes = []
    for node in path[2:end-1]
        time_period = get_prop(graph, node, :time_period)
        active_values = get_prop(graph, node, :active_generator_values)
        reactive_values = get_prop(graph, node, :reactive_generator_values)

        model = create_search_model(factory, 1, ramping_data, [active_demands[time_period]], [reactive_demands[time_period]])

        # Fix both active and reactive power values
        for (gen_id, p_value) in active_values
            fix(model.model[:pg][1, gen_id], p_value, force=true)
        end
        
        for (gen_id, q_value) in reactive_values
            fix(model.model[:qg][1, gen_id], q_value, force=true)
        end
        
        optimize!(model.model)
        status = termination_status(model.model)

        if status != MOI.LOCALLY_SOLVED && status != MOI.OPTIMAL
            push!(infeasible_nodes, node)
            continue
        end
    end

    return infeasible_nodes
end

# Additional helper functions for AC formulation...

"""
    shortest_path_AC(graph, time_periods)

Find the shortest path from source to sink in the AC graph using Dijkstra's algorithm, considering ramp cost edge weights.

# Arguments
- `graph::MetaDiGraph`: The directed graph containing nodes and edges with weights
- `time_periods::Int64`: The number of time periods in the optimization problem

# Returns
- `path::Vector{Int64}`: A vector of node indices representing the shortest path from source to sink. Returns `false` if no path exists.
"""
function shortest_path_AC(graph::MetaDiGraph, time_periods::Int64)
    working_graph = deepcopy(graph)

    for e in edges(working_graph)
        src = Graphs.src(e)
        dst = Graphs.dst(e)
        current_weight = get_prop(working_graph, src, dst, :weight)
        node_cost = get_prop(working_graph, src, :cost)
        set_prop!(working_graph, src, dst, :weight, current_weight + node_cost)
    end

    source_node = 1
    sink_node = first(filter_vertices(working_graph, :time_period, time_periods + 1))

    state = Graphs.dijkstra_shortest_paths(working_graph, source_node, MetaGraphs.weights(working_graph))

    if state.parents[sink_node] == 0 && sink_node != source_node
        return false
    end

    path = Int64[]
    current = sink_node

    while current != source_node
        push!(path, current)
        current = state.parents[current]
    end
    push!(path, source_node)

    reverse!(path)
    return path
end

"""
    build_initial_graph_AC(scenarios, time_periods)

Construct initial AC graph with nodes containing both P and Q values.

# Arguments
- `scenarios::Vector{Any}`: A vector of time periods, each containing the respective group of candidate scenario nodes.
- `time_periods::Int64`: The number of time periods in the optimization problem

# Returns
- `graph::MetaDiGraph`: A directed graph with nodes representing scenarios. No edges exist save for between the source and sink nodes
and their respective first and last time period nodes. (Ex. Node 0 connects to all time period 1 nodes)
"""
function build_initial_graph_AC(scenarios::Vector{Any}, time_periods::Int64)
    graph = MetaDiGraph()
    defaultweight!(graph, 1.0)
    
    # Add source node
    add_vertex!(graph)
    first_node = nv(graph)
    set_prop!(graph, first_node, :time_period, 0)
    set_prop!(graph, first_node, :active_generator_values, Dict{Int64, Float64}())
    set_prop!(graph, first_node, :reactive_generator_values, Dict{Int64, Float64}())
    set_prop!(graph, first_node, :cost, 0)

    for p in 1:time_periods
        for (t, scenario) in enumerate(scenarios)
            add_vertex!(graph)
            current_node = nv(graph)
            
            set_prop!(graph, current_node, :time_period, p)
            set_prop!(graph, current_node, :active_generator_values, scenario[1])   # Active power values
            set_prop!(graph, current_node, :reactive_generator_values, scenario[2]) # Reactive power values
            set_prop!(graph, current_node, :cost, scenario[3]) # Combined cost
        end
    end

    # Add sink node
    add_vertex!(graph)
    last_node = nv(graph)
    set_prop!(graph, last_node, :time_period, time_periods + 1)
    set_prop!(graph, last_node, :active_generator_values, Dict{Int64, Float64}())
    set_prop!(graph, last_node, :reactive_generator_values, Dict{Int64, Float64}())
    set_prop!(graph, last_node, :cost, 0)

    # Connect source to first period nodes
    first_nodes = collect(filter_vertices(graph, :time_period, 1))
    for n in first_nodes
        add_edge!(graph, first_node, n)
        edge = Edge(first_node, n)
        set_prop!(graph, edge, :weight, 0)
    end

    # Connect last period nodes to sink
    last_nodes = collect(filter_vertices(graph, :time_period, time_periods))
    for n in last_nodes
        add_edge!(graph, n, last_node)
        edge = Edge(n, last_node)
        set_prop!(graph, edge, :weight, 0)
    end

    return graph
end

"""
    add_weighted_edges_AC!(graph, time_periods, ramping_data)

Add edges between all adjacent time periods denoted by AC ramping costs.
The use of this function allows minimization of ramping costs to be considered when choosing a shortest path.
Keep in mind, reactive power does not (in the current state) influence cost, so it is not part of our edge weighting

# Arguments
- `graph::MetaDiGraph`: The directed graph containing scenario nodes
- `time_periods::Int64`: The number of time periods we are solving for
- `ramping_data::Dict{String, Any}`: Dictionary containing ramp limits, ids and costs for generators
"""
function add_weighted_edges_AC!(graph::MetaDiGraph, time_periods::Int64, ramping_data::Dict{String, Any})
    ramp_costs = ramping_data["costs"]
    ramp_limits = ramping_data["ramp_limits"]
    
    for n in 1:(time_periods - 1)
        nodes_n = collect(filter_vertices(graph, :time_period, n))
        nodes_n1 = collect(filter_vertices(graph, :time_period, n + 1))

        for node_n in nodes_n
            active_values_n = get_prop(graph, node_n, :active_generator_values)
            
            for node_n1 in nodes_n1
                active_values_n1 = get_prop(graph, node_n1, :active_generator_values)
                
                total_edge_cost = 0
                violates = false
                
                # Check ramping constraints for active power
                for gen_id in keys(active_values_n)
                    active_difference = abs(active_values_n[gen_id] - active_values_n1[gen_id])
                    
                    if active_difference <= ramp_limits[gen_id]
                        total_edge_cost += active_difference * ramp_costs[gen_id]
                    else
                        violates = true
                        break
                    end
                end
                
                if !violates
                    add_edge!(graph, node_n, node_n1)
                    edge = Edge(node_n, node_n1)
                    set_prop!(graph, edge, :weight, total_edge_cost)
                end
            end
        end
        edge_count = 0
        for node_n in nodes_n
            for node_n1 in nodes_n1
                if has_edge(graph, node_n, node_n1)
                    edge_count += 1
                end
            end
        end
        println("Created $edge_count edges from period $n to $(n+1)\n")
    end
end

"""
    calculate_path_cost_AC(path, graph)

Calculate the total node + edge cost, and also return individually the ramping cost and generation cost of a path.

# Arguments
-   `path::Vector{Int64}`: A vector of node indices representing a path through the graph
-   `graph::MetaDiGraph`: The directed graph containing nodes and edges with ramping cost weights

# Returns
-   `Dict{Symbol, Float64}`: A dictionary containing the total cost, generation cost, and ramping cost for the given path
"""
function calculate_path_cost_AC(path::Vector{Int64}, graph::MetaDiGraph)
    total_cost = 0.0
    generation_cost = 0.0
    ramping_cost = 0.0

    for i in 1:(length(path)-1)
        src_node = path[i]
        dst_node = path[i+1]
        
        # Add generation cost from source node
        if has_prop(graph, src_node, :cost)
            node_cost = get_prop(graph, src_node, :cost)
            generation_cost += node_cost
            total_cost += node_cost
        end
        
        # Add ramping cost from edge
        if has_edge(graph, src_node, dst_node)
            edge_cost = get_prop(graph, src_node, dst_node, :weight)
            ramping_cost += edge_cost
            total_cost += edge_cost
        end
    end
    
    return Dict(
        :total_cost => total_cost,
        :generation_cost => generation_cost,
        :ramping_cost => ramping_cost
    )
end

"""
    extract_solution_AC(graph, path)

Extract AC solution from shortest path, returning the generator values for active and reactive power, as well as the total generation cost
    for each time period (ramping not included).

# Arguments
- `graph::MetaDiGraph`: The directed graph containing nodes with generator values and costs, as well as edges weighed by ramping costs.
- `path::Vector{Int64}`: A vector of node indices representing a path through the graph

# Returns
- `solution::Dict{Int, Dict{Symbol, Any}}`: A time period-ordered dictionary containing 
    the generator values and costs for each time period.
    - `solution[time_period][:active_generator_values]`
    - `solution[time_period][:reactive_generator_values]`
    - `solution[time_period][:cost]`
"""
function extract_solution_AC(graph::MetaDiGraph, path::Vector{Int64})
    solution = Dict{Int, Dict{Symbol, Any}}()

    for node in path
        time_period = get_prop(graph, node, :time_period)
        solution[time_period] = Dict(
            :active_generator_values => get_prop(graph, node, :active_generator_values),
            :reactive_generator_values => get_prop(graph, node, :reactive_generator_values),
            :cost => get_prop(graph, node, :cost)
        )
    end

    return solution
end

"""
    build_new_graph_AC(new_scenarios, time_periods)

Construct new AC graph using updated scenarios. The generated graph will have time_periods + 2 nodes,
with a source node at time 0 and a sink node at time time_periods + 1. Each time period will have its own set of scenario nodes.
The source and sink nodes have 0 weight and connect to all first and last time period nodes, all other nodes have no edges.

# Arguments
- `new_scenarios::Vector{Vector{Any}}`: A vector which contains other vectors full of scenario tuples.
    Each tuple contains dictionaries for active and reactive powers, plus the scenario cost. There will be a vector for every time period,
    containing a list of its valid scenarios.
- `time_periods::Int64`: The number of time periods in the current model.

# Returns
- `new_graph::MetaDiGraph`: A new directed graph containing all valid scenarios for each time period.
"""
function build_new_graph_AC(new_scenarios::Vector{Vector{Any}}, time_periods::Int64)
    new_graph = MetaDiGraph()
    defaultweight!(new_graph, 1.0)

    # Add source node
    add_vertex!(new_graph)
    source_node = nv(new_graph) # Stands for number of vertices. Not sure why it isn't hard coded as 1 in this case, but I won't touch it.
    set_prop!(new_graph, source_node, :time_period, 0)
    set_prop!(new_graph, source_node, :active_generator_values, Dict{Int64, Float64}())
    set_prop!(new_graph, source_node, :reactive_generator_values, Dict{Int64, Float64}())
    set_prop!(new_graph, source_node, :cost, 0)

    # Add scenario nodes for each time period
    for t in 1:time_periods
        for (s, scenario) in enumerate(new_scenarios[t])
            add_vertex!(new_graph)
            current_node = nv(new_graph)
            set_prop!(new_graph, current_node, :time_period, t)
            set_prop!(new_graph, current_node, :active_generator_values, scenario[1])
            set_prop!(new_graph, current_node, :reactive_generator_values, scenario[2])
            set_prop!(new_graph, current_node, :cost, scenario[3])
        end
    end

    # Add sink node
    add_vertex!(new_graph)
    sink_node = nv(new_graph)
    set_prop!(new_graph, sink_node, :time_period, time_periods + 1)
    set_prop!(new_graph, sink_node, :active_generator_values, Dict{Int64, Float64}())
    set_prop!(new_graph, sink_node, :reactive_generator_values, Dict{Int64, Float64}())
    set_prop!(new_graph, sink_node, :cost, 0)

    # Connect source to first time period nodes
    first_nodes = collect(filter_vertices(new_graph, :time_period, 1))
    for n in first_nodes
        add_edge!(new_graph, source_node, n)
        edge = Edge(source_node, n)
        set_prop!(new_graph, edge, :weight, 0)
    end
    
    # Connect last time period nodes to sink
    last_nodes = collect(filter_vertices(new_graph, :time_period, time_periods))
    for n in last_nodes
        add_edge!(new_graph, n, sink_node)
        edge = Edge(n, sink_node)
        set_prop!(new_graph, edge, :weight, 0)
    end

    return new_graph
end

"""
    get_generation_and_ramping_costs_AC(data, info, model)

Runs the cost calculations for both a provided graph search model and a full local search model
    by adding all generation costs multiplied by their coefficients, as well as the polynomial cost terms
    for active and reactive power generation. Then returns a 4 element dictionary with the respective model costs.

# Arguments
- `data::Dict{String, Any}`: The PowerModels case dictionary
- `info::Dict{Symbol, Any}`: An optimized solution from AC_graph_search
- `model::AbstractMPOPFModel`: A fully optimized external model, typically a SearchModel

# Returns
- `info::Dict{Symbol, Any}`: A dictionary containing symbols for the generation and ramping costs of the provided models
"""
function get_generation_and_ramping_costs_AC(data::Dict{String, Any}, info::Dict{Symbol, Any}, model::AbstractMPOPFModel)
    graph_model_generation_cost = info[:generation_cost]
    graph_model_ramping_cost = info[:ramping_cost]
    search_model_generation_cost = 0.0
    search_model_ramping_cost = 0.0

    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
    gen_data = ref[:gen]
    T = model.time_periods
    ramping_data = model.ramping_data

    # Sum total generation costs (active + reactive power)
    for t in 1:T, g in keys(gen_data)
        p_val = value(model.model[:pg][t,g])
        # q_val = value(model.model[:qg][t,g])
        
        # Active power cost
        search_model_generation_cost += 
            gen_data[g]["cost"][1] * p_val^2 +
            gen_data[g]["cost"][2] * p_val +
            gen_data[g]["cost"][3]
            
            # # Reactive power cost (if available)
            # if haskey(gen_data[g], "qcost") && !isempty(gen_data[g]["qcost"])
            #     search_model_generation_cost += 
            #         gen_data[g]["qcost"][1] * q_val^2 +
            #         gen_data[g]["qcost"][2] * q_val +
            #         gen_data[g]["qcost"][3]
            # end
    end

    # Sum total ramping costs (active power ramping)
    for t in 2:T, g in keys(gen_data)
        if haskey(model.model, :ramp_up) && haskey(model.model, :ramp_down)
            search_model_ramping_cost +=
                ramping_data["costs"][g] * (value(model.model[:ramp_up][t,g]) + value(model.model[:ramp_down][t, g]))
        end
    end

    return Dict(
        :graph_model_generation_cost => graph_model_generation_cost,
        :graph_model_ramping_cost => graph_model_ramping_cost,
        :search_model_generation_cost => search_model_generation_cost,
        :search_model_ramping_cost => search_model_ramping_cost
    )
end

"""
    graph_demands_and_generation_AC(active_demands, reactive_demands, full_model, graph_solution)

Plot AC demand and generation output comparisons.

# Arguments
- `active_demands::Vector{Dict{Int64, Float64}}`: All active demands over all time periods
- `reactive_demands::Vector{Dict{Int64, Float64}}`: All reactive demands over all time periods
- `full_model::AbstractMPOPFModel`: The full MPOPF model to be compared against
- `graph_solution::MetaDiGraph`: The optimized graph solution returned by AC_graph_search

# Output
- Displays and saves comparative graphs
"""
function graph_demands_and_generation_AC(active_demands::Vector{Dict{Int64, Float64}}, reactive_demands::Vector{Dict{Int64, Float64}}, full_model::AbstractMPOPFModel, graph_solution::MetaDiGraph)
    time_periods = length(graph_solution) - 2
    
    # Extract active power outputs
    graph_active_outputs = []
    graph_reactive_outputs = []

    for i in 1:time_periods
        push!(graph_active_outputs, sum(values(graph_solution[i][:active_generator_values])))
        push!(graph_reactive_outputs, sum(values(graph_solution[i][:reactive_generator_values])))
    end

    # Get full model outputs
    full_model_active_outputs = Array(value.(full_model.model[:pg]))
    full_model_active_outputs = vec(sum(full_model_active_outputs, dims=2))
    
    full_model_reactive_outputs = Array(value.(full_model.model[:qg]))
    full_model_reactive_outputs = vec(sum(full_model_reactive_outputs, dims=2))

    # Prepare demand data
    active_demand_totals = [sum(values(d)) for d in active_demands[1:time_periods]]
    reactive_demand_totals = [sum(values(d)) for d in reactive_demands[1:time_periods]]

    # Plot active power
    p1 = plot(full_model_active_outputs, label="Optimal Model - Active", lw=2)
    plot!(p1, graph_active_outputs, label="Graph Model - Active", lw=2)
    plot!(p1, active_demand_totals, label="Active Demand", lw=2)
    xlabel!(p1, "Time Period")
    ylabel!(p1, "Active Power (MW)")
    title!(p1, "Active Power Generation vs Demand")
    display(p1)
    savefig("ac_active_power_curve.png")

    # Plot reactive power
    p2 = plot(full_model_reactive_outputs, label="Optimal Model - Reactive", lw=2)
    plot!(p2, graph_reactive_outputs, label="Graph Model - Reactive", lw=2)
    plot!(p2, reactive_demand_totals, label="Reactive Demand", lw=2)
    xlabel!(p2, "Time Period")
    ylabel!(p2, "Reactive Power (MVAR)")
    title!(p2, "Reactive Power Generation vs Demand")
    display(p2)
    savefig("ac_reactive_power_curve.png")

    # Plot generation errors
    p3 = plot(full_model_active_outputs .- active_demand_totals, label="Optimal - Active Error", lw=2)
    plot!(p3, graph_active_outputs .- active_demand_totals, label="Graph - Active Error", lw=2)
    plot!(p3, full_model_reactive_outputs .- reactive_demand_totals, label="Optimal - Reactive Error", lw=2)
    plot!(p3, graph_reactive_outputs .- reactive_demand_totals, label="Graph - Reactive Error", lw=2)
    xlabel!(p3, "Time Period")
    ylabel!(p3, "Generation - Demand Error")
    title!(p3, "Generation Error Compared to Demand")
    display(p3)
    savefig("ac_generation_error.png")
end

"""
    output_run_data_to_csv_AC(data, file_path, active_demands, reactive_demands, model, info)

Write AC run summary and time-series data to CSV.

# Arguments
- `data::Dict{String, Any}`: The PowerModels case dictionary
- `file_path::String`: The path to the folder where the CSV file will be written
- `active_demands::Vector{Dict{Int64, Float64}}`: All active demands over all time periods
- `reactive_demands::Vector{Dict{Int64, Float64}}`: All reactive demands over all time periods
- `model::AbstractMPOPFModel`: The fully optimized model for comparison
- `info::Dict{Symbol, Any}`: The optimized solution from AC_graph_search

# Returns
- `Filename::String`: The name of the written CSV file.
"""
function output_run_data_to_csv_AC(data::Dict{String, Any}, file_path::String, active_demands::Vector{Dict{Int64, Float64}}, reactive_demands::Vector{Dict{Int64, Float64}}, model::AbstractMPOPFModel, info::Dict{Symbol, Any})
    filename = split(file_path, "/") |> last
    time_periods = length(info[:solution]) - 2
    
    # Calculate active and reactive power totals for graph model
    graph_active_totals = []
    graph_reactive_totals = []
    
    for i in 1:time_periods
        push!(graph_active_totals, sum(values(info[:solution][i][:active_generator_values])))
        push!(graph_reactive_totals, sum(values(info[:solution][i][:reactive_generator_values])))
    end
    
    # Calculate totals for optimal model
    optimal_active_totals = Array(value.(model.model[:pg]))
    optimal_active_totals = vec(sum(optimal_active_totals, dims=2))
    
    optimal_reactive_totals = Array(value.(model.model[:qg]))
    optimal_reactive_totals = vec(sum(optimal_reactive_totals, dims=2))
    
    # Calculate demand totals
    active_demand_totals = [sum(values(d)) for d in active_demands[1:time_periods]]
    reactive_demand_totals = [sum(values(d)) for d in reactive_demands[1:time_periods]]

    # Get cost information
    cost_info = get_generation_and_ramping_costs_AC(data, info, model)
    
    # Prepare CSV data
    csv_data = []
    
    # Basic information
    push!(csv_data, ["filename", filename])
    push!(csv_data, ["time_periods", time_periods])
    push!(csv_data, ["formulation", "AC"])
    
    # Graph cost information
    push!(csv_data, ["graph_total_cost", info[:cost]])
    push!(csv_data, ["graph_generation_cost", info[:generation_cost]])
    push!(csv_data, ["graph_ramping_cost", info[:ramping_cost]])

    # Optimal cost information
    push!(csv_data, ["optimal_total_cost", objective_value(model.model)])
    push!(csv_data, ["optimal_generation_cost", cost_info[:search_model_generation_cost]])
    push!(csv_data, ["optimal_ramping_cost", cost_info[:search_model_ramping_cost]])
    
    # Performance metrics
    push!(csv_data, ["cost_gap_percent", 100 * (info[:cost] - objective_value(model.model)) / objective_value(model.model)])
    
    # Timing information
    push!(csv_data, ["graph_solve_time", info[:time]])
    push!(csv_data, ["optimal_model_solve_time", solve_time(model.model)])
    push!(csv_data, ["speedup_ratio", solve_time(model.model) / info[:time]])
    
    # Violations information
    violations = info[:violations]
    push!(csv_data, ["active_pmin_pmax_violations", violations[:active_pmin_pmax_out_of_bounds]])
    push!(csv_data, ["reactive_qmin_qmax_violations", violations[:reactive_qmin_qmax_out_of_bounds]])
    push!(csv_data, ["voltage_violations", violations[:voltage_violations]])
    push!(csv_data, ["infeasible_model_violations", violations[:infeasible_model]])
    push!(csv_data, ["min_active_demand_not_met", violations[:min_active_demand_not_met]])
    push!(csv_data, ["min_reactive_demand_not_met", violations[:min_reactive_demand_not_met]])
    
    # Graph statistics
    if haskey(info, :graph)
        graph = info[:graph]
        push!(csv_data, ["graph_nodes", nv(graph)])
        push!(csv_data, ["graph_edges", ne(graph)])
    end
    
    # Path information
    if haskey(info, :path)
        path_str = join(info[:path], ";")
        push!(csv_data, ["optimization_path", path_str])
        push!(csv_data, ["path_length", length(info[:path])])
    end
    
    # Cost history summary
    if haskey(info, :cost_history)
        cost_hist = info[:cost_history]
        push!(csv_data, ["cost_history_length", length(cost_hist)])
        push!(csv_data, ["initial_cost", cost_hist[1]])
        push!(csv_data, ["final_cost", cost_hist[end]])
        push!(csv_data, ["cost_improvement", cost_hist[1] - cost_hist[end]])
        push!(csv_data, ["convergence_iterations", length(cost_hist)])
    end
    
    # Power balance metrics
    active_balance_error = sum(abs.(graph_active_totals .- active_demand_totals))
    reactive_balance_error = sum(abs.(graph_reactive_totals .- reactive_demand_totals))
    push!(csv_data, ["total_active_balance_error", active_balance_error])
    push!(csv_data, ["total_reactive_balance_error", reactive_balance_error])
    
    # Add separator for detailed data
    push!(csv_data, ["--- DETAILED TIME SERIES DATA ---", ""])
    
    # Time series data
    # Active power data
    graph_active_row = ["graph_active_power_totals"; graph_active_totals]
    push!(csv_data, graph_active_row)
    
    optimal_active_row = ["optimal_active_power_totals"; optimal_active_totals]
    push!(csv_data, optimal_active_row)
    
    active_demand_row = ["active_demand_totals"; active_demand_totals]
    push!(csv_data, active_demand_row)
    
    # Reactive power data
    graph_reactive_row = ["graph_reactive_power_totals"; graph_reactive_totals]
    push!(csv_data, graph_reactive_row)
    
    optimal_reactive_row = ["optimal_reactive_power_totals"; optimal_reactive_totals]
    push!(csv_data, optimal_reactive_row)
    
    reactive_demand_row = ["reactive_demand_totals"; reactive_demand_totals]
    push!(csv_data, reactive_demand_row)
    
    # Individual generator data for each time period
    for i in 1:time_periods
        active_gen_values = info[:solution][i][:active_generator_values]
        reactive_gen_values = info[:solution][i][:reactive_generator_values]
        
        for (gen_id, p_value) in active_gen_values
            active_row = ["t$(i)_generator_$(gen_id)_active", p_value]
            push!(csv_data, active_row)
        end
        
        for (gen_id, q_value) in reactive_gen_values
            reactive_row = ["t$(i)_generator_$(gen_id)_reactive", q_value]
            push!(csv_data, reactive_row)
        end
        
        # Cost breakdown by time period
        if haskey(info[:solution][i], :cost)
            cost_row = ["t$(i)_total_cost", info[:solution][i][:cost]]
            push!(csv_data, cost_row)
        end
    end
    
    # Full cost history
    if haskey(info, :cost_history)
        cost_hist_row = ["full_cost_history"; info[:cost_history]]
        push!(csv_data, cost_hist_row)
    end
    
    # Write to CSV
    csv_filename = replace(filename, ".m" => "_ac_results.csv")
    
    open(csv_filename, "w") do io
        for row in csv_data
            row_str = join([string(x) for x in row], ",")
            println(io, row_str)
        end
    end
    
    println("AC results written to: $csv_filename")
    return csv_filename
end

"""
    extract_power_flow_data_AC(model)

Extract both active and reactive power generator values from AC model, returning them as symbols in a dictionary.

# Symbols
- `:active`
- `:reactive`

# Arguments
- `model::AbstractMPOPFModel`: The solved AC model from which to extract generator values

# Returns
- `Dict{Symbol, Dict{Int64, Float64}}`: A dictionary containing two dictionaries used to access active and reactive generation.
"""
function extract_power_flow_data_AC(model)
    pg_values = value.(model.model[:pg])
    qg_values = value.(model.model[:qg])
    
    active_dict = Dict(zip(pg_values.axes[2], [value(pg_values[key]) for key in keys(pg_values)]))
    reactive_dict = Dict(zip(qg_values.axes[2], [value(qg_values[key]) for key in keys(qg_values)]))
    
    return Dict(
        :active => active_dict,
        :reactive => reactive_dict
    )
end
