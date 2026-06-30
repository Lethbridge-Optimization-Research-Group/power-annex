using Graphs, MetaGraphs, Gurobi, JuMP

"""
    DC_graph_search(data, factory, demands, ramping_data, time_periods)

Create a DC graph model that iteratively adjusts generator values 
in order to form a solution.

# Arguments
- `data::Dict{String, Any}` : Powermodels parsed Matpower case data
- `factory::DCMPOPFSearchFactory` : Model factory for creating JuMP OPF models
- `demands::Vector{Dict{Int64, Float64}}` : Demands for each time period
- `ramping_data::Dict{String, Any}` : Ramping costs and limits for each generator
- `time_periods::Int` : Number of time periods 

# Returns 
- `info::Dict{Symbol, Any}` : Model info and associated data

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
function DC_graph_search(data, factory, demands, ramping_data, time_periods)
    
    iteration = 1
    max_iterations = 500
    

    start_time = time()
    time_vec = []

    search_parameters = Dict(
        :iteration => iteration,
        :cost_history => Vector{Float64}(),
        :total_demand => nothing,
        :total_generation => nothing,
        :data => data,
        :ramping_data => ramping_data
    )

    violations = Dict(
        :min_demand_not_met => 0,
        :pmin_pmax_out_of_bounds => 0,
        :infeasible_model => 0
    )

    highest_demand = find_largest_time_period(time_periods, demands)
    largest_model = build_and_optimize_largest_period(factory, demands[highest_demand], ramping_data)
    status = termination_status(largest_model.model)
    println(status)
    if status != MOI.LOCALLY_SOLVED && status != MOI.OPTIMAL 
        error("Largest time period infeasible")
    end

    largest_values = [value(largest_model.model[:pg][key]) for key in keys(largest_model.model[:pg])]
    baseline_generator_values = Dict(zip(largest_model.model[:pg].axes[2], largest_values))

    search_parameters[:total_demand] = [sum(values(demands[i])) for i in 1:time_periods]
    # For the first iteration, use the baseline values for all time periods
    baseline_total_gen = sum(values(baseline_generator_values))
    search_parameters[:total_generation] = fill(baseline_total_gen, time_periods)

    initial_scenarios_raw = generate_new_scenarios_subset(data, baseline_generator_values, search_parameters, 1)

    scenarios, scenario_violations = test_scenarios(data, factory, demands[highest_demand], ramping_data, initial_scenarios_raw)
    violations[:min_demand_not_met] += scenario_violations[:min_demand_not_met]
    violations[:pmin_pmax_out_of_bounds] += scenario_violations[:pmin_pmax_out_of_bounds]

    graph = build_initial_graph(scenarios, time_periods)
    add_weighted_edges!(graph, time_periods, ramping_data)

    feasibility = false
    path = nothing
    while !feasibility

        path = shortest_path(graph, time_periods)

        if path == false || path === nothing
            error("No feasible path found in the graph. The problem may be infeasible.")
        end

        infeasible_nodes = test_feasibility(factory, path, graph, demands, ramping_data)
        violations[:infeasible_model] += length(infeasible_nodes)

        if isempty(infeasible_nodes) 
            feasibility = true
        else
            # Remove infeasible nodes and rebuild connections
            for node in sort(infeasible_nodes, rev=true)  # Remove in reverse order to maintain indices
                rem_vertex!(graph, node)
            end
            # Need to rebuild edges after removing vertices
            add_weighted_edges!(graph, time_periods, ramping_data)
        end
    end

    path_results = calculate_path_cost(path, graph)
    
    best_graph = graph
    best_path = path
    best_cost = path_results[:total_cost]
    best_solution = extract_solution(best_graph, best_path)
    
    # update search parameters
    push!(search_parameters[:cost_history], best_cost)
    search_parameters[:total_demand] = [sum(values(demands[i])) for i in 1:time_periods]
    search_parameters[:total_generation] = [sum(values(best_solution[i][:generator_values])) for i in 1:time_periods]
    

    generation_cost = 0.0
    ramping_cost = 0.0
    no_improvement = 0
    finished = false

    while iteration < max_iterations

        current_generator_values = Vector{Dict{Int64, Float64}}()
        new_generator_values = Vector{Vector{Any}}()

        for i in 1:time_periods
            time_period_values = Dict()
            for (gen_id, val) in best_solution[i][:generator_values]
                time_period_values[gen_id] = val
            end
            push!(current_generator_values, time_period_values)
            push!(new_generator_values, Vector{Any}())
        end

        for i in 1:time_periods
            scenarios_for_period = generate_new_scenarios_subset(data, current_generator_values[i], search_parameters, i)
            tested_scenarios, scenario_violations = test_scenarios(data, factory, demands[i], ramping_data, scenarios_for_period)
            violations[:min_demand_not_met] += scenario_violations[:min_demand_not_met]
            violations[:pmin_pmax_out_of_bounds] += scenario_violations[:pmin_pmax_out_of_bounds]
            new_generator_values[i] = tested_scenarios
        end

        new_graph = build_new_graph(new_generator_values, time_periods)
        add_weighted_edges!(new_graph, time_periods, ramping_data)

        feasibility = false

        while !feasibility

            path = shortest_path(new_graph, time_periods)

            if path == false || path === nothing
                error("No feasible path found in the graph. The problem may be infeasible.")
            end

            infeasible_nodes = test_feasibility(factory, path, new_graph, demands, ramping_data)
            violations[:infeasible_model] += length(infeasible_nodes)

            if isempty(infeasible_nodes) 
                feasibility = true
            else
                for node in infeasible_nodes
                    rem_vertex!(new_graph, node)
                end
            end
        end
        
        path_results = calculate_path_cost(path, new_graph)

        if path_results[:total_cost] < best_cost
            best_graph = new_graph
            best_cost = path_results[:total_cost]
            best_path = path
            best_solution = extract_solution(best_graph, best_path)
            generation_cost = path_results[:generation_cost]
            ramping_cost = path_results[:ramping_cost]
        end

        push!(search_parameters[:cost_history], best_cost)
        iteration += 1

        iter_time = time()
        time_elapsed = iter_time - start_time
        push!(time_vec, time_elapsed)
        if iteration > 10
            recent_costs = search_parameters[:cost_history][iteration - 10:iteration-1]
            improvement = best_cost / maximum(recent_costs)#maximum(recent_costs) - best_cost

            if improvement > 0.999
                println("No improvement, stopping at $iteration iterations")
                break;
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
        :time => time() - start_time,
        :time_vec => time_vec
    )

    return info
end

"""
    shortest_path(graph, time_periods)

Find the shortest path from the source node to the sink node in the graph,
taking into account node costs and edge weights.

# Arguments
- `graph::MetaDiGraph{Int64, Float64}` : The graph representing generator scenarios and transitions
- `time_periods::Int` : The total number of time periods

# Returns
- `Vector{Int}` : A list of node indices representing the path, or `false` if none exists
"""

function shortest_path(graph, time_periods)

    working_graph = deepcopy(graph)

    for e in edges(working_graph)
        src = Graphs.src(e)
        dst = Graphs.dst(e)
        current_weight = get_prop(working_graph, src, dst, :weight)
        node_cost = get_prop(working_graph, src, :cost)
        set_prop!(working_graph, src, dst, :weight, current_weight + node_cost)
    end

    # find the source node and sink nodes
    source_node = 1
    sink_node = first(filter_vertices(working_graph, :time_period, time_periods + 1))

    state = Graphs.dijkstra_shortest_paths(working_graph, source_node, MetaGraphs.weights(working_graph))

    if state.parents[sink_node] == 0 && sink_node != source_node
        return false
    end

    path = Int[]
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
    test_feasibility(factory, path, graph, demands, ramping_data)

Test the feasibility of each node in a path by solving a power flow model.

# Arguments
- `factory::DCMPOPFSearchFactory` : Model factory for creating JuMP OPF models
- `path::Vector{Int}` : A path through the graph
- `graph::MetaDiGraph{Int64, Float64}` : The graph to test against
- `demands::Vector{Dict{Int64, Float64}}` : Time-varying demand values
- `ramping_data::Dict{String, Any}` : Ramping constraints and costs

# Returns
- `Vector{Int}` : List of infeasible node indices in the path
"""


function test_feasibility(factory, path, graph, demands, ramping_data)
    
    infeasible_nodes = []

    for node in path[2:end-1]
        time_period = get_prop(graph, node, :time_period)
        generator_values = get_prop(graph, node, :generator_values)

        model = create_search_model(factory, 1, ramping_data, [demands[time_period]])
        #set_optimizer_attribute(model.model, "LogToConsole", Float64(0))

        for (gen_id, value) in generator_values
            fix(model.model[:pg][1, gen_id], value, force=true)
        end
        #=
        set_optimizer(model.model, Gurobi.Optimizer)
        set_optimizer_attribute(model.model, "Presolve", 2)
        set_optimizer_attribute(model.model, "OutputFlag", 0)
        @objective(model.model, Min, 0)
        
        set_optimizer(model, HiGHS.Optimizer)
        set_optimizer_attribute(model, "presolve", "on")
        set_optimizer_attribute(model, "output_flag", false)
        @objective(model, Min, 0)
        =#
        
        optimize!(model.model)
        status = termination_status(model.model)

        if status != MOI.LOCALLY_SOLVED && status != MOI.OPTIMAL
            push!(infeasible_nodes, node)
            continue
        end

        #set_prop!(graph, node, :cost, objective_value(model.model))

    end

    return infeasible_nodes
end

"""
    calculate_path_cost(path, graph)

Calculate the total generation and ramping cost of a path in the graph.

# Arguments
- `path::Vector{Int}` : Path of node indices
- `graph::MetaDiGraph{Int64, Float64}` : The graph from which to extract costs

# Returns
- `Dict{Symbol, Float64}` : Total, generation, and ramping costs
"""

function calculate_path_cost(path, graph) 

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
    test_scenarios(data, factory, demand, ramping_data, random_scenarios)

Validate and cost each proposed generator scenario.

# Arguments
- `data::Dict{String, Any}` : PowerModels data
- `factory::DCMPOPFSearchFactory` : Model factory
- `demand::Dict{Int64, Float64}` : Demand for a specific time period
- `ramping_data::Dict{String, Any}` : Ramping info
- `random_scenarios::Vector{Dict{Int64, Float64}}` : Load scenarios to test

# Returns
- `Vector{Tuple{Dict{Int64, Float64}, Float64}}` : Valid scenarios and costs
- `Dict{Symbol, Int}` : Violation counts
"""

function test_scenarios(data, factory, demand, ramping_data, random_scenarios)

    violations = Dict(
        :min_demand_not_met => 0,
        :pmin_pmax_out_of_bounds => 0
    )

    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
    gen_data = ref[:gen]
    minimum_demand = sum(values(demand))

    tested_scenarios = []
    
    for scenario in random_scenarios
        scenario_valid = true
        
        # Test that minimum demand is met
        if sum(values(scenario)) < minimum_demand
            println("Demand not met, skipping scenario")
            violations[:min_demand_not_met] += 1
            scenario_valid = false
        end

        # Test pmin and pmax values for ALL generators before proceeding
        
        if scenario_valid
            for (gen_id, value) in scenario
                if !((data["gen"][string(gen_id)]["pmin"] - 0.001 <= value) && (value <= data["gen"][string(gen_id)]["pmax"] + 0.001))
                    pmin = data["gen"][string(gen_id)]["pmin"]
                    pmax = data["gen"][string(gen_id)]["pmax"]
                    
                    println("Pmin or Pmax bounds violated for generator $gen_id, (value = $value, pmin = $pmin, pmax = $pmax) skipping scenario")
                    
                    violations[:pmin_pmax_out_of_bounds] += 1
                    scenario_valid = false
                    break  # Exit the generator loop immediately
                end
            end
        end
        
        # Only calculate cost and add scenario if it's valid
        if scenario_valid
            calculated_cost = 0.0
            for (gen_id, value) in scenario
                calculated_cost += gen_data[gen_id]["cost"][1]*value^2 +
                                   gen_data[gen_id]["cost"][2]*value +
                                   gen_data[gen_id]["cost"][3]
            end
            push!(tested_scenarios, (scenario, calculated_cost))
        end
    end

    return tested_scenarios, violations
end

"""
    find_largest_time_period(time_periods, demands)

Find the time period with the highest total demand.

# Arguments
- `time_periods::Int` : Number of periods
- `demands::Vector{Dict{Int64, Float64}}` : Demand per time period

# Returns
- `Int` : Index of the time period with the highest demand
"""

function find_largest_time_period(time_periods, demands)

    largestIndex = -1
    largest = 0

    for t in 1:time_periods
        period_sum = sum(values(demands[t]))
        if period_sum > largest
            largest = period_sum
            largestIndex = t
        end
    end

    return largestIndex
end

"""
    build_and_optimize_largest_period(factory, demand, ramping_data)

Build and optimize a power flow model for the peak demand period.

# Arguments
- `factory::DCMPOPFSearchFactory` : Model factory
- `demand::Dict{Int64, Float64}` : Peak demand
- `ramping_data::Dict{String, Any}` : Ramping information

# Returns
- `MPOPF.MPOPFSearchModel` : Optimized JuMP model for the peak period
"""

function build_and_optimize_largest_period(factory, demand, ramping_data)

    model = create_search_model(factory, 1, ramping_data, [demand])
    optimize!(model.model)

    return model
end

function generate_new_scenarios_subset(data, current_outputs, search_parameters, time_period; 
    scenarios_to_generate=15,
    subset_percentage=0.3, 
    variation_percent=0.05,
    up_probability=0.3)
    
    time_periods = length(search_parameters[:total_generation])
    Random.seed!(42 + search_parameters[:iteration])
    core_generators = 0.0
    auxiliary_generators = 0.3

    all_generators = collect(keys(current_outputs))
    n_generators = length(all_generators)
    
    n_to_modify_core = max(1, round(Int, n_generators * core_generators))
    n_to_modify_auxiliary = max(1, round(Int, n_generators * auxiliary_generators))
    
    core_generators_to_modify = rand(all_generators, n_to_modify_core)

    # iteration > 1, use search heuristic

    random_scenarios = Vector{Dict{Int64, Float64}}()

    for scenario_idx in 1:scenarios_to_generate
        # Start with current outputs
        new_scenario = copy(current_outputs)
        auxiliary_generators_to_modify = rand(all_generators, n_to_modify_auxiliary) #TODO

        variation_percent = delta(scenario_idx, search_parameters, 2, time_period)
        

        # Modify each selected generator
        for gen_id in core_generators_to_modify
            current_value = current_outputs[gen_id]
            max_variation = current_value * variation_percent
            variation = rand() * max_variation
            
            # Calculate new value and clamp to bounds
            if rand() < up_probability
                new_value = current_value + variation
            else
                new_value = current_value - variation
            end
            
            # Always clamp to both min and max bounds
            pmin = data["gen"][string(gen_id)]["pmin"]
            pmax = data["gen"][string(gen_id)]["pmax"]
            new_scenario[gen_id] = clamp(new_value, pmin, pmax)
        end

        for gen_id in auxiliary_generators_to_modify
            current_value = current_outputs[gen_id]
            max_variation = current_value * variation_percent
            variation = rand() * max_variation
            
            # Calculate new value and clamp to bounds
            if rand() < up_probability
                new_value = current_value + variation
            else
                new_value = current_value - variation
            end
            
            # Always clamp to both min and max bounds
            pmin = data["gen"][string(gen_id)]["pmin"]
            pmax = data["gen"][string(gen_id)]["pmax"]
            new_scenario[gen_id] = clamp(new_value, pmin, pmax)
        end
        
        push!(random_scenarios, new_scenario)
        #variation_percent += 0.01
    end
    
    # Always include the current solution as one scenario
    push!(random_scenarios, current_outputs)
    
    return random_scenarios
end

function delta(scenario_idx, search_parameters, method_choice, time_period)
    
    # 1 = standard random approach
    # 2 = alpha technique (alpha = actual demand / total demand * some factor)
    # 3 = convolution technique (average of adjacent time periods / total demand * some factor)
    # 4 = cost history (use change in objective function to determine delta)
    time_periods = length(search_parameters[:total_generation])
    if search_parameters[:iteration] < 0
        factor = 0.05
    else
        factor = 0.3
    end
    
    if method_choice == 1
        delta = 0.05
    elseif method_choice == 2
        total_demand = search_parameters[:total_demand][time_period]
        total_generation = search_parameters[:total_generation][time_period]
        # If generation significantly exceeds demand, use larger variation
        # If generation barely meets demand, use smaller variation
        generation_demand_ratio = total_generation / total_demand
        delta = abs(generation_demand_ratio - 1.0) * factor
        # delta = max(0.01, min(delta, 0.2))  # Use to clamp if wanted
    elseif method_choice == 3
        n = 2 # number of neighbours to average
        total = 0.0
        no_of_time_periods = 0

        for i in time_period - n:time_period + n
            if i < 1 || i > time_periods
                continue
            else
                total += search_parameters[:total_generation][i]
                no_of_time_periods += 1
            end
        end

        if no_of_time_periods > 0
            average_generation = total / no_of_time_periods
            current_demand = search_parameters[:total_demand][time_period]
            
            # If average generation in neighborhood differs significantly from current demand
            avg_demand_ratio = average_generation / current_demand
            delta = abs(avg_demand_ratio - 1.0) * factor
            #delta = max(0.01, min(delta, 0.2))  # Use if wanting to clamp
        else

            delta = 0.05  # Fallback to default
        end
    else
        delta = 0.05
        # future, do not do right now
    end
    
    return delta

end

"""
    extract_power_flow_data(model)

Extract generator values from a JuMP model.

# Arguments
- `model::MPOPF.MPOPFSearchModel` : The optimization model

# Returns
- `Dict{Int, Float64}` : Generator ID to output value
"""

function extract_power_flow_data(model)
    
    m = value.(model.model[:pg])
    values = [value(m[key]) for key in keys(m)]
    return Dict(zip(m.axes[2], values'))
end

"""
    build_initial_graph(scenarios, time_periods)

Construct a graph with nodes and edges based on initial generator scenarios.

# Arguments
- `scenarios::Vector{Tuple{Dict{Int64, Float64}, Float64}}` : Generator values and costs
- `time_periods::Int` : Number of time periods

# Returns
- `MetaDiGraph{Int64, Float64}` : Graph with nodes for each scenario and period
"""

function build_initial_graph(scenarios::Vector{Any}, time_periods)
    graph = MetaDiGraph()
    defaultweight!(graph, 1.0)
    
    # add first node
    add_vertex!(graph)
    first_node = nv(graph)
    set_prop!(graph, first_node, :time_period, 0)
    set_prop!(graph, first_node, :generator_values, 0)
    set_prop!(graph, first_node, :cost, 0)

    for p in 1:time_periods
        for (t, scenario) in enumerate(scenarios)
            add_vertex!(graph)
            current_node = nv(graph)
            
            set_prop!(graph, current_node, :time_period, p) # time period
            set_prop!(graph, current_node, :generator_values, scenario[1]) # generator values
            set_prop!(graph, current_node, :cost, scenario[2]) # sum of values
        end
    end

    # add last node and corresponding edges
    add_vertex!(graph)
    last_node = nv(graph)
    set_prop!(graph, last_node, :time_period, time_periods + 1)
    set_prop!(graph, last_node, :generator_values, 0)
    set_prop!(graph, last_node, :cost, 0)

    first_nodes = collect(filter_vertices(graph, :time_period, 1))
    for n in first_nodes
        add_edge!(graph, first_node, n)
        edge = Edge(first_node, n)
        set_prop!(graph, edge, :weight, 0)
    end

    last_nodes = collect(filter_vertices(graph, :time_period, time_periods))
    for n in last_nodes
        add_edge!(graph, n, last_node)
        edge = Edge(n, last_node)
        set_prop!(graph, edge, :weight, 0)
    end

    return graph
end

"""
    add_weighted_edges!(graph, time_periods, ramping_data)

Add edges between nodes in adjacent time periods with ramping cost as weight.

# Arguments
- `graph::MetaDiGraph{int64, Float64}` : The scenario graph
- `time_periods::Int` : Number of time periods
- `ramping_data::Dict{String, Any}` : Ramping costs and limits
"""

function add_weighted_edges!(graph, time_periods, ramping_data)

    ramp_costs = ramping_data["costs"]
    ramp_limits = ramping_data["ramp_limits"]
    for n in 1:(time_periods - 1)
        nodes_n = collect(filter_vertices(graph, :time_period, n))
        nodes_n1 = collect(filter_vertices(graph, :time_period, n + 1))

        for node_n in nodes_n
            gen_values_n = get_prop(graph, node_n, :generator_values)
            for node_n1 in nodes_n1
                gen_values_n1 = get_prop(graph, node_n1, :generator_values)
                total_edge_cost = 0
                violates = false
                for gen_id in keys(gen_values_n)
                    difference = abs(gen_values_n[gen_id] - gen_values_n1[gen_id])
                    if difference <= ramp_limits[gen_id]
                        total_edge_cost += difference * ramp_costs[gen_id]
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
    end
end

"""
    extract_solution(graph, path)

Extract generator values and costs from a given path in the graph.

# Arguments
- `graph::MetaDiGraph{Int64, Float64}` : The scenario graph
- `path::Vector{Int}` : A feasible path through the graph

# Returns
- `Dict{Int, Dict{Symbol, Any}}` : Mapping from time period to generator values and cost
"""

function extract_solution(graph, path)

    solution = Dict{Int, Dict{Symbol, Any}}()  # Dictionary to store node properties

    for node in path
        time_period = get_prop(graph, node, :time_period)
        solution[time_period] = Dict(
            :generator_values => get_prop(graph, node, :generator_values),
            :cost => get_prop(graph, node, :cost)
        )
    end

    return solution
end

"""
    build_new_graph(new_scenarios, time_periods)

Construct a new graph using updated generator scenarios.

# Arguments
- `new_scenarios::Vector{Vector{Tuple{Dict{Int64, Float64}, Float64}}}` : New generator data
- `time_periods::Int` : Number of periods

# Returns
- `MetaDiGraph{Int64, Float64}` : Graph built from new scenarios
"""

function build_new_graph(new_scenarios, time_periods) 

    new_graph = MetaDiGraph()
    defaultweight!(new_graph, 1.0)

    add_vertex!(new_graph)
    source_node = nv(new_graph)
    set_prop!(new_graph, source_node, :time_period, 0)
    set_prop!(new_graph, source_node, :generator_values, 0)
    set_prop!(new_graph, source_node, :cost, 0)

    for t in 1:time_periods
        for (s, scenario) in enumerate(new_scenarios[t])
            add_vertex!(new_graph)
            current_node = nv(new_graph)
            set_prop!(new_graph, current_node, :time_period, t)
            set_prop!(new_graph, current_node, :generator_values, scenario[1])
            set_prop!(new_graph, current_node, :cost, scenario[2])
        end
    end

    add_vertex!(new_graph)
    sink_node = nv(new_graph)
    set_prop!(new_graph, sink_node, :time_period, time_periods + 1)
    set_prop!(new_graph, sink_node, :generator_values, 0)
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
    get_generation_and_ramping_costs(data, info, model)

Compare cost breakdowns between the graph model and the full optimization model.

# Arguments
- `data::Dict{String, Any}` : PowerModels parsed case
- `info::Dict` : Result dictionary from DC graph search
- `model::JuMP.Model` : Full optimized model

# Returns
- `Dict{Symbol, Float64}` : Breakdown of generation and ramping costs
"""

function get_generation_and_ramping_costs(data, info, model)

    graph_model_generation_cost = info[:generation_cost]
    graph_model_ramping_cost = info[:ramping_cost]
    search_model_generation_cost = 0.0
    search_model_ramping_cost = 0.0

    ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
    gen_data = ref[:gen]
    T = model.time_periods
    ramping_data = model.ramping_data

    # sum total generation costs
    for t in 1:T, g in keys(gen_data)
        search_model_generation_cost += 
            gen_data[g]["cost"][1]*value(model.model[:pg][t,g])^2 +
            gen_data[g]["cost"][2]*value(model.model[:pg][t,g]) +
            gen_data[g]["cost"][3]
    end

    # sum total ramping costs
    for t in 2:T, g in keys(gen_data)
        search_model_ramping_cost +=
            ramping_data["costs"][g] * (value(model.model[:ramp_up][t,g]) + value(model.model[:ramp_down][t, g]))
    end

    return Dict(
        :graph_model_generation_cost => graph_model_generation_cost,
        :graph_model_ramping_cost => graph_model_ramping_cost,
        :search_model_generation_cost => search_model_generation_cost,
        :search_model_ramping_cost => search_model_ramping_cost
    )
end

"""
    graph_demands_and_generation(demands, full_model, graph_solution)

Plot demand and generation output from both the graph model and optimal model.

# Arguments
- `demands::Vector{Dict{Int64, Float64}}` : Demand data
- `full_model::JuMP.Model` : Optimal solution model
- `graph_solution::Dict{Int, Dict{Symbol, Any}}` : Generator outputs from graph path
"""

function graph_demands_and_generation(demands, full_model, graph_solution)

    time_periods = length(graph_solution) - 2
    graph_outputs = []

    for i in 1:time_periods
        push!(graph_outputs, sum(values(graph_solution[i][:generator_values])))
    end

    full_model_outputs = Array(value.(full_model.model[:pg]))
    full_model_outputs = vec(sum(full_model_outputs, dims=2))

    demand_to_graph = demands[1:time_periods]
    demand_to_graph = [sum(values(d)) for d in demand_to_graph]


    p1 = plot(full_model_outputs, label="Optimal Model", lw=2)
    plot!(p1, graph_outputs, label="Graph Model", lw=2)
    plot!(p1, demand_to_graph, label="Demand", lw=2)
    xlabel!(p1, "Time Period")
    ylabel!(p1, "Total Generation")
    title!(p1, "Generation vs Demand")
    display(p1)
    savefig("demand_curve.png")

    # Second plot
    p2 = plot(full_model_outputs .- demand_to_graph, label="Optimal Model - Demand", lw=2)
    plot!(p2, graph_outputs .- demand_to_graph, label="Graph Model - Demand", lw=2)
    xlabel!(p2, "Time Period")
    ylabel!(p2, "Difference")
    title!(p2, "Generation Error Compared to Demand")
    display(p2)

end

"""
    output_run_data_to_csv(data, file_path, demands, model, info)

Write summary and time-series data from a run to a CSV file.

# Arguments
- `data::Dict{String, Any}` : PowerModels case data
- `file_path::String` : Path to input .m file
- `demands::Vector{Dict{Int64, Float64}}` : Time-varying demand
- `model::MPOPF.MPOPFModel` : Full optimization model
- `info::Dict` : Results from DC graph search

# Returns
- `String` : Path to the output CSV file
"""

function output_run_data_to_csv(data, file_path, demands, model, info)
    # Extract filename
    filename = split(file_path, "/") |> last
    time_periods = length(info[:solution]) - 2
    
    # Calculate pg values for graph model
    graph_pg_values = []
    for i in 1:time_periods
        push!(graph_pg_values, sum(values(info[:solution][i][:generator_values])))
    end
    
    # Calculate pg values for optimal model
    optimal_model_pg_values = Array(value.(model.model[:pg]))
    optimal_model_pg_values = vec(sum(optimal_model_pg_values, dims=2))
    
    # Calculate demand values
    demand = demands[1:time_periods]
    demand = [sum(values(d)) for d in demand]

    # Calculate ramping and generation costs
    cost_info = get_generation_and_ramping_costs(data, info, model)
    
    # Prepare CSV data
    csv_data = []
    
    # Basic information
    push!(csv_data, ["filename", filename])
    push!(csv_data, ["time_periods", time_periods])
    
    # Graph cost information
    push!(csv_data, ["graph_total_cost", info[:cost]])
    push!(csv_data, ["graph_generation_cost", info[:generation_cost]])
    push!(csv_data, ["graph_ramping_cost", info[:ramping_cost]])

    # Optimal cost information
    push!(csv_data, ["optimal_total_cost", objective_value(model.model)])
    push!(csv_data, ["optimal_generation_cost", cost_info[:search_model_generation_cost]])
    push!(csv_data, ["optimal_ramping_cost", cost_info[:search_model_ramping_cost]])
    
    # Timing information
    push!(csv_data, ["graph_solve_time", info[:time]])
    push!(csv_data, ["optimal_model_solve_time",solve_time(model.model)])
    
    # Violations information
    violations = info[:violations]
    push!(csv_data, ["pmin_pmax_violations", violations[:pmin_pmax_out_of_bounds]])
    push!(csv_data, ["infeasible_model_violations", violations[:infeasible_model]])
    push!(csv_data, ["min_demand_not_met_violations", violations[:min_demand_not_met]])
    
    # Graph information
    if haskey(info, :graph)
        graph = info[:graph]
        # Extract graph properties (adjust based on your graph type)
        push!(csv_data, ["graph_nodes", nv(graph)])
        push!(csv_data, ["graph_edges", ne(graph)])
    end
    
    # Path information
    if haskey(info, :path)
        path_str = join(info[:path], ";")  # Use semicolon to separate path elements
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
    end
    
    # Add separator row before verbose data
    push!(csv_data, ["--- DETAILED TIME SERIES DATA ---", ""])
    
    # Time series data (more verbose, at the end)
    # Create headers for time series
    time_headers = ["time_period_" * string(i) for i in 1:time_periods]
    
    # Graph model pg values
    graph_pg_row = ["graph_pg_sums"; graph_pg_values]
    push!(csv_data, graph_pg_row)
    
    # Full model pg values
    full_model_pg_row = ["full_model_pg_sums"; optimal_model_pg_values]
    push!(csv_data, full_model_pg_row)
    
    # Demand values
    demand_row = ["demand_sums"; demand]
    push!(csv_data, demand_row)
    
    # Individual generator values for each time period (if needed)
    for i in 1:time_periods
        gen_values = info[:solution][i][:generator_values]
        for (gen_id, gen_value) in gen_values
            gen_row = ["t$(i)_generator_$(gen_id)", gen_value]
            push!(csv_data, gen_row)
        end
    end
    
    # Cost breakdown by time period
    for i in 1:time_periods
        if haskey(info[:solution][i], :cost)
            cost_row = ["t$(i)_cost", info[:solution][i][:cost]]
            push!(csv_data, cost_row)
        end
    end
    
    # Full cost history (if you want it)
    if haskey(info, :cost_history)
        cost_hist_row = ["full_cost_history"; info[:cost_history]]
        push!(csv_data, cost_hist_row)
    end
    
    # Write to CSV
    csv_filename = replace(filename, ".m" => "_results.csv")

    
    # Convert to DataFrame for easier CSV writing
    # Since rows have different lengths, we'll write manually
    open(csv_filename, "w") do io
        for row in csv_data
            # Convert all elements to strings and join with commas
            row_str = join([string(x) for x in row], ",")
            println(io, row_str)
        end
    end
    
    println("Results written to: $csv_filename")
    return csv_filename
end