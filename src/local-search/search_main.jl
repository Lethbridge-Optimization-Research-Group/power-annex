using JuMP, Ipopt, Gurobi, Serialization, Random, Graphs, MetaGraphs, MathOptInterface
using PowerModels, Statistics, Plots, GraphRecipes, DataFrames
include("SearchModel.jl")
using .SearchModel # Local module. Remember since we use this, you have to restart REPL and rerun this code
# each time a modification is made to the code in SearchModel.jl

#= 
Note that include functions will allow relative pathing from the folder they are executed (in this case the local-search folder)
but when assigning file paths to below variables, the are relative to the environment/REPL, not the file you run them in.
=#

# This file is an example for how to run GraphSearch and compare to a standard local-search MPOPF

matpower_file_path = "src/cases/case14.m"

t = 24 # Current functions only support up to 24 time periods

output_dir = "src/local-search/CSV/"

# Makes the reference dictionary
data = PowerModels.parse_file(matpower_file_path)
PowerModels.standardize_cost_terms!(data, order=2)
PowerModels.calc_thermal_limits!(data)

# # These are used in some commented-out code below
# global total_cost_diffs = []
# global ramping_costs_diffs = []
# global iterations_vec = []
# global time_vec = []

### DC ###

# ramping_csv_file_DC = generate_daily_demand_csv(data, output_dir)

# ramping_data_DC, demands_DC = parse_power_system_csv(ramping_csv_file_DC, matpower_file_path)

# global search_factory_DC = DCMPOPFSearchFactory(matpower_file_path, Gurobi.Optimizer)
# search_model_DC = create_search_model(search_factory_DC, t, ramping_data_DC, demands_DC)

# optimize!(search_model_DC.model)

# global info_DC = DC_graph_search(data, search_factory_DC, demands_DC, ramping_data_DC, t)

# #filename = split(matpower_file_path, "/") |> last
# optimal_cost = objective_value(search_model_DC.model)
# graph_cost = info_DC[:cost]

### AC ###

hourly_demand_multipliers = get_date_percentages("src/local-search/CSV/PUB_Demand_2025.csv", "2025-10-01") # Makes demand curve from public data files

ramping_csv_file_AC = generate_ac_vector_demand_csv(data, output_dir, hourly_demand_multipliers) # You can optionally include a seed here

ramping_data_AC, active_demands_AC, reactive_demands_AC = parse_AC_power_system_csv(ramping_csv_file_AC, matpower_file_path)

global search_factory_AC = ACMPOPFSearchFactory(matpower_file_path, Ipopt.Optimizer)

search_model_AC = create_search_model(search_factory_AC, t, ramping_data_AC, active_demands_AC, reactive_demands_AC)

optimize_model(search_model_AC)
# Alternatively: JuMP.optimize!(search_model_AC.model)

global info_AC = AC_graph_search(data, search_factory_AC, active_demands_AC, reactive_demands_AC, ramping_data_AC, t; max_it = 7)

optimal_cost = objective_value(search_model_AC.model)
graph_cost = info_AC[:cost]

#=
for x in info[:cost_history]
    println(x, ',')
end
println()
for x in info[:time_vec]
    println(x, ',')
end
=#
#=
percent_decrease = costs ./ costs[1] .* 100
plot(percent_decrease,
            xlabel="Iteration",
            ylabel="% of initial cost",
            title="Cost Decrease Over Iterations",
            linewidth=2,
            legend=false)
=#

#=
global optimal_sum = 0
global graph_sum = 0
for x in total_cost_diffs
    global optimal_sum += x[1]
    global graph_sum += x[2]
end

ratio = graph_sum / optimal_sum

average_iter = 0
for x in iterations_vec
    global average_iter += x[1]
end
global average_iter = average_iter / 10
#end
# If wanting to graph the Cost History
    
    
    models = []
    global total_cost = 0
    for i in 1:t
        model = create_search_model(search_factory, 1, ramping_data, [demands[i]])
        optimize!(model.model)
        push!(models, model)
        global total_cost += objective_value(model.model)
    end
    
    pg_values_by_t = []
    for i in 1:t
        values = [value(models[i].model[:pg][key]) for key in keys(models[i].model[:pg])]
        pg_values = Dict(zip(models[i].model[:pg].axes[2], values))
        push!(pg_values_by_t, pg_values)
    end
    
    # Calculate total ramping cost
    global total_ramping_cost = 0.0
    for t in 2:t
        for gen_id in ramping_data["gen_id"]
            gen_key = Int(gen_id)
            current_power = pg_values_by_t[t][gen_key]
            previous_power = pg_values_by_t[t-1][gen_key]
            ramp_amount = abs(current_power - previous_power)
            global total_ramping_cost += ramp_amount * ramping_data["costs"][gen_key]
        end
    end
    
    individual_cost = total_cost

    ramping_figures = get_generation_and_ramping_costs(data, info, search_model)
    optimal_ramping_costs = ramping_figures[:search_model_ramping_cost]
    graph_ramping_costs = ramping_figures[:graph_model_ramping_cost]
    individual_ramping_costs = total_ramping_cost

    push!(total_cost_diffs, (optimal_cost, graph_cost, individual_cost))
    push!(ramping_costs_diffs, (optimal_ramping_costs, graph_ramping_costs, individual_ramping_costs))
    
    plot(info[:cost_history], 
    label="Optimization Cost", 
    title="Cost History : $matpower_file_path",
    xlabel="Iteration", 
    ylabel="Cost",
    linewidth=2,
    marker=:circle,
    markersize=3)
    
global individual_sum = 0
global optimal_ramping_sum = 0
global graph_ramping_sum = 0
global individual_ramping_sum = 0


for x in ramping_costs_diffs
    global optimal_ramping_sum += x[1]
    global graph_ramping_sum += x[2]
    global individual_ramping_sum += x[3]
end
total_cost_difference = graph_sum / optimal_sum
total_ramping_difference = individual_ramping_sum / graph_ramping_sum

#=
plot(info[:cost_history], 
label="Optimization Cost", 
title="Cost History : case14",
xlabel="Iteration", 
ylabel="Cost",
linewidth=2,
marker=:circle,
markersize=3)
title = time()
#savefig("second_iteratoin.png")
=#

# If wanting to test with PowerModelds PF
#test_model = PowerModels.solve_pf(data, DCPPowerModel, Gurobi.Optimizer)

#=
for t in 1:5
    x = info[:solution][t]
    println()
    for (gen_id, gen_data) in data["gen"]
        gen_id_int = parse(Int, gen_id)
        if x[:generator_values][gen_id_int] < gen_data["pmin"]
            println("Pmin violation on $gen_id")
        end
        if x[:generator_values][gen_id_int] > gen_data["pmax"]
            println("Pmax violation on $gen_id")
        end 
    end 
end

for t in 1:5
    x = info[:solution][t]
    println()
    for (gen_id, gen_data) in data["gen"]
        gen_id = parse(Int, gen_id)
        println("Gen ID: $gen_id")
        println("Gen pmin: ", gen_data["pmin"])
        println("Gen Pg: ", x[:generator_values][gen_id])
        println()
    end 
end
=#
=# 

#=
loads = data["load"]

bus_loads = Dict{Int, Dict{Symbol, Float64}}()

for (k, v) in loads
    bus = v["load_bus"]
    pd  = v["pd"]
    qd  = v["qd"]
    bus_loads[bus] = Dict(:pd => pd, :qd => qd)
end

# collect data
buses = sort(collect(keys(bus_loads)))  # bus numbers in order
pds   = [bus_loads[b][:pd] for b in buses]
qds   = [bus_loads[b][:qd] for b in buses]

# plot
plot(#=buses,=# (pds,qds); label="Pd", lw=0, marker=:circle, markersize=1, color=:blue)
#plot!(#=buses,=# qds; label="Qd", lw=1, marker=:circle, markersize=1, color=:red)

xlabel!("Bus")
ylabel!("Load (p.u.)")
title!("$matpower_file_path")
=#
#=
data2023 = parse_csv_data("./Attachments/PUB_Demand_2023.csv")
data2024 = parse_csv_data("./Attachments/PUB_Demand_2024.csv")


average2023 = get_hourly_average(data2023)
average2024 = get_hourly_average(data2024)
average2025 = get_hourly_average(data2025)

percent2023 = percentages_of_max_demand(average2023)
percent2024 = percentages_of_max_demand(average2024)
percent2025 = percentages_of_max_demand(average2025)
total_averages = []
for i in 1:24
    push!(total_averages, (percent2023[i] + percent2024[i] + percent2025[i]) / 3)
end
=#
