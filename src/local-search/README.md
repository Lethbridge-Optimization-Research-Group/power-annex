Hint: in vscode there is a button in the top right that enables preview mode for markdown files. It pretties things up quite a bit

# Graph Search
Primarily, this document will focus on the AC version of graph_search, but the procedure for DC is very similar and there is a demo of it inside search_main.jl. Let it also be known that (for graph_search_ac.jl at least) all functions have some kind of docstring attached which should be read when confused (hover over the function or type ?'function_name' into REPL). These docstrings outline the parameter data types, kwargs, return types, and functionality of the associated processes.

## Utilization
To use AC graph search, I'll list the functions needed in order and provide some explanation on them.

```
matpower_file_path = "../cases/case14.m"
t = 24
output_dir = "./CSV"
data = PowerModels.parse_file(matpower_file_path)
PowerModels.standardize_cost_terms!(data, order=2)
PowerModels.calc_thermal_limits!(data)
```

- The above code sets constants for ease of use and readability of functions, as well as setting our time periods to 24 (24 hours in a day), and most importantly setting up a case dictionary using a MatPower case file named data. This is the basis of the entire optimization problem, and contains all base constraint values, static loads, generators, arcs, etc... In this case the dictionary is accessed via strings, so data["gen"] for example. Some functions however will provide the same dictionary using PowerModels.build_ref, the only difference being the ref is accessed using symbols (data[:gen]) instead. Make sure you are aware of the one you are using because passing in a symbol dict to graph search functions is likely to cause an access error. 

`hourly_multipliers = get_date_percentages("path/to/PUB_Demand_'year'.csv", 'yy/mm/dd')`

- This bit grabs the public demand data we were given by ontario, and spits out a vector containing floating point representations of each hour's percentage of the maximum demand for a given day. For example, if 8:00 PM was the highest demand period, and had 2 units of demand, while 1:00 AM only has 1 unit, then the vector has 0.5 in position 1. The vector being 1 indexed, starts from 1 AM and ends at hour 24 or 12 AM. These multipliers could theoretically be extrapolated to make more time periods for more accuracy, but the data we were given only provides hour-by-hour data so that is what is used. Most AC functions go off of the number of time periods passed in so if someone did get a multiplier vector for say 60 time periods, it wouldn't (shouldn't) be hard to scale up our graph search with it. 

- These multipliers represent demand over a day, and are used to create a curve over which to optimize an MPOPF model.

`ramping_csv_file_AC = generate_ac_vector_demand_csv(data, output_dir, hourly_demand_multipliers; seed=1)`

- This creates a custom CSV file based on our demand multipliers which provides a basis for generator loads over a day. The base demands given by the data dict are simply multiplied each time period and are scaled simply. There is no modeling of how certain buses may only see high loads at certain times of day however, such as residential areas only having high demand at night and in the morning as an example. There is some noise injected for each time period, but you can specify a seed as a keyword argument during the call to make the same file repeatedly. This function also goes through our data dictionary, and finds the generator ramping limits, then creates some ramping cost coefficient for each generator based on a random number from 100 to 300. As far as I am aware this range is unscientific, and can be changed very easily. The data here is important for the ramping constraint portion of Multi Period Optimal Power Flow (MPOPF) problems, as well as providing time-period seperated demands for each generator, both active and reactive.

`ramping_data_AC, active_demands_AC, reactive_demands_AC = parse_ac_power_system_csv(ramping_csv_file_AC, matpower_file_path)`

- 

global search_factory_AC = ACMPOPFSearchFactory(matpower_file_path, Ipopt.Optimizer)

search_model_AC = create_search_model(search_factory_AC, t, ramping_data_AC, active_demands_AC, reactive_demands_AC)

optimize!(search_model_AC.model)

global info_AC = AC_graph_search(data, search_factory_AC, active_demands_AC, reactive_demands_AC, ramping_data_AC, t)

optimal_cost = objective_value(search_model_AC.model)
graph_cost = info_AC[:cost]`