Hint: in vscode there is a button in the top right that enables preview mode for markdown files. It pretties things up quite a bit

# Graph Search
Primarily, this document will focus on the AC version of graph_search, but the procedure for DC is very similar and there is a demo of it inside search_main.jl. Let it also be known that (for graph_search_ac.jl at least) all functions have some kind of docstring attached which should be read when confused (hover over the function or type ?'function_name' into REPL). These docstrings outline the parameter data types, kwargs, return types, and functionality of the associated processes.

## Utilization
To use AC graph search, I'll list the functions needed in order and provide some explanation on them.
It says so already in search_main.jl, but to re-state: The file paths given are relative to your environment being set to the base power-annex folder.

```
matpower_file_path = "src/cases/case14.m"
t = 24
output_dir = "src/local-search/CSV"
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

- Given the file path to the file we just generated, and the file path to our matpower case file (for verification), this reads the csv and returns three dictionaries with all of the ramping costs and limits, as well as the 24 hour demands (or whatever your time period is) for all generators.

`global search_factory_AC = ACMPOPFSearchFactory(matpower_file_path, Ipopt.Optimizer)`

- Creates a factory that holds the matpower file and our desired optimizer to maintain consistency across the models we make in the optimization. This factory also allows type overriding for the next function 'create_search_model'. It would take more parameters and annoyance to remove the factory system instead of keeping it so we leave the current system in, despite concerns over excess complication especially in an experimental codebase.

`search_model_AC = create_search_model(search_factory_AC, t, ramping_data_AC, active_demands_AC, reactive_demands_AC)`  
`optimize_model(search_model_AC)` or `JuMP.optimize!(search_model_AC.model)` works the same, but doesn't print the best cost.

- This is creates a full model with no relaxations (to my knowledge) to be solved by a commercial solver. It doesn't use graph search but instead will be optimized for comparison with graph search models. The answer here *should* be the theoretical optimum, and will take into account the ramping data and demands for multiple periods we created. create_search_model is a relatively simple function, and works to set up model parameters like the data dictionary accessed via search_model_AC.model as an example, or set_model_variables! for JuMP priming. Most of these functions are in model-creation-helpers, which you can check out if trying to change the JuMP side of things.

`global info_AC = AC_graph_search(data, search_factory_AC, active_demands_AC, reactive_demands_AC, ramping_data_AC, t; max_it = 1)`

- When performing AC_graph_search, the function returns a dictionary with the following symbols as keys:  
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
- In general, you should be able to understand what most of them represent without an explanation. The main focus will be on time (how long the function took) and cost which represents the final cost including ramping. AC_graph_search runs almost all of the other functions in the graph_search_AC.jl file, like delta_ac, build_and_optimize_largest_period_AC, etc. If you haven't read it, there is a research paper that explains how everything works (for DC not AC, but still relevant). 

- Some modifications have been made to this function:  
    - 1: Nodes in the shortest path are now marked after evaluation so they aren't re-evaluated during subsequent iterations or if infeasible nodes are removed and new ones are interchanged into the path
    - 2: It has been clarified that the reactive constraint only needs to be added as a JuMP variable for modification of the power variable (S). We don't have additional costs/ramping/vector-perturbing to worry about since elements of a power grid in real life can inject reactive power almost freely. (It doesn't cost effort to make imaginary power the same way it does for real)
    - 3: The max_it argument in graph_search_AC is trivial if you are familiar with the algorithm, but it should be known that since the current implementation runs into **severe** slowdown during later iterations, it is necessary to limit it to only ~ 1-5 runs, otherwise it could calculate for an hour and return no result
    - 4: Swapped generator fixing across scenarios, to instead be the same across time periods as a temporal linkage attempt
    - 5: Added a fallback + low cost option to applicable scenarios during local node generation. This takes the previously found lowest cost/generation period which is known to be feasible, and adds it as an option in the graph to any time periods with demands lower than said node's production. Of all the changes, this has had a noticably positive impact. I've run up to 7 iterations now in a run, with notably less infeasible models removed from our node graph, which ended in a cost of 1.71e5, vs Ipopt's usual 1.65-1.66e5 (this is close compared to prior attempts). Tests so far have been conducted on case 14 only for reference.

- After running the function, you should now have a solution which is feasible in AC space and you only need compare statistics from the return value.

- The solution key returns a dictionary with the final generation values and node cost at each time period. So  
`info[:solution][1][:active_generator_values]`  
Returns a dictionary of time period 1's optimal solution.

`optimal_cost = objective_value(search_model_AC.model)`  
`graph_cost = info_AC[:cost]`

- The last two lines of code are merely for comparison purposes to see whether graph search was able to produce an answer comparable to the commercial solver.

At this point you know how to use the graph search algorithm, the next step is likely to make improvements on existing code or methods. Hopefully that shouldn't be too hard now that some documentation exists within the AC file regarding input data types, functionality and some lines to start looking around in comments at the top. If you plan to modify existing code, either adding new functions or splitting the main ac_graph_search function into smaller more modular bits, make sure to continue the docstring format -  
(Triple quotation marks: """ ~ """) for future viewers and for self understanding as much as possible. 

As well, be mindful that while Julia is a fast and smart language, you can still help the compiler optimize your code by giving everything types as much as possible (Ex. `foo::Int64`, `bar::String`), with the added benefit of helping other programmers be able to work with your data easier. 

That was a long section, if you made it this far then thanks for reading, I hope it helps!

## Graphing Results
Sometimes you may want to create graphs of the result from the optimization for demo purposes or to visualize how any modifications made to the algorithm performed. Helpful tools like the Plots package or PlotlyJS allow in-editor graphs to be made easily, if there is something lacking from a graph or a new kind of graph desired, look up the library and consider pairing it with Dataframes for ease of use. 

For our purposes, the following function comes in handy:

`graph_demands_and_generation_AC(active_demands_AC, reactive_demands_AC, full_model, graph_solution)`

- To gather the graph solution, you can use the info dict returned by AC_graph_search as follows: `graph = info[:graph]`  
Then to get the model use the optimized one we made prior: `full_model = search_model_AC.model`  
The active and reactive demands will be the same ones returned from: `parse_AC_power_system_csv`

The function will then create 3 comparison graphs for us, and save them as png files

## Making CSV data from outputs

If, instead of graphing or using terminal commands to sift through data, you want to make a CSV file with all the results, the following command may be of use:

`output_run_data_to_csv_AC(data, file_path, active_demands, reactive_demands, model, info)`

- Most of the parameters should look familiar but I'll give a brief rundown again:
    - data refers to a case dictionary created using the following sequence: 
    ```
    data = PowerModels.parse_file(factory.file_path)
    PowerModels.standardize_cost_terms!(data, order=2)
    PowerModels.calc_thermal_limits!(data)
    ```
    - file_path in this case refers to the folder where you want to *store* the new CSV
    - active and reactive demands come from `parse_AC_power_system_csv`  
    - model can be obtained via a previously created and optimized search model using `search_model_AC.model`  
    - info refers to the returned dictionary from `AC_graph_search` 
 
- The function will use other commands in your stead to gather necessary data from the results of our optimization such as generation and ramping costs before appending them to a buffer and spitting out a CSV with all the data one could want. This includes **graph path data**, **cost improvement metrics**, **graph iteration history**, **overall cost**, **ramping cost**, **generation cost**, **optimal vs graph solution cost**, **time comparisons**, **individual generator data**, **feasibility violations** and *more*. 