## Search Model usage

There are a few functions that should be understood before jumping into making a local search model and attempting to solve it. 
- factory = DC/ACMPOPFSearchFactory("path/to/case.m", Ipopt.Optimizer)
    : make a factory with our case file and optimizer of choice.

This is just used to reduce the number of parameters entered into the model creation function, as well as to make the factory extensible by having it inherit from a common abstraction. (makes it easy to use the same optimize_model function)

After making the factory, we need to gather demand data and ramping constraint data for the buses in our case file. This is where we call
- dict = PowerModels.parse_file("path/to/case.m")
- PowerModels.standardize_cost_terms!(dict)
- PwerModels.calc_thermal_limits!(dict)

Now that our data (dict) has been initialized, we can generate a demand profile on it.

Call **generate_daily_demand_csv(dict, "path/to/desired/output_dir/", Int64::Num_Time_periods)**. This will spit out a csv in the specified path. From here, we simply need call **ramping_data, demands = parse_power_system_csv("path/to/generated/file", "path/to/case.m")**, which will give us the ramp limits and a vector within a vector of bus demands for each time period.

Finally, use the previous data to call: 
- search_model = create_search_model(factory, Int64::time_periods, ramping_data, demands)
- optimize_model(search_model) (or JuMP.optimize!(search_model.model))

We should now have an optimized model for our given demands and ramping constraints. It should be noted that one should use **include("src/local-search/SearchModel.jl")** and **using .SearchModel** in order to use the provided functions. However there have been problems with this that need to be tracked down and fixed. If it throws problems, using JuMP.optimize!(search_model.model) yields the same result essentially as its the function called in optimize_model() to begin with.