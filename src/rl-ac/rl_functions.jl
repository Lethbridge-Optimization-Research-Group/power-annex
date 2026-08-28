include("../local-search/SearchModel.jl")
using ReinforcementLearning, JuMP, PowerModels, CommonRLSpaces

# Create a new environment by type overriding the following interfaces
#=
action_space(env::YourEnv)
state(env::YourEnv)
state_space(env::YourEnv)
reward(env::YourEnv)
is_terminated(env::YourEnv)
reset!(env::YourEnv)
act!(env::YourEnv, action)
=#

# The below is all deprecated, I've since read papers that change entirely how I want to go about implementation

"""
    init_model(model_factory, periods, output_dir)
Initializes the model we will use in an environment. Uses the AC local search model as this
experiment is supposed to be for AC. Though with the right abstracting and modifying the input parameter type, you could likely
extend it to take a different AC-based model factory as long as you change the model creation function below.

# Arguments
- `model_factory::ACMPOPFSearchFactory`: A factory containing a file path and optimizer
- `periods::Int64`: Number of time periods. Currently only takes 24 for safety, may be able to use more with RL now...
- `output_dir::String`: File path for storing 
- `date::String`: Optional argument to specify the date you want to mimic the demands of from the public data we have (yyyy-mm-dd)

# Returns
- `model::ACMPOPFSearchModel`: A JuMP model that has been
"""
function init_model(model_factory::ACMPOPFSearchFactory, periods::Int64, output_dir::String; date::String = "2025-10-01")::AbstractMPOPFModel
    data = PowerModels.parse_file(model_factory.file_path)
    PowerModels.standardize_cost_terms!(data, order=2)
    PowerModels.calc_thermal_limits!(data)

    ramping_file = generate_ac_vector_demand_csv(data, output_dir, get_date_percentages("src/local-search/CSV/PUB_Demand_2025.csv", date))
    ramping_data, active_demands, reactive_demands = parse_AC_power_system_csv(ramping_file, model_factory.file_path)
    return create_search_model(model_factory, periods, ramping_data, active_demands, reactive_demands)
end

"""
    BusEnv inherits from the abstract environment type defined in ReinforcementLearning.jl.
    We set it up so it works with the generalized interfaces of the RL package, and since it is
    user-defined we have freedom to add whatever parameters we need.

    Note: We can run a function called RLBase.test_runnable! to determine if our environment is set up correctly

    # Fields
    - `model::AbstractMPOPFModel`: A bus network model we want to make optimizations on
    - `reward::Float64`
"""
Base.@kwdef mutable struct BusEnv <: AbstractEnv

    model::AbstractMPOPFModel

    reward::Float64

    terminated::Bool

    objective::Float64
    violations::Float64
    reference_objective::Float64
end

"""
    RLBase.action_space(environment::BusEnv)

I'm using an old library CommonRLSpaces which has some interfaces for making discrete and continuous action spaces.
In this case, the function uses a multi-dimension continuous space "Box" to allow the our agent to sample from 
nearly infinite actions on a defined interval. This gives freedom to modify generator values by smaller and smaller 
amounts for precision, instead of predefined delta steps like in the search_model.

We define an action as being a vector with real floating point values, each of which corresponding to an additive 
modification on a generator's active power generation. (pg_i + action_i = new pg_i, where action_i can be 
a positive or negative value).

The actions should be iterated through via keys(data[:gen]). If there are 6 generators, then there will be 6 elements in the action,
corresponding element-wise to their intended gen.

# Arguments
- `environment::BusEnv`: Our environment which contains generator limits for use in defining action space limits
"""
function RLBase.action_space(env::BusEnv)
    # There are a few ways to implement the bounds. We could generate an action in the whole generator space, or
    # we could pass in a delta that allows action within a range on either the min or max, or the current generation.
    # For example, a +-5% current generation range, or anywhere in the min or max that we directly set as the new generation.
    # Probably additive will be the best, but giving access to the whole range may allow faster convergence(?)
    gen_data = env.model.data[:gen]
    upper_limits::Vector{Float64}[]
    lower_limits::Vector{Float64}[]
    for gen in keys(gen_data)
        upper_limits.push!(gen_data[gen][:pmax])
        lower_limits.push!(gen_data[gen][:pmin])
    end
    
    return Box(lower_limits, upper_limits)
end

#= 
Insert modified bellman equasion here. Im pretty sure this is purely for a reward calculation,
and it policy considerations factor more into the act! function.

Based on studies I have read, using single step episodes on models typically does better for power flow.
However, it was mentioned that for time-period models, using multiple steps before updating the reward
may be better.
=#
function RLBase.reward(env::BusEnv) 
    error("Not implemented")
end

# We can create a continuous space representing the states we care about like generation.
# Alternatively, we can make a data structure with many of these spaces representing different values.
# Maybe one is used for generation, one for the objective cost, one for 
function RLBase.state_space(env::BusEnv) 
    # return ArraySpace(Float64, dimensions)
    error("Not implemented")
end

# Return the desired state values from the model
# maybe this looks like a 5 column, n-bus row matrix, with each column being a data point like voltage
# but if our agent only cares about generation, I can instead return only those values, then pull the feasibility from the env
# separately to inform policy in an action. Something like a 3xn matrix, with bus_id, pg, qg
function RLBase.state(env::BusEnv, ::Observation, ::DefaultPlayer) # Not sure what default player is, but its madatory as per the package
    error("Not implemented")
end

# We can either write the termination status to the env somewhere else, or turn this into its own function.
function RLBase.is_terminated(env::BusEnv)
    error("Not implemented")
end

# We need to reset the model's values. Maybe rewriting with a fresh model construction + case read.
# Probably also the reward, objective, termination status and violations. 
function RLBase.reset!(env::BusEnv) 
    error("Not implemented")
end

# Placeholder code for an action. Each action should update the reward in our environment
function RLBase.act!(env::BusEnv, action)
    error("Not implemented")
end