include("../local-search/SearchModel.jl")
using ReinforcementLearning, JuMP, PowerModels, CommonRLSpaces

# Create a new environment by type overriding the following interfaces at a minimum.

#=
action_space(env::YourEnv)
state(env::YourEnv)
state_space(env::YourEnv)
reward(env::YourEnv)
is_terminated(env::YourEnv)
reset!(env::YourEnv)
act!(env::YourEnv, action)
=#
"""
A convenient package carrying useful data for calculating the results of our current network system state

# Fields
- `converged::Bool`: Simple bool describing relative convergence satisfaction
- `voltage::Vector{ComplexF64}`: Vector containing bus voltages with real and imaginary portions
- `pg`: Active generation at each generator
- `qg`: Reactive generation at each generator
- `line_loads`: Loading across all line arcs (monodirectional)
- `residual`: A calculated value representing how far we are from a tolerable result based on current generation (overall
generation differences, not total cost. This should be modified since we won't know ideal generation)
"""
struct PFresult
    converged::Bool
    voltage::Vector{ComplexF64}
    pg::Vector{Float64}
    qg::Vector{Float64}
    line_loads::Vector{Float64}
    residual::Float64
end

function init_model(model_factory::ACMPOPFSearchFactory, periods::Int64, output_dir::String; date::String = "2025-10-01")::AbstractMPOPFModel
    data = PowerModels.parse_file(model_factory.file_path)
    PowerModels.standardize_cost_terms!(data, order=2)
    PowerModels.calc_thermal_limits!(data)

    ramping_file = generate_ac_vector_demand_csv(data, output_dir, get_date_percentages("src/local-search/CSV/PUB_Demand_2025.csv", date))
    ramping_data, active_demands, reactive_demands = parse_AC_power_system_csv(ramping_file, model_factory.file_path)
    return create_search_model(model_factory, periods, ramping_data, active_demands, reactive_demands)
end

"""
EcoDispatchEnv inherits from the abstract environment type defined in ReinforcementLearning.jl.
We set it up so it works with the generalized interfaces of the RL package, and since it is
user-defined we have freedom to add whatever parameters we need.

Note: We can run a function called RLBase.test_runnable! to determine if our environment is set up correctly

# Fields
- `network_model::ACMPOPFSearchModel`: Custom power model with all network data, and additional benefits like ramping data and demands
- `state::Vector{Float64}`: Current model observable constraint values (voltage, )
- `last_action::Vector{Float64}`: Our last action vector for initial fallback safety
- `fallback_action::Vector{Float64}`: A fallback action thats conservative and acts as a second line of constraint safety
- `result::PFResult`: The result obtained from acting upon the model network (convergence, pg, qg, voltage, line loads, residual)
- `terminated::Bool`: Whether the RL epoch is finished or not
- `t::Int64`: run count, need to confirm
- `horizon::Int`: Might remove...
- `load_scale::Float64`: Might remove...
- `Fallback_count::Int64`: How many times the agent has had to resort to a fallback action after a mistake
"""
Base.@kwdef mutable struct EcoDispatchEnv <: AbstractEnv

    network_model::ACMPOPFSearchModel

    state::Vector{Float64}
    last_action::Vector{Float64}
    fallback_action::Vector{Float64}

    result::PFResult
    terminated::Bool
    t::Int
    horizon::Int

    load_scale::Float64
    fallback_count::Int
end

function EcoDispatchEnv(model; horizon=24)
    ng = length(model.data[:gen])

    dummy_fallback = PFresult(
        false,
        ones(ComplexF64, length(model.data[:bus])),
        zeros(ng),
        zeros(ng),
        zeros(length(model.data[:branch])),
        Inf
    )

    env = EcoDispatchEnv(
        model,
        zeros(2 * length(model.data[:bus]) + length(model.data[:branch])), # This is a vcatted array
        zeros(ng),
        zeros(ng),
        dummy_fallback,
        false,
        0,
        horizon,
        1.0,
        0
    )

    reset!(env)
    return env
end

"""
    RLBase.action_space(env)

I'm using an old library CommonRLSpaces which has some interfaces for making discrete and continuous action spaces.
In this case, the function uses a multi-dimension continuous space "Box" to allow the our agent to sample from 
nearly infinite actions on a defined interval. 

In this case, we will sample from the continuous floating point interval [-1, 1] for each generator
which can be used to scale values from pmin to pmax at -1 and 1 respectively. (contains generation)

# Arguments
- `environment::EcoDispatchEnv`: Our environment which contains generator counts
"""
function RLBase.action_space(env::EcoDispatchEnv)
    return ArraySpace(-1.0..1.0, length(env.fallback_action))
end

# Doesn't include ramping yet
function generation_cost(env::EcoDispatchEnv)
    cost = 0.0

    for gen in env.model[:gen]
        cost += ((gen[:cost][1] * gen[:pg]^2) + (gen[:cost][2] * gen[:pg]) + gen[:cost][3])
    end

    return cost
end

#= 
Many ways to do this, for example adding a reward barrier that prohibits reward gain until a feasible solution is found.
=#
function RLBase.reward(env::EcoDispatchEnv)
    # Somewhat arbitrary large negative reward for non-convergence. might need scaling for larger systems and such
    if env.terminated && !env.result.converged
        return -1.0e5
    end

    cost = generation_cost(env)
    # 1% penalty for every fallback used to encourage less of them
    fallback_penalty = (0.01 * env.fallback_count) * cost

    return -cost - fallback_penalty
end

"""
    action_to_gen(env, action)

Takes the RL model's output which is a single dimensional vector of values from -1 to 1, 
and produces usable generation setpoints by adjusting a set midpoint based on the action value
for a specific generator, and adding it to the minimum generation. for example, an action of 1.0
would cause min + 2*(min-max)/2 which just sets generation to the maximum.

Because the min setpoint is fixed, and the addition is calculated every time, we cannot produce generation
outside of the active min/max. This could be extended to reactive as well, I'm just facing time constraints.

# Arguments
- `env::EcoDispatchEnv`: The given environment for our agent to interact with
- `action::Vector{Float64}`: a series of continuous values between -1 and 1 dictating active generation

# Returns
- `active::Vector{Float64}`: The new Pg values for use in mutating network gens
"""
function action_to_gen(env::EcoDispatchEnv, action::Vector{Float64})
    active = similar(action, Float64)

    for k in eachindex(action)
        gen = env.model.data[:gen]
        fixed_action = clamp(Float64(action[k]), -1.0, 1.0) # Reinforces domain limits so actions stay between 1 and -1
        active[k] = gen[:pmin] + (fixed_action + 1.0) * 0.5 * (gen[:pmax] - gen[:pmin])
    end

    return active
end

"""
    state_space(env::EcoDispatchEnv)
"""
function RLBase.state_space(env::EcoDispatchEnv) 
    return ArraySpace(-2.0)
end

"""
    get_state(env::EcoDispatchEnv)

Makes a state item out of environment parameters to be observed later, things like voltages, angles and line loads.

# Returns
- `state::Vector{Float64}`: A long vector of length 3n, with 'vertically concatenated' values for voltage magnitude, angle,
and line loads in that order. In a 2 bus system, this means [|V1|, |V2|, angle1, angle2, line_load1, line_load2]
"""
function get_state(env::EcoDispatchEnv)
    v = env.voltage

    return Float64[
        abs.(v); # The dot applies abs over all elements of the vector
        angle.(v);
        env.result.line_loads
    ]
end

# Return the desired state values from the Model
# From examples in papers, it seems just active and reactive power generation
# is generally enough. In this case we observe voltage and line loads for constraint analysis
function RLBase.state(env::EcoDispatchEnv)
    return env.state
end

# We can either write the termination status to the env somewhere else, or turn this into its own function.
# Determines epoch end, which in our case may be just 1 step.
function RLBase.is_terminated(env::EcoDispatchEnv)
    return env.is_terminated
end


"""
    reset!(env::EcoDispatchEnv)

    Resets and initializes an environment with dummy values after a run, 

"""
function RLBase.reset!(env::EcoDispatchEnv) 
    env.terminated = false
    env.t = 0
    env.fallback_count = 0

    env.last_action .= env.fallback_action
    env.fallback_action .= 0.0 # midpoint generation

    # Try various start points to find a feasible fallback point. Not an elegant approach,
    # just a placeholder in case I or someone else thinks of something better
    n_gens = length(env.model.data[:gen])
    candidates = [
        zeros(n_gens),
        fill(0.25, n_gens),
        fill(0.50, n_gens),
        fill(0.75, n_gens),
        ones(n_gens)
    ]

    found = false

    for candidate in candidates
        result, gen_values = evaluate_action(env, candidate)

        if safe_state(env, result, gen_values)
            env.fallback_action .= candidate
            env.last_action .= candidate
            env.result = result
            env.s .= make_state(env)
            found = true
            break
        end
    end

    found || error("No feasible fallback action was found")

    env.last_action .= env.fallback_action # In case it changed after the midpoint generation
    env.result = result
    env.state .= get_state(env)

    return env
end

# Placeholder code for an action. Each action should update the reward in our environment
# This is where we simply update generator values and such to match whatever action has been passed in.
function RLBase.act!(env::EcoDispatchEnv, action)
    error("Not implemented")
end