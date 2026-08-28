include("rl_functions.jl")
# SearchModel is a module included in rl_functions, this just specifies it for open use in REPL now
using .SearchModel
using ReinforcementLearning, Ipopt

matpower_case_path = "src/cases/case14.m"
factory = ACMPOPFSearchFactory(matpower_case_path, Ipopt.Optimizer)
time_periods::Int64 = 24
output_dir = "src/rl-ac/CSV"

environment = BusEnv(init_model(factory, time_periods, output_dir; date="2025-10-04"))

RLBase.test_runnable!(env)

run(RandomPolicy(action_space(env)), env, StopAfterNEpisodes(1000))

hook = TotalRewardPerEpisode()

run(RandomPolicy(action_space(env)), env, StopAfterNEpisodes(1000), hook)

using Plots

plot(hook.rewards)


function training_loop(env::BusEnv)
    # Reset environment
    # Observe initial state
    # For each step: 
        # select an action from current policy
        # apply it to the environment model
        # compute reward
        # observe the next state and discounting
        # update policy after desired number of steps
    # After training, we can benchmark the agent against ipopt
end