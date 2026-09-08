include("rl_functions.jl")
# SearchModel is a module included in rl_functions, this just specifies it for open use in REPL now
using .SearchModel
using ReinforcementLearning, Ipopt

matpower_case_path = "src/cases/case14.m"
factory = ACMPOPFSearchFactory(matpower_case_path, Ipopt.Optimizer)
time_periods::Int64 = 24
output_dir = "src/rl-ac/CSV"

environment = EcoDispatchEnv(init_model(factory, time_periods, output_dir; date="2025-10-04"))

RLBase.test_runnable!(env)

run(RandomPolicy(action_space(env)), env, StopAfterNEpisodes(1000))

run(RandomPolicy(action_space(env)), env, StopAfterNEpisodes(1000), TotalRewardPerEpisode())

using Plots

plot(hook.rewards)