# Reinforcement Learning Experiment
In this, the aim is to circumvent the currently ultra-slow processing of our regular local-search model in AC-space by using reinforement learning. I utilize a package called ReinforcementLearning to provide a structure for everything, read the documentation for that if you want to make changes or continue this. 

**Disclaimer: I have very little experience with reinforcement learning and machine learning in general.**

Update:  
- I've come across a paper describing safe fallback methods for rl in power flow. It does seem possible to use rl standalone to solve opf, while obeying all constraints almost completely and not using a solver like ipopt. This is perfect, but unfortunately trying to mimic this model alone is difficult enough to wrap my head around, let alone extrapolating it out to be only a helper in local search node creation. As a result, most of what has been written is now useless and needs reconsideration...

My thoughts are: Try experimenting recreating a solo model, and once you have a hand on that, try to downsize or modify it for use in local search. If done well with constraints built into the rl process, the suggested points should always pass feasibility tests in our search.

## rl_functions.jl

This file is used to manage the ReinforcementLearning package's interface. It provides several abstractions that need overriding and defining to use, so this file does just that.

**BusEnv(::ACMPOPFSearchModel)**  
- This struct is the most basic block of our RL environment. It provides all of the data we need in order to make observations and suggest improvements. You can then assign reward to the environment space (Required by the RL package) based on inverse cost for example (lower cost = higher reward). This environment is unfinished

**Init_model(factory, periods, output_dir)**
- There is documentation above where this function is defined in-file, but essentially you need only call **BusEnv(init_model(factory, periods, output_dir; date))** to create the environment and set its model. The date is pretty optional, most days of the year have similar enough demand profiles to not make a major difference. This is unfinished and may not be necessary even...