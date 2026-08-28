# Reinforcement Learning Experiment
In this, the aim is to circumvent the currently ultra-slow processing of our regular local-search model in AC-space by using reinforement learning. I utilize a package called ReinforcementLearning to provide a structure for everything, read the documentation for that if you want to make changes or continue this. 

**Disclaimer: I have very little experience with reinforcement learning and machine learning in general. Don't expect cutting-edge methods or scientific approaches to the process. If there appears to be an obvious improvement to be made, its likely I didn't know about it and you should feel free to make changes.**

## rl_functions.jl

This file is used to manage the ReinforcementLearning package's interface. It provides several abstractions that need overriding and defining to use, so this file does just that.

**BusEnv(::ACMPOPFSearchModel)**  
- This struct is the most basic block of our RL environment. It provides all of the data we need in order to make observations and model improvements. You should seek to modify the data/model inside in ways that approach optimality, then assign reward to the environment space (Required by the RL package) based on how close it is to Ipopt's result for example.
- Importantly, it will contain a full JuMP model of our bus network if you initialize it properly. Our actions and states should work with this to lead the RL model effectively.  

**Init_model(factory, periods, output_dir)**
- There is documentation above where this function is defined in-file, but essentially you need only call **BusEnv(init_model(factory, periods, output_dir; date))** to create the environment and set its model. The date is pretty optional, most days of the year have similar enough demand profiles to not make a major difference.