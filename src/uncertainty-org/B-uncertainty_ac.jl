# =====================================================
# B-uncertainty_ac.jl
#
# Driver file for uncertainty experiments on AC OPF.
# Based on main_epsilon_ac.jl
#
# Author: Shidratul Muntaha
# =====================================================


#= configuration variables =#
CASE_DIR = "/Users/shidratulmuntaha/power-annex/.github/workflows/cases/"
CASE_FILE = "case118.m"
ROOT_DIR = "/Users/shidratulmuntaha/power-annex/"

#= ***********
include first the files with the needed functions
**************
=#
include(ROOT_DIR * "src/basemodels-org/B-uncertainty_ac_opf.jl")
include(ROOT_DIR * "src/util-org/read_case.jl")

#= build the ref dictionary with the case data
=#
ref = read_case(CASE_DIR * CASE_FILE)

# Uncertainty scaling factor
lambda = 2.1 #least feasible point for this system

#= create the model for Ipopt
=#
model = init_ac()

result = solve_model_ac!(ref, model,lambda)

# Check that the solver terminated without an error
println("The solver termination status is $(result[:status])")

# Check the value of the objective function
println("The cost of generation is $(result[:cost]).")

# output execution time (sec)
println("Execution time for optimization: $(result[:time_sec]) sec")

println("Power flow on lines: $(result[:p_arcs])")
