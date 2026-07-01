#uncertainty driver
#= configuration variables =#
CASE_DIR="../../data/"
CASE_FILE="case1354pegase.m"
ROOT_DIR="/home/rbenkocz/proj/power-flow/src/power-annex/"

#= ***********
include first the files with the needed functions
**************
=# 
include(ROOT_DIR * "src/basemodels-org/ac_opf.jl")
include(ROOT_DIR*"src/util-org/read_case.jl")

#= build the ref dictionary with the case data
=#
ref = read_case(CASE_DIR * CASE_FILE)

#= create the model for Ipopt
=#
model = init_ac()

result = solve_model_ac!(ref, model)

# Check that the solver terminated without an error
println("The solver termination status is $(result[:status])")

# Check the value of the objective function
println("The cost of generation is $(result[:cost]).")

# output execution time (sec)
println("Execution time for optimization: $(result[:time_sec]) sec")

println("Power flow on lines: $(result[:p_arcs])")
