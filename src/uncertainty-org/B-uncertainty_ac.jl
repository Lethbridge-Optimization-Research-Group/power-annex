# =====================================================
# B-uncertainty_ac.jl
#
# Driver file for uncertainty experiments on AC OPF.
# Based on main_epsilon_ac.jl
#
# Author: Shidratul Muntaha
# =====================================================


#= configuration variables =#
CASE_DIR = "/Users/shidratulmuntaha/power-annex/.github/workflows/"
CASE_FILE = "case14.m"
ROOT_DIR = "/Users/shidratulmuntaha/power-annex/"

#= ***********
include first the files with the needed functions
**************
=#
include(ROOT_DIR * "src/basemodels-org/B-uncertainty_ac_opf.jl")
include(ROOT_DIR * "src/util-org/read_case.jl")
include(ROOT_DIR * "src/uncertainty-org/B-read_uncertainty.jl")

#= build the ref dictionary with the case data
=#
# ==========================================
# Read the network case
# ==========================================
ref = read_case(CASE_DIR * CASE_FILE)

# ==========================================
# Read uncertainty data
# ==========================================
UNCERTAINTY_DIR = ROOT_DIR * "B-uncertainty_data/"

# Demand uncertainty
demand = read_uncertainty(
    UNCERTAINTY_DIR * "B-case14.csv"
)

# Renewable uncertainty
renewable = read_renewable_uncertainty(
    UNCERTAINTY_DIR * "B-case14_renewable.csv"
)

# ==========================================
# Store everything together
# ==========================================
uncertainty = Dict(
    :case_name => CASE_FILE,
    :case => ref,
    :demand => demand,
    :renewable => renewable
)
####### ** Uncertainty scaling factor **
lambda = 3.4  #Maximum feasible load scaling factor for this system-2.1

# #= create the model for Ipopt
# =#
model = init_ac()

result = solve_model_ac!(ref, model,lambda)



##0.05 -> So if the original load is 100 MW, then most of the time the random load will be around:95 MW to 105 MW
##### **random lambda**

# using Random
# lambda = 1 + 0.05 * randn()
# println("Random lambda = ", round(lambda, digits=3))

# #######

# #= create the model for Ipopt
# =#
# model = init_ac()

# result = solve_model_ac!(ref, model,lambda)



# #### ** identify cost ***
# # using Random
# for sim = 1:10

#    # Generate a random lambda
#     lambda = 1 + 0.05 * randn()
   
#     println("\nSimulation ", sim)
#     println("Random lambda = ", round(lambda, digits=3))

#     # Create a new optimization model
#     model = init_ac()

#     # Solve the OPF
#     result = solve_model_ac!(ref, model, lambda)

#     # Print results
#     println("Status = ", result[:status])
#     println("Cost = ", result[:cost])

# end

# ###


# ### **increase the lambda 5% on each time
lambda=1.0
for sim = 1:5

    println("\nSimulation ", sim)
    println("lambda = ", round(lambda, digits=3))

    # Create a new optimization model
    model = init_ac()

    # Solve the OPF
    result = solve_model_ac!(ref, model, lambda)

    # Print results
    println("Status = ", result[:status])
    println("Cost = ", result[:cost])

   global lambda += 0.05
end

# ###


# ## ** generator-power violation **

# println("\nChecking generators...\n")
# for (i, gen) in ref[:gen]

#     pg_value = result[:pg][i]
#     println("Generator ", i,
#             "  Pg = ", round(pg_value, digits=2),
#             "  Pmax = ", gen["pmax"])
# end



# # Check that the solver terminated without an error
# println("The solver termination status is $(result[:status])")

# # Check the value of the objective function
# println("The cost of generation is $(result[:cost]).")

# # output execution time (sec)
# println("Execution time for optimization: $(result[:time_sec]) sec")

# println("Power flow on lines: $(result[:p_arcs])")

println("==========================================")
println("      UNCERTAINTY INFORMATION")
println("==========================================")

println("\nCASE")
println("------------------------------------------")
println(CASE_FILE)

println("\nDEMAND UNCERTAINTY")
println("------------------------------------------")
println("Bus ID\tMean (μ)\tStd Dev (σ)")

for (bus, data) in sort(collect(uncertainty[:demand]), by = x -> x[1])
    println("$(bus)\t$(data.mu)\t\t$(data.sigma)")
end

println("\nRENEWABLE UNCERTAINTY")
println("------------------------------------------")
println("Gen ID\tBus ID\tMean (μ)\tStd Dev (σ)")

for (gen, data) in sort(collect(uncertainty[:renewable]), by = x -> x[1])
    println("$(gen)\t$(data.bus_id)\t$(data.mu)\t\t$(data.sigma)")
end
println("==========================================")