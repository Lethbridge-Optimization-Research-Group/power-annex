lambda=1.0
# for sim = 1:100

#     println("\nSimulation ", sim)
#     println("lambda = ", round(lambda, digits=3))

#     # Create a new optimization model
#     model = init_ac()

#     # Solve the OPF
#     result = solve_model_ac!(ref, model, lambda)

#     # Print results
#     println("Status = ", result[:status])
#     println("Cost = ", result[:cost])

#    global lambda += 0.05
# end