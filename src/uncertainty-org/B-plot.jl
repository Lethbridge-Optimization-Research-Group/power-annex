using Plots
using Printf

valid = isfinite.(costs)
any(valid) || error("No successful costs to plot.")

x = collect(sigma_percentages)[valid]
y = costs[valid]

# Print results before creating the graph.
for (sigma, cost) in zip(x, y)
    @printf("Sigma: %.0f%% | Cost: %.10f\n", sigma, cost)
end

println("Creating graph...")

p = plot(
    x, y;
    marker = :circle,
    linewidth = 2,
    ylabel = "Operating cost (case cost units)",
    xlabel = "Generation sigma (%)",
    title = "Cost versus generation uncertainty",
    label = "Model cost",
    size = (1000, 600),
    left_margin = 15Plots.mm,
    bottom_margin = 10Plots.mm
)

println("Saving graph...")
savefig(p, joinpath(@__DIR__, "sigma_cost.png"))
println("Graph saved.")

display(p)