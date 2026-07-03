############################################################
# B-OPF-BASICS
#
# Per-Bus approximate Maximum Loadability (λ_max) Analysis
#
# This program computes the approximate maximum feasible loading factor
# (λ_max) for each load bus in a MATPOWER test system using
# AC Optimal Power Flow (AC-OPF).
#
# Method:
#   - Increase the active and reactive load at one bus
#     while keeping all other buses unchanged.
#   - Solve the AC-OPF using Ipopt.
#   - Continue increasing the load until the first
#     infeasible solution is encountered.
#   - Record the last feasible loading factor (λ_max).
#
# Compatible with the newer PowerModels data structure
# where loads are stored in data["load"].
############################################################

using Printf
using PowerModels
using Ipopt
import MathOptInterface as MOI

############################################################
# Load IEEE 14-bus case
############################################################

case_file = "/Users/shidratulmuntaha/B-OPF-BASICS/case14.m"

data = PowerModels.parse_file(case_file)

############################################################
# Parameters
############################################################

step = 0.1

############################################################
# Create bus -> loads mapping
############################################################

bus_to_loads = Dict{Int, Vector{String}}()

for (load_id, load) in data["load"]

    bus = load["load_bus"]

    if !haskey(bus_to_loads, bus)
        bus_to_loads[bus] = String[]
    end

    push!(bus_to_loads[bus], load_id)

end

############################################################
# Results
############################################################

results = Vector{Tuple{Int, Float64}}()

############################################################
# Loop over every load bus
############################################################

for bus_id in sort(collect(keys(bus_to_loads)))

    println("\n===================================")
    println("Bus $bus_id")

    load_ids = bus_to_loads[bus_id]

    ########################################################
    # Save original loads
    ########################################################

    base_pd = Dict{String, Float64}()
    base_qd = Dict{String, Float64}()

    for id in load_ids
        base_pd[id] = data["load"][id]["pd"]
        base_qd[id] = data["load"][id]["qd"]
    end

    ########################################################
    # Increase load
    ########################################################

    λ = 1.0
    lambda_max = 0.0

    while true

        ####################################################
        # Scale every load connected to this bus
        ####################################################

        for id in load_ids
            data["load"][id]["pd"] = λ * base_pd[id]
            data["load"][id]["qd"] = λ * base_qd[id]
        end

        result = PowerModels.solve_ac_opf(data, Ipopt.Optimizer)

        if result["termination_status"] == LOCALLY_SOLVED

            lambda_max = λ
            λ += step

        else
            break
        end

    end

    ########################################################
    # Restore original loads
    ########################################################

    for id in load_ids
        data["load"][id]["pd"] = base_pd[id]
        data["load"][id]["qd"] = base_qd[id]
    end

    push!(results, (bus_id, lambda_max))

end

############################################################
# Sort buses
############################################################

sort!(results, by = x -> x[2], rev = true)

println("\n=========================================")
println("Bus\tLambda_max")
println("=========================================")

for (bus, λ) in results
    @printf("%-6d\t%.1f\n", bus, λ)
end