# =====================================================
# check_case39.jl
# Get deterministic AC-OPF solution for IEEE 39-bus
# =====================================================

ROOT_DIR = "/Users/shidratulmuntaha/power-annex/"
CASE_DIR = ROOT_DIR * ".github/workflows/cases/"
CASE_FILE = "case39.m"

# Load required functions
include(ROOT_DIR * "src/basemodels-org/B-uncertainty_ac_opf.jl")
include(ROOT_DIR * "src/util-org/read_case.jl")

# Read Case 39
ref = read_case(CASE_DIR * CASE_FILE)

# Create AC-OPF model
model = init_ac()

# Solve deterministic AC-OPF
lambda = 1.0
result = solve_model_ac!(ref, model, lambda)

println("\n==========================================")
println("        IEEE 39-BUS AC-OPF")
println("==========================================")

println("\nStatus = ", result[:status])
println("Cost   = ", result[:cost])

# -----------------------------------------------------
# Bus 39
# -----------------------------------------------------

bus39 = 39

println("\n------------------------------------------")
println("BUS 39")
println("------------------------------------------")

# Demand from case data
pd39 = 11.04
qd39 = 250.0

# Generator at bus 39
pg39 = result[:pg][10]

println("Pd(39) = ", pd39, " MW")


println("Pg(39) = ", pg39, " MW")


# -----------------------------------------------------
# Power balance at Bus 39
# -----------------------------------------------------

net_generation = pg39 - pd39

println("\n------------------------------------------")
println("POWER BALANCE AT BUS 39")
println("------------------------------------------")

println("Pg - Pd = ", net_generation, " MW")

# -----------------------------------------------------
# Branches connected to Bus 39
# -----------------------------------------------------

# -----------------------------------------------------
# Branch flows connected to Bus 39
# -----------------------------------------------------

println("\n------------------------------------------")
println("BRANCH FLOWS CONNECTED TO BUS 39")
println("------------------------------------------")

for (key, value) in result[:p_arcs]

    println("p_arcs key = ", key, "   value = ", value)

end