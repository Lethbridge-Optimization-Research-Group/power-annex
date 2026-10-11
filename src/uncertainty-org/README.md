# Objectives

This folder contains Julia code based on the PowerModels Annex GitHub
repository to explore and test ideas that deal with uncertainty for OPF.

# Contents

- `main_epsilon.jl`: Explores the ability of a grid network to supply
  power to a bus from nearby buses and generators. It uses the AC-OPF
  model from the basemodels-org folder.

- `test1.jl`: Runs the uncertainty experiment. Reads user inputs,
  solves the baseline AC-OPF, constructs uncertainty dictionaries,
  and calls the proposed AC-CC solver.

# Uncertainty Dictionary

Author: Shidratul Muntaha

The `uncertainty` dictionary stores information about selected uncertain
loads and generators. Only active power is modeled as uncertain.
Reactive demand is deterministic, and reactive generation is chosen
by the optimizer through the AC equations.

## Structure

    uncertainty[:load][load_id]
    uncertainty[:gen][gen_id]

The IDs are component IDs from the PowerModels reference dictionary,
not bus IDs. Components not selected for uncertainty use the ordinary
deterministic formulation.

## Load Fields

| Field | Meaning |
|---|---|
| `"load_bus"` | Bus containing the load |
| `"exp_pd"` | Mean active demand, taken from the case |
| `"sigma_pd"` | Standard deviation of active demand |

## Generator Fields

| Field | Meaning |
|---|---|
| `"gen_bus"` | Bus containing the generator |
| `"exp_pg"` | Fixed active-generation mean, taken from the baseline AC-OPF solution |
| `"sigma_pg"` | Standard deviation of active generation |

All power values and standard deviations are in per unit (p.u.).
The `"exp_"` prefix means expected value.

If retained, `"exp_qd"` and `"exp_qg"` store case reactive demand and
baseline reactive generation. They are unused by the active-power
uncertainty calculations.

## Example

    uncertainty = Dict(
        :load => Dict(
            1 => Dict(
                "load_bus" => 1,
                "exp_pd" => 0.51,
                "sigma_pd" => 0.05
            )
        ),
        :gen => Dict(
            1 => Dict(
                "gen_bus" => 1,
                "exp_pg" => 0.265,
                "sigma_pg" => 0.02
            )
        )
    )

These numbers illustrate the structure; they are not prescribed inputs.

## Related Dictionaries

- `uncertain_buses[bus_id]`: Lists selected load and generator IDs
  at each affected bus under `:load` and `:gen`.

- `bus_uncertainty[bus_id]`: Stores `"gen_ids"`, total selected
  mean demand `"mu_pd"`, and combined standard deviation `"sigma_p"`.
  Errors are assumed independent, so combined sigma is the square
  root of the sum of squared component sigmas.

- `bus_bounds[bus_id]`: Stores active net-injection quantiles
  `"a1"` and `"a2"` from the proposed bus-level truncated-Normal model.
  Its underlying Normal uses baseline generation minus mean demand;
  the user supplies lower and upper truncation limits.

- `bus_epsilon[bus_id]`: Stores power tolerances `"p_minus"` and
  `"p_plus"` in p.u. For the current experiment, both are fixed at 1.0.

## Probabilities and Results

`alpha` is the total violation probability. Confidence is `1 - alpha`.
The lower and upper tail probabilities sum to `alpha`.
These probabilities are separate from the power tolerances.

The solver returns a `result` dictionary:

- `result[:status]`: Solver termination status.
- `result[:objective]`: Objective value after a successful solve;
  otherwise `nothing`.

This documents the current proposed formulation. Refactoring the code
does not establish its physical validity or chance-constraint guarantees.