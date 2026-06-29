# Objectives

This module contains various Julia functions useful for many projects.

# Contents

read_case.jl
  : provides a function to read cases and return the reference dictionary with all the needed network model components computed by powermodels-annex. It is a common/general file.

clear_screen.jl
  : provides both `clear()` and `@clear` which wipe a terminal for convenience.

rampingCSVimplementationAC/DC.jl
  : 
  - primarily these provide `parse_power_system_csv` and `parse_ac_power_system_csv` which read any of our generated demand/ramp limit files and return the dictionary ramping_data, as well as a vector 'demands'. Which both are indexed by individual bus/gen IDs.
  - secondarily provides `generate_power_system_csv`, and `generate_power_system_csv_AC` which use the other helper functions in the files to make random variations on real demand curves over a given number of hours (usually 24), as well as ramp limits based on a random amount from 9-10% of the max power output of any given generator. These return the relative file path for the generated data based on a provided output directory.
  
  It should be noted that DC also has a function `generate_daily_demand_csv` which works similarly to generate_power_system_csv but the demands will adhere more strictly to a realistic preset demand profile. Also consider that all of these functions make use of rand.seed() in them, sometimes at multiple steps. so if desired, one can modify this seed to change only the demands or ramp limits and costs separately. If not modified, the functions will reproduce the same csv repeatedly.

aggregate_demand_data.jl
  :
  - provides functions that read from public demand data files (PUB_Demand_'year'.csv)
  - `parse_csv_data(file_path)`: Gathers the whole year's data and returns a vector of average demand multipliers.
  - `get_date_percentages(file_path, date)`: Same as above, but instead of averaging a year it only gets one day of multipliers

# ref[] dictionary

The following keys are defined for the reference dictionary after reading a case.

ref[:bus_arcs][i]
  : _i_ is the ID of a bus. The _:bus_arcs_ symbol returns a vector of arc tuples that are incident to bus _i_. Every arc tuple consists of three integers: $(l, i, y)$. Item _l_ is an arc ID, _i_ is the source bus (aka from bus), and _y_ is the destination bus. In the ref[:arcs] dictionary, every arc appears in both directions, but the arcs IDs are the same. For example, ref[:arcs] contains both $(l, i, y)$ and $(l, y, i)$. When OPF is calculated, the $p$ and $q$ variables for an arc $(l, i, y)$ contains the power flow along the arc that "leaves" bus _i_. If the $p$ value is negative, it means the power flow is coming `into` bus _i_. 