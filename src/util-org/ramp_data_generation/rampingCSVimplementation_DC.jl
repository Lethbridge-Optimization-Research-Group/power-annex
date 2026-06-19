using CSV, DataFrames, Random

function safe_parse_float(s::AbstractString)
    # Return the cell as a float, if none, input mising
    try
        return parse(Float64, s)
    catch
        return missing
    end
end

# Do not leave cells blank, insert 0 for busses that don't demand power
function parse_power_system_csv(file_path::String, matpower_file_path::String)
    # Get CSV content, compare CSV case name with matpoewr case name
    csv_content = read(file_path, String)
    lines = split(csv_content, '\n')
    csv_case_name = strip(lines[1])

    mat_power_case_name = basename(matpower_file_path)
    mat_power_case_name = replace(mat_power_case_name, ".m" => "")

    if csv_case_name != mat_power_case_name
        error("CSV case name ($csv_case_name) does not match the loaded MATPOWER case ($mat_power_case_name)")
    end
    
    # Read the entire CSV file into a DataFrame
    df = CSV.read(IOBuffer(join(lines[2:end], '\n')), DataFrame, header=1, skipto=2)
    # Initialize the output structures
    ramping_data = Dict{String, Vector{Float64}}()
    demands = Vector{Vector{Float64}}()

    # Find the row where bus data starts
    bus_data_start = findfirst(x -> x == "#bus_data", df[!, 1])

    # Process generator data
    gen_data = df[1:bus_data_start-1, :]
    ramping_data["gen_id"] = [safe_parse_float(x) for x in gen_data[!, 1] if x != "gen_id"]
    ramping_data["ramp_limits"] = [safe_parse_float(x) for x in gen_data[!, 2] if x != "ramp_limits"]
    ramping_data["costs"] = [safe_parse_float(x) for x in gen_data[!, 3] if x != "costs"]

    # Process bus data
    bus_data = df[bus_data_start+1:end, 2:end]
    for col in names(bus_data)
        push!(demands, filter(!ismissing, [safe_parse_float(x) for x in bus_data[!, col]]))
    end

    return ramping_data, demands
end

function generate_power_system_csv(data::Dict, output_dir::String, num_periods::Int=24)
    
    Random.seed!(42)
    
    # Extract case name
    case_name = basename(data["name"])
    case_name = replace(case_name, ".m" => "")

    # Create a filename
    output_file = joinpath(output_dir, "$(case_name)_rampingData.csv")

    # Extract generator data
    gen_data = []
    for (_, gen) in data["gen"]
        # Calculate ramping limit as percentage of generator output
        pmax = get(gen, "pmax", 0.0)
        ramp_percent = rand(50:100)  # Random percentage between 5% and 50%
        ramp_limit = pmax * (ramp_percent / 100)
        
        # Generate random ramping cost
        ramp_cost = rand(1000000:3000000) ### RAMP COST ### 

        push!(gen_data, (
            gen["index"],
            ramp_limit,
            ramp_cost
        ))
    end
    sort!(gen_data, by = x -> x[1])

    # Extract initial bus demands
    demand_dict = Dict{Int, Float64}()
    for (_, load) in data["load"]
        bus_id = load["load_bus"]
        pd = get(load, "pd", 0.0)
        demand_dict[bus_id] = get(demand_dict, bus_id, 0.0) + pd
        total_initial_demand += pd
    end
    println(demand_dict)
    sleep(10)
    # Safety margin (95% of total capacity)
    max_allowable_demand = total_generation_capacity * 0.95

    # If initial demand exceeds capacity, scale it down
    if total_initial_demand > max_allowable_demand
        scaling_factor = max_allowable_demand / total_initial_demand
        for bus_id in keys(demand_dict)
            demand_dict[bus_id] *= scaling_factor
        end
    end

    # Create initial demands only for actual buses
    initial_demand = [get(demand_dict, bus_id, 0.0) for bus_id in bus_ids]

    # Generate random variations for additional time periods
    Random.seed!(42)  # Input a seed if you like for reproducibility
    demands = [initial_demand]
    for _ in 2:num_periods
        variation = rand(length(bus_ids)) * 0.2 .- 0.1  ### RAMP VARIATION ### 
        new_demand = initial_demand .* (1 .+ variation)
        push!(demands, new_demand)  # Maybe ensure non-negative demands
    end

    # Create the CSV content
    csv_content = IOBuffer()
    println(csv_content, case_name)
    println(csv_content, "#gen_data")
    println(csv_content, "gen_id,ramp_limits,costs")
    for (index, ramp, cost) in gen_data
        println(csv_content, "$index,$ramp,$cost")
    end
    println(csv_content, "#bus_data")
    print(csv_content, "bus_id")
    for i in 1:num_periods
        print(csv_content, ",T$i")
    end
    println(csv_content)
    for bus in 1:num_buses
        print(csv_content, bus)
        for period in 1:num_periods
            print(csv_content, ",", round(demands[period][bus], digits=3))
        end
        println(csv_content)
    end

    # Write to file
    open(output_file, "w") do f
        write(f, String(take!(csv_content)))
    end

    println("CSV file generated successfully: $output_file")
    println("Total generation capacity: ", round(total_generation_capacity, digits=2))
    println("Maximum allowable demand: ", round(max_allowable_demand, digits=2))
    return output_file
end

function generate_daily_demand_profile(base_demand::Float64, hour::Int)
    """
    Generate realistic demand multiplier based on hour of day (1-24)
    Hour 1 = midnight, Hour 24 = 11 PM
    
    Typical daily pattern:
    - Low demand: midnight to 6 AM (0.6-0.7x base)
    - Morning ramp: 6-9 AM (0.7-0.9x base)
    - Midday: 9 AM-2 PM (0.8-0.9x base)
    - Afternoon/Evening peak: 2-8 PM (0.9-1.0x base)
    - Evening decline: 8 PM-midnight (1.0-0.6x base)
    """
    
    Random.seed!(42)

    hourly_demand_multipliers = [
        0.7774898823226734,  # Hour 1 (Midnight-1 AM)
        0.7577746367160817,  # Hour 2
        0.7475629591967881,  # Hour 3
        0.7467429624859722,  # Hour 4
        0.7601066115854397,  # Hour 5
        0.7957550749427186,  # Hour 6
        0.848195634490196,   # Hour 7
        0.8882239333758674,  # Hour 8
        0.9048222421762029,  # Hour 9
        0.914126440595437,   # Hour 10
        0.9204892926317222,  # Hour 11
        0.9255591999962104,  # Hour 12
        0.9287639638251047,  # Hour 13
        0.9297138190082938,  # Hour 14
        0.9362318261053391,  # Hour 15
        0.9566494709496483,  # Hour 16
        0.9857044729639849,  # Hour 17
        0.9995858620897072,  # Hour 18
        0.997902383811302,   # Hour 19
        0.986088604927653,   # Hour 20
        0.9620366549171514,  # Hour 21
        0.9152052978222504,  # Hour 22
        0.8590014459494637,  # Hour 23
        0.8105390064925645   # Hour 24 (11 PM-Midnight)
    ]

    base_multiplier = hourly_demand_multipliers[hour]

    # Add some random variation (±5%)
    noise = (rand() - 0.5) * 0.1
    multiplier = base_multiplier + noise
    
    # Ensure multiplier stays within reasonable bounds
    return max(0.5, min(1.1, multiplier))
end

function generate_daily_demand_csv(data::Dict, output_dir::String, num_periods::Int=24)
    # Extract case name
    case_name = basename(data["name"])
    case_name = replace(case_name, ".m" => "")

    # Create a filename
    output_file = joinpath(output_dir, "$(case_name)_rampingData.csv")

    # Calculate total generation capacity
    total_generation_capacity = 0.0
    gen_data = []
    for (_, gen) in data["gen"]
        pmax = get(gen, "pmax", 0.0)
        total_generation_capacity += pmax

        # Calculate ramping limit as percentage of generator output
        ramp_percent = rand(90:100)  # Random percentage between 90% and 100%
        ramp_limit = pmax * (ramp_percent / 100)

        # Generate random ramping cost
        ramp_cost = rand(100:300) ### RAMP COST ### 

        push!(gen_data, (
            gen["index"],
            round(ramp_limit, digits=2),
            round(ramp_cost, digits=2)
        ))
    end
    sort!(gen_data, by=x -> x[1])

    # Extract bus demands and create a mapping of actual bus IDs
    demand_dict = Dict{Int,Float64}()
    bus_ids = Int[]  # Store actual bus IDs in order

    # First, collect all bus IDs from the bus data
    for (_, bus) in data["bus"]
        push!(bus_ids, bus["bus_i"])
    end
    sort!(bus_ids)  # Ensure buses are in order

    # Then collect the demands
    total_initial_demand = 0.0
    for (_, load) in data["load"]
        bus_id = load["load_bus"]
        pd = get(load, "pd", 0.0)
        demand_dict[bus_id] = get(demand_dict, bus_id, 0.0) + pd
        total_initial_demand += pd
    end

    # Safety margin (95% of total capacity)
    max_allowable_demand = total_generation_capacity * 0.95

    # If initial demand exceeds capacity, scale it down
    if total_initial_demand > max_allowable_demand
        scaling_factor = max_allowable_demand / total_initial_demand
        for bus_id in keys(demand_dict)
            demand_dict[bus_id] *= scaling_factor
        end
        total_initial_demand = sum(values(demand_dict))
    end

    # Create base demands for actual buses (this becomes our reference demand)
    base_demand_per_bus = [get(demand_dict, bus_id, 0.0) for bus_id in bus_ids]

    # Generate realistic daily demand pattern for each time period
    Random.seed!(42)  # For reproducibility - change or remove for random patterns
    demands = []

    for hour in 1:num_periods
        # Generate demand multiplier for this hour
        hourly_demands = Float64[]
        
        for bus_idx in 1:length(bus_ids)
            base_bus_demand = base_demand_per_bus[bus_idx]
            
            # Apply daily profile multiplier
            multiplier = generate_daily_demand_profile(base_bus_demand, hour)
            new_demand = base_bus_demand * multiplier
            
            # Ensure non-negative
            new_demand = max(new_demand, 0.0)
            push!(hourly_demands, new_demand)
        end
        
        # Check if total demand exceeds capacity and scale if necessary
        total_hourly_demand = sum(hourly_demands)
        if total_hourly_demand > max_allowable_demand
            scaling_factor = max_allowable_demand / total_hourly_demand
            hourly_demands .*= scaling_factor
        end
        
        push!(demands, hourly_demands)
    end

    # Create the CSV content
    csv_content = IOBuffer()
    println(csv_content, case_name)
    println(csv_content, "#gen_data")
    println(csv_content, "gen_id,ramp_limits,costs")
    for (index, ramp, cost) in gen_data
        println(csv_content, "$index,$ramp,$cost")
    end
    println(csv_content, "#bus_data")
    print(csv_content, "bus_id")
    for i in 1:num_periods
        print(csv_content, ",T$i")
    end
    println(csv_content)

    # Only output data for buses that exist in the system
    for (idx, bus_id) in enumerate(bus_ids)
        print(csv_content, bus_id)
        for period in 1:num_periods
            print(csv_content, ",", demands[period][idx])
        end
        println(csv_content)
    end

    # Write to file
    open(output_file, "w") do f
        write(f, String(take!(csv_content)))
    end

    # Calculate and display statistics
    total_demands = [sum(period_demands) for period_demands in demands]
    min_demand = minimum(total_demands)
    max_demand = maximum(total_demands)
    peak_hour = argmax(total_demands)
    min_hour = argmin(total_demands)
    
    println("CSV file generated successfully: $output_file")
    println("Total generation capacity: ", round(total_generation_capacity, digits=2))
    println("Maximum allowable demand: ", round(max_allowable_demand, digits=2))
    println("Daily demand statistics:")
    println("  Peak demand: ", round(max_demand, digits=2), " at hour ", peak_hour)
    println("  Minimum demand: ", round(min_demand, digits=2), " at hour ", min_hour)
    println("  Peak/Min ratio: ", round(max_demand/min_demand, digits=2))
    
    return output_file
end