using CSV, DataFrames, Random

function parse_ac_power_system_csv(file_path::String, matpower_file_path::String)
    csv_content = read(file_path, String)
    lines = split(csv_content, '\n')
    csv_case_name = strip(lines[1])

    mat_power_case_name = basename(matpower_file_path)
    mat_power_case_name = replace(mat_power_case_name, ".m" => "")

    if csv_case_name != mat_power_case_name
        error("CSV case name ($csv_case_name) does not match the loaded MATPOWER case ($mat_power_case_name)")
    end

    df = CSV.read(IOBuffer(join(lines[2:end], '\n')), DataFrame, header=1, skipto=2)
    
    ramping_data = Dict{String,Any}()
    active_demands = Vector{Dict{Int,Float64}}()
    reactive_demands = Vector{Dict{Int,Float64}}()

    bus_data_start = findfirst(x -> x == "#bus_data", df[!, 1])

    gen_data = df[1:bus_data_start-1, :]
    ramping_data["gen_id"] = [safe_parse_float(x) for x in gen_data[!, 1] if x != "gen_id"]
    ramping_data["ramp_limits"] = Dict{Int,Float64}()
    ramping_data["costs"] = Dict{Int,Float64}()
    
    num_of_gens = length(ramping_data["gen_id"])

    for i in 1:num_of_gens
        gen_id = Int(ramping_data["gen_id"][i])
        ramping_data["ramp_limits"][gen_id] = safe_parse_float(gen_data[i+1, 2])
        ramping_data["costs"][gen_id] = safe_parse_float(gen_data[i+1, 3])
    end

    bus_data = df[bus_data_start+1:end, :]
    bus_ids = [parse(Int, x) for x in bus_data[!, 1] if x != "bus_id" && !ismissing(x)]
    
    columns = size(bus_data, 2)
    has_reactive = (columns - 1) % 2 == 0
    num_periods = has_reactive ? (columns - 1) ÷ 2 : columns - 1

    for t in 1:num_periods
        active_period_demands = Dict{Int,Float64}()
        reactive_period_demands = Dict{Int,Float64}()
        
        for (idx, bus_id) in enumerate(bus_ids)
            if has_reactive
                active_col = 2 * t
                reactive_col = 2 * t + 1
                
                active_val = safe_parse_float(bus_data[idx+1, active_col])
                reactive_val = safe_parse_float(bus_data[idx+1, reactive_col])
                
                active_period_demands[bus_id] = ismissing(active_val) ? 0.0 : active_val
                reactive_period_demands[bus_id] = ismissing(reactive_val) ? 0.0 : reactive_val
            else
                active_val = safe_parse_float(bus_data[idx+1, t+1])
                active_period_demands[bus_id] = ismissing(active_val) ? 0.0 : active_val
                reactive_period_demands[bus_id] = 0.0
            end
        end
        
        push!(active_demands, active_period_demands)
        push!(reactive_demands, reactive_period_demands)
    end

    return ramping_data, active_demands, reactive_demands
end

function calculate_power_factor(pd::Float64, qd::Float64)
    """Calculate power factor from active and reactive power"""
    if pd == 0.0 && qd == 0.0
        return 1.0
    end
    s = sqrt(pd^2 + qd^2)
    return pd / s
end

function vector_magnitude(pd::Float64, qd::Float64)
    """Calculate magnitude of power vector"""
    return sqrt(pd^2 + qd^2)
end

function vector_angle(pd::Float64, qd::Float64)
    """Calculate angle of power vector in radians (angle from P-axis)"""
    if pd == 0.0 && qd == 0.0
        return atan(tan(acos(0.85)))  # Default to ~0.85 PF angle
    end
    return atan(qd, pd)
end

function vector_to_power(magnitude::Float64, angle::Float64)
    """Convert magnitude and angle to P and Q"""
    pd = magnitude * cos(angle)
    qd = magnitude * sin(angle)
    return pd, qd
end

function perturb_power_vector(pd::Float64, qd::Float64, magnitude_multiplier::Float64, 
                              angle_perturbation_deg::Float64; 
                              min_pf::Float64=0.7, max_pf::Float64=0.98)
    """
    Modify power demand using vector approach:
    - Scale magnitude by multiplier
    - Perturb angle by specified degrees
    - Constrain to realistic power factor range
    """
    
    # Handle zero load case
    if pd == 0.0 && qd == 0.0
        return 0.0, 0.0
    end
    
    # Get current magnitude and angle
    current_magnitude = vector_magnitude(pd, qd)
    current_angle = vector_angle(pd, qd)
    
    # Apply magnitude scaling
    new_magnitude = current_magnitude * magnitude_multiplier
    
    # Apply angle perturbation (convert degrees to radians)
    angle_perturbation_rad = deg2rad(angle_perturbation_deg)
    new_angle = current_angle + angle_perturbation_rad
    
    # Calculate power factor bounds in terms of angles
    # PF = cos(angle), so angle = acos(PF)
    max_angle = acos(min_pf)  # Maximum angle (lowest PF)
    min_angle = acos(max_pf)  # Minimum angle (highest PF)
    
    # Constrain angle to realistic power factor range
    new_angle = clamp(new_angle, min_angle, max_angle)
    
    # Convert back to P and Q
    new_pd, new_qd = vector_to_power(new_magnitude, new_angle)
    
    return max(0.0, new_pd), max(0.0, new_qd)
end

function generate_ac_vector_demand_profile(base_pd::Float64, base_qd::Float64, hour::Int, hourly_demand_multipliers;
                                           peak_magnitude::Float64=1.0, min_magnitude::Float64=0.6,
                                           max_angle_variation::Float64=10.0)
    """
    Generate demand for a specific hour using vector perturbation approach
    
    Parameters:
    - base_pd, base_qd: Base active and reactive power
    - hour: Hour of day (1-24)
    - peak_hour, min_hour: Hours of peak and minimum demand
    - peak_magnitude, min_magnitude: Multipliers for magnitude at peak and minimum
    - max_angle_variation: Maximum angle change in degrees from base
    """
    Random.seed!(42 + hour)
    

    min_hour = minimum(hourly_demand_multipliers)
    peak_hour = maximum(hourly_demand_multipliers)

    
    # Get base multiplier from historical data
    base_multiplier = hourly_demand_multipliers[hour]
    
    # Add random variation (±3%)
    magnitude_noise = (rand() - 0.5) * 0.06
    magnitude_multiplier = base_multiplier + magnitude_noise
    magnitude_multiplier = clamp(magnitude_multiplier, 0.5, 1.1)
    
    # Calculate angle perturbation (more inductive during high load, less during low load)
    # Negative angle = more inductive (lower PF)
    normalized_hour = (hour - min_hour) / (peak_hour - min_hour)
    normalized_hour = clamp(normalized_hour, 0.0, 1.0)
    
    # During peak hours, loads tend to be slightly more inductive
    angle_bias = -max_angle_variation * 0.3 * normalized_hour
    angle_noise = (rand() - 0.5) * max_angle_variation * 0.5
    angle_perturbation = angle_bias + angle_noise
    
    return perturb_power_vector(base_pd, base_qd, magnitude_multiplier, angle_perturbation)
end

function generate_ac_vector_demand_csv(data::Dict, output_dir::String, hourly_demand_multipliers, num_periods::Int=24;
                                       default_power_factor::Float64=0.85,
                                       capacity_safety_margin::Float64=0.95)
    """
    Generate AC multi-period demand using vector perturbation approach
    """
    case_name = basename(data["name"])
    case_name = replace(case_name, ".m" => "")
    output_file = joinpath(output_dir, "$(case_name)_AC_rampingData.csv")

    # Calculate generation capacity and prepare generator data
    total_generation_capacity = 0.0
    gen_data = []
    for (_, gen) in data["gen"]
        pmax = get(gen, "pmax", 0.0)
        total_generation_capacity += pmax

        ramp_percent = rand(90:100)
        ramp_limit = pmax * (ramp_percent / 100)
        ramp_cost = rand(100:300)

        push!(gen_data, (
            gen["index"],
            ramp_limit,
            ramp_cost
        ))
    end
    sort!(gen_data, by=x -> x[1])

    # Extract base demands from loads
    base_demand_vectors = Dict{Int,Tuple{Float64,Float64}}()
    bus_ids = Int[]

    for (_, bus) in data["bus"]
        push!(bus_ids, bus["bus_i"])
    end
    sort!(bus_ids)

    # Initialize all buses with zero demand
    for bus_id in bus_ids
        base_demand_vectors[bus_id] = (0.0, 0.0)
    end

    # Collect demands from load data
    total_base_pd = 0.0
    total_base_qd = 0.0
    
    for (_, load) in data["load"]
        bus_id = load["load_bus"]
        pd = get(load, "pd", 0.0)
        qd = get(load, "qd", 0.0)
        
        current_pd, current_qd = base_demand_vectors[bus_id]
        base_demand_vectors[bus_id] = (current_pd + pd, current_qd + qd)
        
        total_base_pd += pd
        total_base_qd += qd
    end

    # Handle missing reactive power with default power factor
    for bus_id in bus_ids
        pd, qd = base_demand_vectors[bus_id]
        if pd > 0.0 && qd == 0.0
            angle = acos(default_power_factor)
            qd = pd * tan(angle)
            base_demand_vectors[bus_id] = (pd, qd)
            total_base_qd += qd
        end
    end

    # Scale down if exceeds capacity
    max_allowable_active_demand = total_generation_capacity * capacity_safety_margin
    if total_base_pd > max_allowable_active_demand
        scaling_factor = max_allowable_active_demand / total_base_pd
        for bus_id in keys(base_demand_vectors)
            pd, qd = base_demand_vectors[bus_id]
            base_demand_vectors[bus_id] = (pd * scaling_factor, qd * scaling_factor)
        end
        total_base_pd *= scaling_factor
        total_base_qd *= scaling_factor
    end

    # Generate demands for each time period using vector perturbations
    Random.seed!(123)
    active_demands = []
    reactive_demands = []

    for hour in 1:num_periods
        hourly_active_demands = Float64[]
        hourly_reactive_demands = Float64[]
        
        for bus_id in bus_ids
            base_pd, base_qd = base_demand_vectors[bus_id]
            
            # Apply vector perturbation
            new_pd, new_qd = generate_ac_vector_demand_profile(base_pd, base_qd, hour, hourly_demand_multipliers)
            
            push!(hourly_active_demands, new_pd)
            push!(hourly_reactive_demands, new_qd)
        end
        
        # Scale if exceeds capacity
        total_hourly_active = sum(hourly_active_demands)
        if total_hourly_active > max_allowable_active_demand
            scaling_factor = max_allowable_active_demand / total_hourly_active
            hourly_active_demands .*= scaling_factor
            hourly_reactive_demands .*= scaling_factor
        end
        
        push!(active_demands, hourly_active_demands)
        push!(reactive_demands, hourly_reactive_demands)
    end

    # Write CSV file
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
        print(csv_content, ",P_T$i,Q_T$i")
    end
    println(csv_content)

    for (idx, bus_id) in enumerate(bus_ids)
        print(csv_content, bus_id)
        for period in 1:num_periods
            active_val = active_demands[period][idx]
            reactive_val = reactive_demands[period][idx]
            print(csv_content, ",$active_val,$reactive_val")
        end
        println(csv_content)
    end

    open(output_file, "w") do f
        write(f, String(take!(csv_content)))
    end

    # Calculate and display statistics
    total_active_demands = [sum(period_demands) for period_demands in active_demands]
    total_reactive_demands = [sum(period_demands) for period_demands in reactive_demands]
    
    min_active = minimum(total_active_demands)
    max_active = maximum(total_active_demands)
    min_reactive = minimum(total_reactive_demands)
    max_reactive = maximum(total_reactive_demands)
    
    peak_hour = argmax(total_active_demands)
    min_hour = argmin(total_active_demands)
    
    avg_power_factors = []
    for period in 1:num_periods
        pf = calculate_power_factor(total_active_demands[period], total_reactive_demands[period])
        push!(avg_power_factors, pf)
    end
    
    println("AC CSV file generated successfully: $output_file")
    println("Total generation capacity: ", round(total_generation_capacity, digits=2))
    println("Maximum allowable active demand: ", round(max_allowable_active_demand, digits=2))
    println("Daily demand statistics:")
    println("  Peak active demand: ", round(max_active, digits=2), " at hour ", peak_hour)
    println("  Minimum active demand: ", round(min_active, digits=2), " at hour ", min_hour)
    println("  Peak reactive demand: ", round(max_reactive, digits=2))
    println("  Minimum reactive demand: ", round(min_reactive, digits=2))
    println("  Active Peak/Min ratio: ", round(max_active/min_active, digits=2))
    println("  Average power factor range: ", round(minimum(avg_power_factors), digits=3), 
            " to ", round(maximum(avg_power_factors), digits=3))
    
    return output_file
end

# Convenience wrapper
function generate_power_system_csv_AC(data::Dict, output_dir::String, num_periods::Int=24)
    return generate_ac_vector_demand_csv(data, output_dir, num_periods)
end

function plot_demand_curve(csv_file_path::String; plot_title::String="", save_path::String="")
    """
    Plot the total active and reactive demand across all time periods
    
    Parameters:
    - csv_file_path: Path to the generated CSV file
    - plot_title: Optional custom title for the plot
    - save_path: Optional path to save the plot (if empty, just displays)
    """
    
    # Parse the CSV to extract demand data
    csv_content = read(csv_file_path, String)
    lines = split(csv_content, '\n')
    case_name = strip(lines[1])
    
    # Read into DataFrame
    df = CSV.read(IOBuffer(join(lines[2:end], '\n')), DataFrame, header=1, skipto=2)
    
    # Find bus data start
    bus_data_start = findfirst(x -> x == "#bus_data", df[!, 1])
    bus_data = df[bus_data_start+1:end, :]
    
    # Determine number of periods
    columns = size(bus_data, 2)
    num_periods = (columns - 1) ÷ 2  # Assuming P and Q columns
    
    # Calculate total demands for each period
    total_active = Float64[]
    total_reactive = Float64[]
    
    for period in 1:num_periods
        active_col = 2 * period
        reactive_col = 2 * period + 1
        
        period_active = 0.0
        period_reactive = 0.0
        
        for row_idx in 2:size(bus_data, 1)  # Skip header row
            active_val = safe_parse_float(string(bus_data[row_idx, active_col]))
            reactive_val = safe_parse_float(string(bus_data[row_idx, reactive_col]))
            
            period_active += ismissing(active_val) ? 0.0 : active_val
            period_reactive += ismissing(reactive_val) ? 0.0 : reactive_val
        end
        
        push!(total_active, period_active)
        push!(total_reactive, period_reactive)
    end
    
    # Calculate power factor for each period
    power_factors = [calculate_power_factor(total_active[i], total_reactive[i]) 
                     for i in 1:num_periods]
    
    # Create the plot
    hours = 1:num_periods
    
    p1 = plot(hours, total_active, 
              label="Active Power (P)", 
              linewidth=2, 
              marker=:circle,
              color=:blue,
              xlabel="Time Period (Hour)",
              ylabel="Power (p.u.)",
              title=isempty(plot_title) ? "Demand Curve - $case_name" : plot_title,
              legend=:topleft,
              grid=true)
    
    plot!(p1, hours, total_reactive,
          label="Reactive Power (Q)",
          linewidth=2,
          marker=:square,
          color=:red)
    
    # Add power factor on secondary axis
    p2 = plot(hours, power_factors,
              label="Power Factor",
              linewidth=2,
              marker=:diamond,
              color=:green,
              xlabel="Time Period (Hour)",
              ylabel="Power Factor",
              legend=:bottomright,
              grid=true,
              ylim=(0.7, 1.0))
    
    # Combine plots
    final_plot = plot(p1, p2, layout=(2,1), size=(800, 600))
    
    # Save or display
    if !isempty(save_path)
        savefig(final_plot, save_path)
        println("Plot saved to: $save_path")
    else
        display(final_plot)
    end
    
    return final_plot
end

function plot_bus_power_scatter(csv_file_path::String, time_period::Int; 
                                plot_title::String="", save_path::String="")
    """
    Plot Pd vs Qd for all buses at a specific time period (like the scatter plot shown)
    
    Parameters:
    - csv_file_path: Path to the generated CSV file
    - time_period: Which time period to plot (1 to num_periods)
    - plot_title: Optional custom title
    - save_path: Optional path to save the plot
    """

    
    # Parse the CSV
    csv_content = read(csv_file_path, String)
    lines = split(csv_content, '\n')
    case_name = strip(lines[1])
    
    df = CSV.read(IOBuffer(join(lines[2:end], '\n')), DataFrame, header=1, skipto=2)
    
    # Find bus data
    bus_data_start = findfirst(x -> x == "#bus_data", df[!, 1])
    bus_data = df[bus_data_start+1:end, :]
    
    # Get bus IDs
    bus_ids = Float64[]
    pd_values = Float64[]
    qd_values = Float64[]
    
    active_col = 2 * time_period
    reactive_col = 2 * time_period + 1
    
    for row_idx in 2:size(bus_data, 1)
        bus_id_val = safe_parse_float(string(bus_data[row_idx, 1]))
        active_val = safe_parse_float(string(bus_data[row_idx, active_col]))
        reactive_val = safe_parse_float(string(bus_data[row_idx, reactive_col]))
        
        if !ismissing(bus_id_val)
            push!(bus_ids, bus_id_val)
            push!(pd_values, ismissing(active_val) ? 0.0 : active_val)
            push!(qd_values, ismissing(reactive_val) ? 0.0 : reactive_val)
        end
    end
    
    # Create scatter plot
    p = scatter(bus_ids, pd_values,
                label="Pd",
                marker=:circle,
                markersize=4,
                color=:blue,
                xlabel="Bus",
                ylabel="Load (p.u.)",
                title=isempty(plot_title) ? 
                      "$case_name - Time Period $time_period" : plot_title,
                legend=:topright,
                grid=true)
    
    # Optionally add Qd on same plot with different marker
    scatter!(p, bus_ids, qd_values,
             label="Qd",
             marker=:square,
             markersize=3,
             color=:red,
             alpha=0.6)
    
    # Save or display
    if !isempty(save_path)
        savefig(p, save_path)
        println("Plot saved to: $save_path")
    else
        display(p)
    end
    
    return p
end

function plot_bus_pq_vectors(csv_file_path::String, time_period::Int;
                             plot_title::String="", save_path::String="",
                             show_vectors::Bool=true)
    """
    Plot Pd vs Qd as a scatter plot in P-Q space (Qd on y-axis, Pd on x-axis)
    Optionally show vectors from origin to demonstrate the vector approach
    
    Parameters:
    - csv_file_path: Path to the generated CSV file
    - time_period: Which time period to plot (1 to num_periods)
    - plot_title: Optional custom title
    - save_path: Optional path to save the plot
    - show_vectors: If true, draw vectors from origin to each point
    """
    
    # Parse the CSV
    csv_content = read(csv_file_path, String)
    lines = split(csv_content, '\n')
    case_name = strip(lines[1])
    
    df = CSV.read(IOBuffer(join(lines[2:end], '\n')), DataFrame, header=1, skipto=2)
    
    # Find bus data
    bus_data_start = findfirst(x -> x == "#bus_data", df[!, 1])
    bus_data = df[bus_data_start+1:end, :]
    
    # Extract P and Q values
    pd_values = Float64[]
    qd_values = Float64[]
    
    active_col = 2 * time_period
    reactive_col = 2 * time_period + 1
    
    for row_idx in 2:size(bus_data, 1)
        active_val = safe_parse_float(string(bus_data[row_idx, active_col]))
        reactive_val = safe_parse_float(string(bus_data[row_idx, reactive_col]))
        
        pd = ismissing(active_val) ? 0.0 : active_val
        qd = ismissing(reactive_val) ? 0.0 : reactive_val
        
        # Only plot non-zero loads
        if pd > 0.0 || qd > 0.0
            push!(pd_values, pd)
            push!(qd_values, qd)
        end
    end
    
    # Create scatter plot in P-Q space
    p = scatter(pd_values, qd_values,
                label="Bus Loads",
                marker=:circle,
                markersize=5,
                color=:blue,
                xlabel="Active Power Pd (p.u.)",
                ylabel="Reactive Power Qd (p.u.)",
                title=isempty(plot_title) ? 
                      "$case_name - P-Q Space (T=$time_period)" : plot_title,
                legend=:topright,
                grid=true,
                aspect_ratio=:equal)
    
    # Draw vectors if requested
    if show_vectors && length(pd_values) > 0
        # Sample a few vectors to avoid clutter (e.g., every 5th bus or max 20 vectors)
        step = max(1, length(pd_values) ÷ 20)
        for i in 1:step:length(pd_values)
            plot!(p, [0, pd_values[i]], [0, qd_values[i]],
                  arrow=true,
                  color=:gray,
                  alpha=0.3,
                  label="",
                  linewidth=1)
        end
    end
    
    
    
    # Save or display
    if !isempty(save_path)
        savefig(p, save_path)
        println("Plot saved to: $save_path")
    else
        display(p)
    end
    
    return p
end