
module SearchModel
    using CSV, DataFrames, Random
    using PowerModels, JuMP, Dates, Serialization, PlotlyJS, Ipopt, Graphs
    using Distributions, Statistics
    using LinearAlgebra
    #=
    Lots of functions are exported here, but of course not all of them are useful outside of the larger functions that call them.
    I included them as it can be helpful to read a list of available functions when trying to experiment or when looking to tackle
    adding to/modifying existing functions. 
    The readme (if finished...) should contain an explanation and list of all the useful functions.
    
    If curious, all AC functions have been commented now, so you can hover over any function to get a thorough description.
    =#

    # Export of aggregate_demand_data.jl
    export get_date_percentages, percentages_of_max_demand, parse_csv_data, get_hourly_average
    # Export of rampingCSVimplementation_DC.jl
    export safe_parse_float, parse_power_system_csv, generate_power_system_csv, generate_daily_demand_csv, generate_daily_demand_profile

    # Export of rampingCSVimplementation_AC.jl
    export parse_ac_power_system_csv, calculate_power_factor, vector_magnitude, vector_angle, vector_to_power, perturb_power_vector, generate_ac_vector_demand_profile, generate_ac_vector_demand_csv, generate_power_system_csv_AC, 
            plot_demand_curve, plot_bus_power_scatter, plot_bus_pq_vectors
    
    # Export of graph_search_AC.jl
    export AC_graph_search, test_scenarios_AC, build_new_graph_AC, calculate_path_cost_AC, get_generation_and_ramping_costs_AC, output_run_data_to_csv_AC,
    generate_new_scenarios_subset_AC, delta_AC, find_largest_time_period_AC, build_and_optimize_largest_period_AC, test_feasibility_AC, shortest_path_AC, 
    build_initial_graph_AC, add_weighted_edges_AC!, calculate_path_cost_AC, extract_solution_AC, graph_demands_and_generation_AC, output_run_data_to_csv_AC
    
    # Export of graph_search_DC.jl
    export DC_graph_search, shortest_path, test_feasibility, calculate_path_cost, test_scenarios, find_largest_time_period, build_and_optimize_largest_period,
    generate_new_scenarios_subset, delta, extract_power_flow_data, build_initial_graph, add_weighted_edges!, extract_solution, build_new_graph,
    get_generation_and_ramping_costs, graph_demands_and_generation, output_run_data_to_csv

    # Export of this file
    export create_search_model, optimize_model, DCMPOPFSearchFactory, ACMPOPFSearchFactory

    ###########################################################################
    # Model Factories
    ###########################################################################
    """
    AbstractMPOPFModelFactory

    An abstract type serving as a base for all MPOPF model factories, which contains both the file path and optimizer. 
    The type used tells create_search_model which method to use to make an AC/DC model.
    """
    abstract type AbstractMPOPFModelFactory end

    mutable struct DCMPOPFSearchFactory <: AbstractMPOPFModelFactory
        file_path::String
        optimizer::Type

        function DCMPOPFSearchFactory(file_path::String, optimizer::Type=Ipopt.Optimizer)
            return new(file_path, optimizer)
        end
    end

    mutable struct ACMPOPFSearchFactory <: AbstractMPOPFModelFactory
        file_path::String
        optimizer::Type

        function ACMPOPFSearchFactory(file_path::String, optimizer::Type=Ipopt.Optimizer)
            return new(file_path, optimizer)
        end
    end

    ###########################################################################
    # Model Structs
    ###########################################################################
    """
        AbstractMPOPFModel
    An abstract type serving as a base for all MPOPF model types.
    """
    abstract type AbstractMPOPFModel end

    mutable struct DCMPOPFSearchModel <: AbstractMPOPFModel
        model::JuMP.Model
        data::Dict
        time_periods::Int64
        ramping_data::Dict
        demands::Vector{Vector{Float64}}

        function DCMPOPFSearchModel(model::JuMP.Model, data::Dict, time_periods::Int64, ramping_data::Dict, demands::Vector{Vector{Float64}})
            return new(model, data, time_periods, ramping_data, demands)
        end
    end

    mutable struct ACMPOPFSearchModel <: AbstractMPOPFModel
        model::JuMP.Model
        data::Dict
        time_periods::Int64
        ramping_data::Dict
        active_demands::Vector{Dict{Int64, Float64}}
        reactive_demands::Vector{Dict{Int64, Float64}}

        function ACMPOPFSearchModel(model::JuMP.Model, data::Dict, time_periods::Int64, ramping_data::Dict, active_demands::Vector{Dict{Int64, Float64}}, reactive_demands::Vector{Dict{Int64, Float64}})
            return new(model, data, time_periods, ramping_data, active_demands, reactive_demands)
        end
    end

    ###########################################################################
    # Creation methods
    ###########################################################################
    function create_search_model(factory::DCMPOPFSearchFactory, time_periods::Int64, ramping_data::Dict, demands::Vector{Vector{Float64}})::DCMPOPFSearchModel
        data = PowerModels.parse_file(factory.file_path)
        PowerModels.standardize_cost_terms!(data, order=2)
        PowerModels.calc_thermal_limits!(data)

        model = JuMP.Model(factory.optimizer)

        power_flow_model = DCMPOPFSearchModel(model, data, time_periods, ramping_data, demands)

        set_model_variables!(power_flow_model)
        set_model_objective_function!(power_flow_model)
        set_model_constraints!(power_flow_model)

        return power_flow_model
    end

    function create_search_model(factory::ACMPOPFSearchFactory, time_periods::Int64, ramping_data::Dict,
                active_demands::Vector{Dict{Int64, Float64}}, reactive_demands::Vector{Dict{Int64, Float64}})::ACMPOPFSearchModel
        data = PowerModels.parse_file(factory.file_path)
        PowerModels.standardize_cost_terms!(data, order=2)
        PowerModels.calc_thermal_limits!(data)

        model = JuMP.Model(factory.optimizer)

        power_flow_model = ACMPOPFSearchModel(model, data, time_periods, ramping_data, active_demands, reactive_demands)

        set_model_variables!(power_flow_model)
        set_model_objective_function!(power_flow_model)
        set_model_constraints!(power_flow_model)

        return power_flow_model
    end

    ###########################################################################
    # Include Section
    ###########################################################################

    # Functions which set JuMP Constraints, Objective functions and variables
    # Only needs modifing if changing relaxation space or implementations
    include("model-creation-helpers/implementation-search_ac.jl")
    include("model-creation-helpers/implementation-search_dc.jl")
    
    # CSV file and demand multiplier vector generation functions
    # These make data that can be fed into model creation parameters
    include("../util-org/demand_data_generation/rampingCSVimplementation_AC.jl")
    include("../util-org/demand_data_generation/rampingCSVimplementation_DC.jl")
    include("../util-org/demand_data_generation/aggregate_demand_data.jl")

    # Defines all functionality needed to use graph-search
    include("./graph_search_implementation/graph_search_AC.jl")
    include("./graph_search_implementation/graph_search_DC.jl")

    ###########################################################################
    # Generic optimization function
    ###########################################################################
    """
        optimize_model(model::AbstractMPOPFModel)
    Optimize the given MPOPF model and print the optimal cost.
    
    !!! note
        If the model being optimized is of type `MPOPFModelUncertainty`
        then extra computation will be executed to determine if there are
        any unbalaced power at each bus for every scenario
        when consitering mu_plus and mu_minus.
        That is if `sum_of_mu_plus` and `mu_minus` are both > 0.01
        at any given bus then an error message is printed.
    # Arguments
    - `model::AbstractMPOPFModel`: The MPOPF model to optimize.
    """
    function optimize_model(model::AbstractMPOPFModel)
        optimize!(model.model)
        optimal_cost = objective_value(model.model)
        println("Optimal Cost: ", optimal_cost)
        println()

        # Below is a bunch of legacy code from the other repository 'Power'.
        # This module has not defined the uncertainty model so it will throw an error.
        # I'm leaving it here in case it is decided we want to port that model over as well in this module.

        # if isa(model, MPOPFModelUncertainty)
        #     data = model.data
        #     T = model.time_periods
        #     S = length(model.scenarios)
        #     ref = PowerModels.build_ref(data)[:it][:pm][:nw][0]
        #     error_check = 0

        #     for t in 1:T    
        #         for s in 1:S
        #             for b in keys(ref[:bus])
        #                 sum_of_mu_plus = 0
        #                 mu_minus = 0
        #                 sum_of_mu_plus = sum(value(model.model[:mu_plus][t, g, s]) for g in ref[:bus_gens][b]; init = 0)
        #                 mu_minus = value(model.model[:mu_minus][t, b, s])
        #                 if sum_of_mu_plus >= 0.01 && mu_minus >= 0.01
        #                     error_check = error_check + 1
        #                     println("###############")
        #                     println("#### Error ####")
        #                     println("###############")

        #                     println("Scenario: $s Bus: $b")
        #                     println("mu_plus: $sum_of_mu_plus")
        #                     println("mu_minus: $mu_minus")
        #                     println()
        #                 end
        #             end
        #         end
        #     end
        #     if error_check == 0
        #         println("No mu_plus and mu_minus errors found")
        #         println()
        #     else
        #         println("Found $error_check error(s)")
        #         println()
        #     end
        # end
    end
end