
module SearchModel

    using CSV, DataFrames, Random
    using PowerModels, JuMP, Dates, Serialization, PlotlyJS, Ipopt, Graphs
    using Distributions, Statistics
    using LinearAlgebra
    # Export of rampingCSVimplementation_DC.jl
    export safe_parse_float, parse_power_system_csv, generate_power_system_csv, generate_daily_demand_csv, generate_daily_demand_profile

    # Export of rampingCSVimplementation_AC.jl
    export parse_ac_power_system_csv, calculate_power_factor, vector_magnitude, vector_angle, vector_to_power, perturb_power_vector, generate_ac_vector_demand_profile, generate_ac_vector_demand_csv, generate_power_system_csv_AC, 
            plot_demand_curve, plot_bus_power_scatter, plot_bus_pq_vectors

    # Export of this file
    export create_search_model, create_search_model_AC, optimize_model, DCMPOPFSearchFactory, ACMPOPFSearchFactory

    ###########################################################################
    # Model Factories
    ###########################################################################
    """
        AbstractMPOPFModelFactory
    An abstract type serving as a base for all MPOPF model factories,
    which contains both the file path and optimizer.
    """
    abstract type AbstractMPOPFModelFactory end

    mutable struct DCMPOPFSearchFactory <: AbstractMPOPFModelFactory
        file_path::String
        optimizer::Type

        function DCMPOPFSearchFactory(file_path::String, optimizer::Type)
            return new(file_path, optimizer)
        end
    end

    mutable struct ACMPOPFSearchFactory <: AbstractMPOPFModelFactory
        file_path::String
        optimizer::Type

        function ACMPOPFSearchFactory(file_path::String, optimizer::Type)
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

    mutable struct MPOPFSearchModel <: AbstractMPOPFModel
        model::JuMP.Model
        data::Dict
        time_periods::Int64
        ramping_data::Dict
        demands::Vector{Vector{Float64}}

        function MPOPFSearchModel(model::JuMP.Model, data::Dict, time_periods::Int64, ramping_data::Dict, demands::Vector{Vector{Float64}})
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

    # Want to take the ramping_data and demands out of the parameter list here. Will find a way to remove them
    function create_search_model(factory::AbstractMPOPFModelFactory, time_periods::Int64, ramping_data::Dict, demands::Vector{Vector{Float64}})::MPOPFSearchModel
        data = PowerModels.parse_file(factory.file_path)
        PowerModels.standardize_cost_terms!(data, order=2)
        PowerModels.calc_thermal_limits!(data)

        model = JuMP.Model(factory.optimizer)

        power_flow_model = MPOPFSearchModel(model, data, time_periods, ramping_data, demands)

        set_model_variables!(power_flow_model, factory)
        set_model_objective_function!(power_flow_model, factory)
        set_model_constraints!(power_flow_model, factory)

        return power_flow_model
    end

    function create_search_model_AC(factory::AbstractMPOPFModelFactory, time_periods::Int64, ramping_data::Dict,
                active_demands::Vector{Dict{Int64, Float64}}, reactive_demands::Vector{Dict{Int64, Float64}})::ACMPOPFSearchModel
        data = PowerModels.parse_file(factory.file_path)
        PowerModels.standardize_cost_terms!(data, order=2)
        PowerModels.calc_thermal_limits!(data)

        model = JuMP.Model(factory.optimizer)

        power_flow_model = ACMPOPFSearchModel(model, data, time_periods, ramping_data, active_demands, reactive_demands)

        set_model_variables!(power_flow_model, factory)
        set_model_objective_function!(power_flow_model, factory)
        set_model_constraints!(power_flow_model, factory)

        return power_flow_model
    end

    include("model-creation-helpers/implementation-search_ac.jl")
    include("model-creation-helpers/implementation-search_dc.jl")
    include("../util-org/ramp_data_generation/rampingCSVimplementation_AC.jl")
    include("../util-org/ramp_data_generation/rampingCSVimplementation_DC.jl")

    ###########################################################################
    # Optimization function
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
    - `model`: The MPOPF model to optimize.
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