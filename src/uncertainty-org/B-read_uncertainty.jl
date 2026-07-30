using CSV
using DataFrames

function read_uncertainty(filename)

    df = CSV.read(filename, DataFrame)

    uncertainty = Dict()

    for row in eachrow(df)

        uncertainty[Int(row.bus_id)] = (
            mu = row.mu,
            sigma = row.sigma
        )

    end

    return uncertainty

end

function read_renewable_uncertainty(filename)

    df = CSV.read(filename, DataFrame)

    renewable = Dict()

    for row in eachrow(df)

        renewable[Int(row.gen_id)] = (
            bus_id = Int(row.bus_id),
            mu = row.mu,
            sigma = row.sigma
        )

    end

    return renewable

end