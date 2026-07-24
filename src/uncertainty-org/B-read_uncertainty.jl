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