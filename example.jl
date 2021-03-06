using Revise
using Xstar
using Plots
using Random

using StaticArrays
using LinearAlgebra
gr()

function Xstar.is_obstacle_free(rrtstar::RRTStar, x)
    # custom collision function
    return norm(x[1:2] - [0.5, 0.5]) > 0.3
end

Random.seed!(1)

function example_reedsshepp()
    cspace = BoxSpace(3, [0.0, 0, -2*π], [1., 1., 2*π])
    x_start = SVector{3, Float64}([0.1, 0.1, 0.0])
    x_goal = SVector{3, Float64}([0.5, 0.9, 0.0])
    rrtstar = RRTStar(cspace, x_start, x_goal; metric=CppReedsSheppMetric(0.2), mu=0.3, is_informed=true) 
    @time for i in 1:1000
        println(i)
        is_success = extend(rrtstar)
        #=
        if mod(i, 50)==1
            fig = plot()
            visualize!(rrtstar, fig, with_solution=true)
            savefig(fig, "figs/plot_"*string(i)*".png")
        end
        =#
        #is_success && break
    end
    #Profile.print()

    fig = plot()
    visualize!(rrtstar, fig, with_solution=true)

# plot circle
    function circle(h, k, r)
        θ = LinRange(0, 2*π, 500)
        h .+ r*sin.(θ), k .+ r*cos.(θ)
    end
    plot!(fig, circle(0.5, 0.5, 0.3), label="", seriestype=[:shape],
          c=:blue, linecolor=:black, lw=0.5, fillalpha=0.2)
    fig
end

function example_euclidean()
    dim = 3
    cspace = BoxSpace(dim, zeros(dim), ones(dim))
    x_start = SVector{dim, Float64}([0.1 for _ in 1:dim])
    x_goal = SVector{dim, Float64}([0.9 for _ in 1:dim])
    rrtstar = RRTStar(cspace, x_start, x_goal; mu=0.3, is_informed=false) 
    @time for i in 1:2000
        is_success = extend(rrtstar)
        println(i)
        #is_success && break
    end

    fig = plot()
    visualize!(rrtstar, fig)

    # plot circle
    function circle(h, k, r)
        θ = LinRange(0, 2*π, 500)
        h .+ r*sin.(θ), k .+ r*cos.(θ)
    end
    plot!(fig, circle(0.5, 0.5, 0.3), label="", seriestype=[:shape],
          c=:blue, linecolor=:black, lw=0.5, fillalpha=0.2)
    fig
end
#example_euclidean()
example_reedsshepp()
