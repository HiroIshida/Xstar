using Revise
using Xstar
using Test
using Plots
gr()

function test()
    metric1 = CppReedsSheppMetric(0.5)
    metric2 = PythonReedsSheppMetric(0.5)
    p1 = [0., 0, 0]
    p2 = [1.0, 1.0, 1.0]
    path = Xstar.create_path(metric1, p1, p2)
    @test metric1(p1, p2) == metric2(p1, p2)

    fig = plot()
    xs = Float64[]
    ys = Float64[]
    dist = Xstar.distance(metric1.path_cache)
    step = 0.03
    itr = 0
    while true
        a = zeros(3)
        seg = 0.0 + step*itr
        println(seg)
        Xstar.interpolate(path, seg, a)

        push!(xs, a[1])
        push!(ys, a[2])
        itr += 1
        seg > dist && break
    end
    scatter!(fig, xs, ys)
    fig
end
fig = test()

