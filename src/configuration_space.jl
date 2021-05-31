abstract type ConfigurationSpace{N} end
uniform_sampling(cspace::ConfigurationSpace) = error("please implement this")

struct BoxSpace{N} <: ConfigurationSpace{N}
    dim::Int
    lo::SVector{N, Float64}
    hi::SVector{N, Float64}
end
BoxSpace(dim::Int, lo, hi) = BoxSpace{dim}(dim, SVector{dim, Float64}(lo), SVector{dim, Float64}(hi))
function is_inside(space::BoxSpace, x)
    for i in 1:space.dim
        x[i] < space.lo[i] && (return false)
        x[i] > space.hi[i] && (return false)
    end
    return true
end

function uniform_sampling(box::BoxSpace)
    width = box.hi - box.lo
    r = box.lo + width .* rand(Float64, box.dim)
end

struct EllipticSE2 <: ConfigurationSpace{3}
    x_center::SVector{2, Float64} # TODO Euclid 2
    r1::Float64
    r2::Float64
    angle::Float64
end
function EllipticSE2(c_best, x_start, x_goal)
    c_min = norm(x_start[1:2] - x_goal[1:2])
    r1 = c_best * 0.5
    r2 = sqrt(c_best^2 - c_min^2) * 0.5
    angle = atan(x_goal[2] - x_start[2], x_goal[1] - x_start[1])
    center = 0.5 * (x_start[1:2] + x_goal[1:2])
    EllipticSE2(center, r1, r2, angle)
end

function uniform_sampling(cspace::EllipticSE2)
    # just rejection sampling is find because only 2dim 
    while true
        p = 2 * rand(Float64, 2) .* [cspace.r1, cspace.r2] .- [cspace.r1, cspace.r2]
        is_inside = (p[1]/cspace.r1)^2 + (p[2]/cspace.r2)^2 < 1.0
        if is_inside
            a = cspace.angle
            mat = [cos(a) -sin(a); sin(a) cos(a)]
            p_ = mat * p + cspace.x_center
            theta_ = rand() * 2 * π - π
            return SVector{3, Float64}([p_[1], p_[2], theta_])
        end
    end
end
