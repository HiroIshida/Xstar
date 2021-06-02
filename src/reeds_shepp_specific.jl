function is_obstacle_free(rrtstar::RRTStar{3, <:ReedsSheppMetric}, node_start::Node, x_target)
    pts = waypoints(rrtstar.metric, node_start.x, x_target, 0.01)
    for pt in pts
        p = SVector{3, Float64}([pt[1], pt[2], pt[3]])
        is_obstacle_free(rrtstar, p) || (return false)
        is_inside(rrtstar.cspace, p) || (return false)
    end
    is_obstacle_free(rrtstar, x_target) || (return false)

    return true
end

function truncated_point(rrtstar::RRTStar{3, <:ReedsSheppMetric}, x_nearest, x_rand)
    pts = waypoints(rrtstar.metric, x_nearest, x_rand, rrtstar.mu)
    pt = pts[2]
    return SVector{3, Float64}(pt[1], pt[2], pt[3])
end

function visualize_nodes!(rrtstar::RRTStar{3, <:ReedsSheppMetric}, nodes, fig; color=:black, width=0.5, alpha=0.3)
    for node in nodes
        isnothing(node.parent_idx) && continue

        node_parent = rrtstar.nodes[node.parent_idx]
        x_child = node.x
        x_parent = node_parent.x

        pts = waypoints(rrtstar.metric, x_child, x_parent, 0.005)
        xs = [p[1] for p in pts]
        ys = [p[2] for p in pts]
        plot!(fig, xs, ys, label="", linecolor=color, linewidth=width, alpha=alpha)
    end
end


