mutable struct Node{N}
    x::SVector{N, Float64}
    cost::Float64
    cost_to_goal::Float64
    idx::Int
    parent_idx::Union{Int, Nothing}
end
Node(x, cost, idx) = Node(x, cost, Inf, idx, nothing)

function visualize!(box::BoxSpace, fig)
    v1 = [box.lo[1], box.lo[2]]
    v2 = [box.hi[1], box.lo[2]]
    v3 = [box.hi[1], box.hi[2]]
    v4 = [box.lo[1], box.hi[2]]
    V = [v1, v2, v3, v4]
    for i in 1:4
        if i==4
            pre, post = V[4], V[1]
        else
            pre, post = V[i], V[i+1]
        end
        xs = [pre[1], post[1]]
        ys = [pre[2], post[2]]
        plot!(fig, xs, ys, label="", line=(:red))
    end
end

mutable struct RRTStar{N, MT}
    dim::Int
    cspace::ConfigurationSpace
    x_start::SVector{N}
    x_goal::SVector{N}
    sol_list::Vector{Node{N}}
    goal_node::Node{N}
    nodes::Vector{Node{N}}
    metric::MT
    mu::Float64
    is_informed::Bool
end
function RRTStar(cspace::ConfigurationSpace{N}, x_start, x_goal; mu=0.2, metric=nothing, is_informed=false) where N
    if isnothing(metric)
        metric = Euclidean()
    end
    nodes = Vector{Node{N}}(undef, 0)
    goal_node = Node(x_goal, Inf, -1)
    push!(nodes, Node(x_start, 0.0, 1))
    sol_list = Vector{Node{N}}(undef, 0)
    RRTStar{N, typeof(metric)}(N, cspace, x_start, x_goal, sol_list, goal_node, nodes, metric, mu, is_informed)
end
solution_found(rrtstar::RRTStar) = !isempty(rrtstar.sol_list)

informed_sampling(rrtstar::RRTStar) = error("not implemented") # default
function informed_sampling(rrtstar::RRTStar{3, MT}) where MT <: ReedsSheppMetric
    ellipse = EllipticSE2(rrtstar.goal_node.cost, rrtstar.x_start, rrtstar.x_goal)
    x_rand = uniform_sampling(ellipse)
end

function extend(rrtstar::RRTStar{N}) where N
    if solution_found(rrtstar)
        cost, idx_best = findmin([node.cost + node.cost_to_goal for node in rrtstar.sol_list])
        rrtstar.goal_node.cost = cost
        rrtstar.goal_node.parent_idx = rrtstar.sol_list[idx_best].idx
    end

    if solution_found(rrtstar) && rrtstar.is_informed
        x_rand = informed_sampling(rrtstar)
    else
        x_rand = uniform_sampling(rrtstar.cspace)
    end
    node_nearest, x_new = _find_nearest_and_new(rrtstar, x_rand)
    is_obstacle_free(rrtstar, node_nearest, x_new) || (return false) 

    # find best node and cost
    node_min = node_nearest
    cost_min = node_nearest.cost + rrtstar.metric(node_nearest.x, x_new)

    node_nears = _find_nears(rrtstar, x_new)
    for node_near in node_nears
        # TODO cache metric computation
        cost_expect = node_near.cost + rrtstar.metric(node_near.x, x_new)
        if cost_expect < cost_min
            if is_obstacle_free(rrtstar, node_near, x_new)
                node_min = node_near
                cost_min = cost_expect
            end
        end
    end

    idx_new = length(rrtstar.nodes) + 1
    @assert is_obstacle_free(rrtstar, x_new)
    node_new = Node(x_new, cost_min, Inf, idx_new, node_min.idx)
    push!(rrtstar.nodes, node_new)

    # rewire
    for node_near in node_nears
        is_obstacle_free(rrtstar, node_new, node_near.x) || continue
        rewired_cost = node_new.cost + rrtstar.metric(node_near.x, x_new)
        if rewired_cost < node_near.cost
            node_near.parent_idx = node_new.idx
            node_near.cost = rewired_cost
        end
    end

    is_reached = is_obstacle_free(rrtstar, node_new, rrtstar.x_goal)
    if is_reached
        node_new.cost_to_goal = rrtstar.metric(node_new.x, rrtstar.x_goal)
        push!(rrtstar.sol_list, node_new)
    end
    return is_reached
end

is_obstacle_free(rrtstar::RRTStar, x) = error("please implement this")

function is_obstacle_free(rrtstar::RRTStar{<:Any, Euclidean}, node_start::Node, x_target)
    # assume x1 is known to be obstacle free
    n_wp = 5
    x_start = node_start.x
    step = (x_target - x_start)/n_wp
    for i in 1:n_wp
        is_obstacle_free(rrtstar, x_start + step * i) || (return false)
    end
    return true
end

function _find_nears(rrtstar::RRTStar{N}, x_center) where N
    gamma = 1.0
    card = length(rrtstar.nodes)
    r = min(gamma * (log(card)/card)^(1/N), rrtstar.mu)
    function pred(node)
        worse_calculating(rrtstar.metric, node.x, x_center, r) || (return false)
        return rrtstar.metric(node.x, x_center) < r
    end
    filter(pred, rrtstar.nodes)
end

function _find_nearest_and_new(rrtstar::RRTStar, x_rand)
    dist_min = Inf
    idx_min = -1
    for node in rrtstar.nodes
        worse_calculating(rrtstar.metric, node.x, x_rand, dist_min) || continue
        dist_cand = rrtstar.metric(node.x, x_rand)
        if dist_cand < dist_min
            dist_min = dist_cand
            idx_min = node.idx
        end
    end

    node_nearest = rrtstar.nodes[idx_min]
    x_nearest = node_nearest.x
    if dist_min < rrtstar.mu
        x_new = x_rand
    else
        x_new = truncated_point(rrtstar, x_nearest, x_rand)
    end
    return node_nearest, x_new
end

function truncated_point(rrtstar::RRTStar, x_nearest, x_rand)
    x_new = x_nearest + normalize(x_rand - x_nearest) * rrtstar.mu
end

function visualize_nodes!(rrtstar::RRTStar, nodes, fig; color=:black, width=0.5, alpha=0.3)
    for node in nodes
        isnothing(node.parent_idx) && continue

        node_parent = rrtstar.nodes[node.parent_idx]
        x_child = node.x
        x_parent = node_parent.x
        xs, ys = [[x_child[i], x_parent[i]] for i in 1:2]
        plot!(fig, xs, ys, label="", linecolor=color, linewidth=width, alpha=alpha)
    end
end

function visualize!(rrtstar::RRTStar, fig; with_arrow=false, with_solution=true)
    visualize!(rrtstar.cspace, fig)
    xs, ys = [[n.x[i] for n in rrtstar.nodes] for i in 1:2]
    scatter!(fig, xs, ys, label="", markercolor=:green, markersize=5, markeralpha=0.5)
    visualize_nodes!(rrtstar, rrtstar.nodes, fig)
    if with_solution && solution_found(rrtstar)
        nodes_path = back_trace(rrtstar, rrtstar.goal_node)
        xs, ys = [[n.x[i] for n in nodes_path] for i in 1:2]
        scatter!(fig, xs, ys, label="", markercolor=:blue, markersize=7, markeralpha=1.0)
        visualize_nodes!(rrtstar, nodes_path, fig; color=:blue, width=2.0, alpha=1.0)
    end
end
function back_trace(rrtstar::RRTStar, goal_node::Node)
    nodes = Node[]
    push!(nodes, goal_node)
    node = goal_node
    while node.idx!=1
        node = rrtstar.nodes[node.parent_idx]
        @assert is_obstacle_free(rrtstar, node.x)
        push!(nodes, node)
    end
    reverse!(nodes)
    return nodes
end

