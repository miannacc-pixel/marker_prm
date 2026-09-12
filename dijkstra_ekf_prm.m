function [node, distance, predecessor] = dijkstra_ekf_prm(node, neighbor_ID, ...
    F, R, robot_speed, lambda, obstacle_edge, chi, bound, num_props, prop)
%DIJKSTRA_EKF_PRM Dijkstra search with propagated covariance and time.
%
% The PRM samples positions only. Whenever a shortest-path candidate extends
% from a settled vertex, its endpoint covariance is predicted as
% P_next = F*P_current*F' + R*Dtravel and its time is increased by
% Dtravel/robot_speed. The Dijkstra distance is the exact objective
% Dtotal = Dtravel + lambda*trace(P)/trace(P_initial) for the
% prediction-only model.

N = numel(node);
distance = inf(N,1);
predecessor = zeros(N,1);
settled = false(N,1);
uncertainty_reference = trace(node(1).P);
distance(1) = lambda * trace(node(1).P) / uncertainty_reference;

while true
    candidate_ID = find(~settled);
    if isempty(candidate_ID)
        break
    end

    [minimum_distance, relative_ID] = min(distance(candidate_ID));
    if isinf(minimum_distance)
        break
    end

    current_ID = candidate_ID(relative_ID);
    settled(current_ID) = true;
    next_ID = neighbor_ID{current_ID};
    next_ID(next_ID == current_ID) = [];

    for jj = 1:numel(next_ID)
        next_vertex = next_ID(jj);
        if settled(next_vertex)
            continue
        end

        travel_distance = norm(node(next_vertex).x - node(current_ID).x);
        P_next = F * node(current_ID).P * F.' + R * travel_distance;
        P_next = (P_next + P_next.') / 2;
        t_next = node(current_ID).t + travel_distance / robot_speed;

        % The collision checker requires the ellipse fields at both
        % ends of the edge, so construct only the candidate endpoint here.
        [ra, rb, ang, ellipse_rect] = error_ellipse(node(next_vertex).x, P_next, chi);
        next_node = node(next_vertex);
        next_node.P = P_next;
        next_node.t = t_next;
        next_node.ra = ra;
        next_node.rb = rb;
        next_node.ang = ang;
        next_node.ellipse_rect = ellipse_rect;

        issue_flag = psuedo_obs_check_line2_oct(node(current_ID), next_node, ...
            obstacle_edge, R, chi, bound, num_props, prop);
        if issue_flag
            continue
        end

        % This increment telescopes from the source to the endpoint, so each
        % stored distance equals total travel plus the normalized terminal
        % uncertainty term exactly.
        edge_cost = dist_stigmergy_mat(travel_distance, node(current_ID).P, ...
            next_node.P, lambda, uncertainty_reference);
        tentative_distance = distance(current_ID) + edge_cost;

        if tentative_distance < distance(next_vertex)
            distance(next_vertex) = tentative_distance;
            predecessor(next_vertex) = current_ID;
            node(next_vertex) = next_node;
        end
    end
end

for ii = 1:N
    node(ii).value = distance(ii);
    node(ii).parent = predecessor(ii);
end
end
