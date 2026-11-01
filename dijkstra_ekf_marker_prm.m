function [node, distance, predecessor, path, goal_cost, search] = dijkstra_ekf_marker_prm( ...
    node, neighbor_ID, F, R, robot_speed, lambda, marker, obstacle_edge, ...
    chi, bound, num_props, prop, target)
%DIJKSTRA_EKF_MARKER_PRM Layered Dijkstra search with optional markers.
%
% Layer 1 represents paths before any marker is sensed. Each additional layer
% represents a subset of markers after their single EKF measurement updates.
% Only reachable subsets are allocated, in increasing subset cardinality.
% In the one-belief-per-vertex/layer approximation, the number of layers
% can grow to 2^n.

if ~isequal(F, eye(2))
    error('marker_prm:PredictionModel', 'The marker planner requires F = eye(2).')
end
validateattributes(robot_speed, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(lambda, {'numeric'}, {'scalar','real','finite','nonnegative'});
for k = 1:numel(marker)
    validateattributes(marker(k).x, {'numeric'}, {'size',[1 2],'real','finite'});
    validateattributes(marker(k).sensing_radius, {'numeric'}, ...
        {'scalar','real','finite','nonnegative'});
    validateattributes(marker(k).H, {'numeric'}, {'2d','real','finite','ncols',2});
    m = size(marker(k).H,1);
    validateattributes(marker(k).R, {'numeric'}, {'size',[m m],'real','finite'});
    [~,flag] = chol(marker(k).R);
    if flag || ~isequal(marker(k).R, marker(k).R.')
        error('marker_prm:MeasurementNoise', 'Marker R must be symmetric positive definite.')
    end
end
N = numel(node);
infP = nan(2,2,N);
P_layers = {infP}; t_layers = {inf(N,1)};
distance = inf(N,1); predecessor = zeros(N,1);
parent_state = zeros(N,1);

uncertainty_reference = trace(node(1).P);
validateattributes(uncertainty_reference, {'numeric'}, {'scalar','finite','positive'});
distance(1,1) = lambda * trace(node(1).P) / uncertainty_reference;
P_layers{1}(:,:,1) = node(1).P;
t_layers{1}(1) = 0;
layer_markers = false(1,numel(marker));
processed = false(1,1);
layer_index = containers.Map('KeyType','char','ValueType','double');
layer_index(['s',char('0'+layer_markers(1,:))]) = 1;

while any(~processed)
    pending = find(~processed);
    [~, relative] = min(sum(layer_markers(pending,:),2));
    state = pending(relative);
    sensed = layer_markers(state,:);

    % Paths within this layer have not entered any unsensed marker's range.
    % All subsequent edges within the layer have nonnegative prediction-only
    % cost increments, so ordinary Dijkstra is valid.
    settled = false(N,1);
    while true
        ID = find(~settled);
        if isempty(ID), break, end
        [minimum, relative] = min(distance(ID,state));
        if isinf(minimum), break, end
        current = ID(relative);
        settled(current) = true;
        next_ID = neighbor_ID{current};
        next_ID(next_ID == current) = [];
        for jj = 1:numel(next_ID)
            next = next_ID(jj);
            if settled(next), continue, end
            ids = marker_events(node(current).x, node(next).x, marker, sensed);
            if ~isempty(ids), continue, end
            [nextP, nextt, edge_cost, feasible] = normal_edge(node(current).x, ...
                node(next).x, P_layers{state}(:,:,current), t_layers{state}(current), F, R, ...
                robot_speed, lambda, uncertainty_reference, obstacle_edge, chi, ...
                bound, num_props, prop);
            if feasible && distance(current,state) + edge_cost < distance(next,state)
                distance(next,state) = distance(current,state) + edge_cost;
                P_layers{state}(:,:,next) = nextP; t_layers{state}(next) = nextt;
                predecessor(next,state) = current; parent_state(next,state) = state;
            end
        end
    end

    % Marker transitions seed the after-sensing layers. These are the only edges
    % that can reduce covariance, and they never return to the before layer.
    for current = 1:N
        if ~isfinite(distance(current,state)), continue, end
        next_ID = neighbor_ID{current};
        next_ID(next_ID == current) = [];
        for jj = 1:numel(next_ID)
            next = next_ID(jj);
            [ids, fractions] = marker_events(node(current).x, node(next).x, marker, sensed);
            if isempty(ids), continue, end
            [nextP, nextt, edge_cost, feasible] = marker_edge(node(current).x, ...
                node(next).x, P_layers{state}(:,:,current), t_layers{state}(current), F, R, ...
                robot_speed, lambda, uncertainty_reference, marker, ids, fractions, ...
                obstacle_edge, chi, bound, num_props, prop);
            if ~feasible, continue, end
            updated = sensed; updated(ids) = true;
            key = ['s',char('0'+updated)];
            if isKey(layer_index,key)
                next_state = layer_index(key);
            else
                next_state = size(distance,2)+1;
                layer_index(key) = next_state;
                layer_markers(next_state,:) = updated;
                processed(next_state) = false;
                distance(:,next_state) = inf; predecessor(:,next_state) = 0;
                parent_state(:,next_state) = 0;
                P_layers{next_state} = infP; t_layers{next_state} = inf(N,1);
            end
            if distance(current,state) + edge_cost < distance(next,next_state)
                distance(next,next_state) = distance(current,state) + edge_cost;
                P_layers{next_state}(:,:,next) = nextP; t_layers{next_state}(next) = nextt;
                predecessor(next,next_state) = current; parent_state(next,next_state) = state;
            end
        end
    end
    processed(state) = true;
end

% Choose the lower-cost target across all layers, then copy its beliefs into
% the node format used by the plotter.
[goal_vertex, goal_state, goal_cost] = choose_goal(node, P_layers, distance, chi, target);
path = []; state_path = [];
if isempty(goal_vertex)
    goal_cost = -10^10;
else
    [path, state_path] = reconstruct_layered_path(goal_vertex, goal_state, ...
        predecessor, parent_state);
end
roadmap_path = path;
used = false(N,1);
for kk = 1:numel(path)
    vertex = roadmap_path(kk); state = state_path(kk);
    % Revisited vertices need separate copies to retain both path beliefs.
    if used(vertex)
        node(end+1) = node(vertex); path(kk) = numel(node);
    end
    used(vertex) = true;
    node(path(kk)).P = P_layers{state}(:,:,vertex);
    node(path(kk)).t = t_layers{state}(vertex);
    node(path(kk)).value = distance(vertex,state);
    node(path(kk)).parent = 0;
    if kk > 1, node(path(kk)).parent = path(kk-1); end
    [node(path(kk)).ra,node(path(kk)).rb,node(path(kk)).ang, ...
        node(path(kk)).ellipse_rect] = error_ellipse(node(path(kk)).x, node(path(kk)).P, chi);
end
search = struct('layer_markers',layer_markers,'predecessor_layer',parent_state, ...
    'roadmap_path',roadmap_path,'path_layers',state_path,'roadmap_size',N);
end

function [nextP,nextt,cost,feasible] = normal_edge(x0,x1,P,t,F,R,v,lambda,uncertainty_reference,edges,chi,bound,nprop,prop)
d = norm(x1-x0); nextP = F*P*F.' + R*d; nextP = (nextP+nextP.')/2; nextt = t+d/v;
feasible = ~psuedo_obs_check_line2_oct(belief_node(x0,P,chi), belief_node(x1,nextP,chi), edges,R,chi,bound,nprop,prop);
cost = dist_stigmergy_mat(d, P, nextP, lambda, uncertainty_reference);
end

function [nextP,nextt,cost,feasible] = marker_edge(x0,x1,P,t,F,R,v,lambda,uncertainty_reference,marker,ids,fractions,edges,chi,bound,nprop,prop)
d = norm(x1-x0); Pend = F*P*F.'+R*d; Pend=(Pend+Pend.')/2; nextt=t+d/v;
feasible = ~psuedo_obs_check_line2_oct(belief_node(x0,P,chi), belief_node(x1,Pend,chi), edges,R,chi,bound,nprop,prop);
% Prediction-only envelope for collision checking, including edges with measurements.
nextP=marker_edge_covariance(P,d,R,marker,ids,fractions);
cost=dist_stigmergy_mat(d, P, nextP, lambda, uncertainty_reference);
end

function n = belief_node(x,P,chi)
[ra,rb,ang,rect]=error_ellipse(x,P,chi); n=struct('x',x,'P',P,'ra',ra,'rb',rb,'ang',ang,'ellipse_rect',rect);
end

function [v,s,c] = choose_goal(node,P_layers,distance,chi,target)
v=[]; s=[]; c=inf;
for ii=1:numel(node)
    for state=1:size(distance,2)
        P=P_layers{state}(:,:,ii); d=distance(ii,state);
        if ~isfinite(d), continue, end
        [~,~,~,r]=error_ellipse(node(ii).x,P,chi);
        inside=r(1)>=target(1,1)&&r(1)+r(3)<=target(1,2)&&r(2)>=target(2,1)&&r(2)+r(4)<=target(2,2);
        if inside&&d<c, v=ii; s=state; c=d; end
    end
end
end

function [path,states] = reconstruct_layered_path(v,s,predecessor,parent_state)
path=v; states=s;
while ~(v==1&&s==1)
    oldv=predecessor(v,s); s=parent_state(v,s); v=oldv;
    path=[v;path]; states=[s;states];
end
end
