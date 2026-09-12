function [node, distance, predecessor, path, goal_cost] = dijkstra_ekf_marker_prm( ...
    node, neighbor_ID, F, R, robot_speed, lambda, marker, obstacle_edge, ...
    chi, bound, num_props, prop, target)
%DIJKSTRA_EKF_MARKER_PRM Two-layer Dijkstra search with an optional marker.
%
% Layer 1 represents paths before the marker is sensed. Layer 2 represents
% paths after its single EKF measurement update.

N = numel(node);
infP = nan(2,2,N);
P_before = infP; P_after = infP;
t_before = inf(N,1); t_after = inf(N,1);
d_before = inf(N,1); d_after = inf(N,1);
parent_before = zeros(N,1); parent_after = zeros(N,1);
parent_after_state = zeros(N,1);

uncertainty_reference = trace(node(1).P);
d_before(1) = lambda * trace(node(1).P) / uncertainty_reference;
P_before(:,:,1) = node(1).P;
t_before(1) = 0;
settled = false(N,1);

% First layer: paths that have not entered the marker sensing range.
while true
    ID = find(~settled);
    if isempty(ID), break, end
    [minimum, relative] = min(d_before(ID));
    if isinf(minimum), break, end
    current = ID(relative);
    settled(current) = true;
    next_ID = neighbor_ID{current};
    next_ID(next_ID == current) = [];
    for jj = 1:numel(next_ID)
        next = next_ID(jj);
        if settled(next), continue, end
        [enters_marker, ~] = marker_encounter(node(current).x, node(next).x, marker);
        if enters_marker, continue, end
        [nextP, nextt, edge_cost, feasible] = normal_edge(node(current).x, ...
            node(next).x, P_before(:,:,current), t_before(current), F, R, ...
            robot_speed, lambda, uncertainty_reference, obstacle_edge, chi, ...
            bound, num_props, prop);
        if feasible && d_before(current) + edge_cost < d_before(next)
            d_before(next) = d_before(current) + edge_cost;
            P_before(:,:,next) = nextP; t_before(next) = nextt;
            parent_before(next) = current;
        end
    end
end

% Marker transitions seed the after-sensing layer. These are the only edges
% that can reduce covariance, and they never return to the before layer.
for current = 1:N
    if ~isfinite(d_before(current)), continue, end
    next_ID = neighbor_ID{current};
    next_ID(next_ID == current) = [];
    for jj = 1:numel(next_ID)
        next = next_ID(jj);
        [enters_marker, fraction] = marker_encounter(node(current).x, node(next).x, marker);
        if ~enters_marker, continue, end
        [nextP, nextt, edge_cost, feasible] = marker_edge(node(current).x, ...
            node(next).x, P_before(:,:,current), t_before(current), F, R, ...
            robot_speed, lambda, uncertainty_reference, marker, fraction, ...
            obstacle_edge, chi, bound, num_props, prop);
        if feasible && d_before(current) + edge_cost < d_after(next)
            d_after(next) = d_before(current) + edge_cost;
            P_after(:,:,next) = nextP; t_after(next) = nextt;
            parent_after(next) = current; parent_after_state(next) = 1;
        end
    end
end

% Second layer: paths after the marker has been sensed. All subsequent edges
% have nonnegative prediction-only cost increments, so ordinary Dijkstra is
% valid again.
settled = false(N,1);
while true
    ID = find(~settled);
    if isempty(ID), break, end
    [minimum, relative] = min(d_after(ID));
    if isinf(minimum), break, end
    current = ID(relative);
    settled(current) = true;
    next_ID = neighbor_ID{current};
    next_ID(next_ID == current) = [];
    for jj = 1:numel(next_ID)
        next = next_ID(jj);
        if settled(next), continue, end
        [nextP, nextt, edge_cost, feasible] = normal_edge(node(current).x, ...
            node(next).x, P_after(:,:,current), t_after(current), F, R, ...
            robot_speed, lambda, uncertainty_reference, obstacle_edge, chi, ...
            bound, num_props, prop);
        if feasible && d_after(current) + edge_cost < d_after(next)
            d_after(next) = d_after(current) + edge_cost;
            P_after(:,:,next) = nextP; t_after(next) = nextt;
            parent_after(next) = current; parent_after_state(next) = 2;
        end
    end
end

% Choose the lower-cost target across both layers, then copy its beliefs into
% the node format used by the plotter.
[goal_vertex, goal_state, goal_cost] = choose_goal(node, P_before, P_after, ...
    d_before, d_after, chi, target);
path = [];
if isempty(goal_vertex)
    goal_cost = -10^10;
    distance = [d_before d_after]; predecessor = [parent_before parent_after];
    return
end

[path, state_path] = reconstruct_layered_path(goal_vertex, goal_state, ...
    parent_before, parent_after, parent_after_state);
for kk = 1:numel(path)
    if state_path(kk) == 1
        node(path(kk)).P = P_before(:,:,path(kk));
        node(path(kk)).t = t_before(path(kk));
        node(path(kk)).value = d_before(path(kk));
    else
        node(path(kk)).P = P_after(:,:,path(kk));
        node(path(kk)).t = t_after(path(kk));
        node(path(kk)).value = d_after(path(kk));
    end
    if kk > 1, node(path(kk)).parent = path(kk-1); end
    [node(path(kk)).ra,node(path(kk)).rb,node(path(kk)).ang, ...
        node(path(kk)).ellipse_rect] = error_ellipse(node(path(kk)).x, node(path(kk)).P, chi);
end
distance = [d_before d_after]; predecessor = [parent_before parent_after];
end

function [nextP,nextt,cost,feasible] = normal_edge(x0,x1,P,t,F,R,v,lambda,uncertainty_reference,edges,chi,bound,nprop,prop)
d = norm(x1-x0); nextP = F*P*F.' + R*d; nextP = (nextP+nextP.')/2; nextt = t+d/v;
feasible = ~psuedo_obs_check_line2_oct(belief_node(x0,P,chi), belief_node(x1,nextP,chi), edges,R,chi,bound,nprop,prop);
cost = dist_stigmergy_mat(d, P, nextP, lambda, uncertainty_reference);
end

function [nextP,nextt,cost,feasible] = marker_edge(x0,x1,P,t,F,R,v,lambda,uncertainty_reference,marker,fraction,edges,chi,bound,nprop,prop)
d = norm(x1-x0); Pend = F*P*F.'+R*d; Pend=(Pend+Pend.')/2; nextt=t+d/v;
feasible = ~psuedo_obs_check_line2_oct(belief_node(x0,P,chi), belief_node(x1,Pend,chi), edges,R,chi,bound,nprop,prop);
Ppre=F*P*F.'+R*(fraction*d); Ppre=(Ppre+Ppre.')/2;
Ppost=ekf_update_covariance(Ppre,marker.H,marker.R);
nextP=F*Ppost*F.'+R*((1-fraction)*d); nextP=(nextP+nextP.')/2;
cost=dist_stigmergy_mat(d, P, nextP, lambda, uncertainty_reference);
end

function n = belief_node(x,P,chi)
[ra,rb,ang,rect]=error_ellipse(x,P,chi); n=struct('x',x,'P',P,'ra',ra,'rb',rb,'ang',ang,'ellipse_rect',rect);
end

function [v,s,c] = choose_goal(node,P1,P2,d1,d2,chi,target)
v=[]; s=[]; c=inf;
for ii=1:numel(node)
    for state=1:2
        if state==1, P=P1(:,:,ii); d=d1(ii); else, P=P2(:,:,ii); d=d2(ii); end
        if ~isfinite(d), continue, end
        [~,~,~,r]=error_ellipse(node(ii).x,P,chi);
        inside=r(1)>=target(1,1)&&r(1)+r(3)<=target(1,2)&&r(2)>=target(2,1)&&r(2)+r(4)<=target(2,2);
        if inside&&d<c, v=ii; s=state; c=d; end
    end
end
end

function [path,states] = reconstruct_layered_path(v,s,p1,p2,p2state)
path=v; states=s;
while ~(v==1&&s==1)
    if s==1, v=p1(v); s=1; else, oldv=p2(v); s=p2state(v); v=oldv; end
    path=[v;path]; states=[s;states];
end
end
