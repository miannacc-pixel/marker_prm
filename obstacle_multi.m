function obstacle_edge = obstacle_multi(environment)
% Defines an obstacle as a set of edges 
% each edge is defined by: start point, end point, slope, and Y_axis
% intercept

if strcmp(environment, 'baseline')

% Obstacle_1
x(1).vertex = [0.4,0.0];
vertex_index= length(x);
x(end+1).vertex = [0.5,0.0];
x(end+1).vertex = [0.5,0.45]; 
x(end+1).vertex = [0.4,0.45]; 
for i=vertex_index:length(x)
obstacle_edge(i).start = x(i).vertex;
if (i~=length(x))
obstacle_edge(i).end = x(i+1).vertex;
else
   obstacle_edge(i).end = x(vertex_index).vertex; 
end
obstacle_edge(i).slope = (obstacle_edge(i).start(2)-obstacle_edge(i).end(2))/...
    (obstacle_edge(i).start(1)-obstacle_edge(i).end(1));
obstacle_edge(i).y_inter = obstacle_edge(i).start(2) -...
    obstacle_edge(i).slope*obstacle_edge(i).start(1);
end

% Obstacle_2
x(end+1).vertex = [0.4,0.65]; 
vertex_index= length(x);
x(end+1).vertex = [0.5,0.65];
x(end+1).vertex = [0.5,1.0];
x(end+1).vertex = [0.4,1.0];
for i=vertex_index:length(x)
obstacle_edge(i).start = x(i).vertex;
if (i~=length(x))
obstacle_edge(i).end = x(i+1).vertex;
else
   obstacle_edge(i).end = x(vertex_index).vertex; 
end
obstacle_edge(i).slope = (obstacle_edge(i).start(2)-obstacle_edge(i).end(2))/...
    (obstacle_edge(i).start(1)-obstacle_edge(i).end(1));
obstacle_edge(i).y_inter = obstacle_edge(i).start(2) -...
    obstacle_edge(i).slope*obstacle_edge(i).start(1);
end

elseif strcmp(environment, 'visual_abstract')

% Visual abstract environment: five polygonal obstacles.
obstacle_vertices = { ...
    [0.20,0.76; 0.43,0.80; 0.40,0.60; 0.23,0.61], ...
    [0.18,0.40; 0.34,0.47; 0.39,0.26; 0.22,0.21], ...
    [0.65,0.40; 0.64,0.20; 0.76,0.25; 0.75,0.45], ...
    [0.62,0.75; 0.80,0.80; 0.81,0.63; 0.66,0.60; 0.59,0.69], ...
    [0.40,0.50; 0.60,0.45; 0.50,0.60]};
obstacle_edge = vertices_to_edges(obstacle_vertices);

elseif strcmp(environment, 'forest')

% Forest environment: small octagonal obstacles represent tree trunks.
obstacle_vertices = { ...
    make_octagon_vertices([0.22,0.28],0.055), ...
    make_octagon_vertices([0.20,0.68],0.05), ...
    make_octagon_vertices([0.15,0.48],0.045), ...
    make_octagon_vertices([0.34,0.16],0.05), ...
    make_octagon_vertices([0.38,0.50],0.06), ...
    make_octagon_vertices([0.38,0.82],0.05), ...
    make_octagon_vertices([0.52,0.22],0.05), ...
    make_octagon_vertices([0.56,0.72],0.06), ...
    make_octagon_vertices([0.52,0.92],0.04), ...
    make_octagon_vertices([0.68,0.13],0.045), ...
    make_octagon_vertices([0.72,0.43],0.055), ...
    make_octagon_vertices([0.74,0.67],0.045), ...
    make_octagon_vertices([0.78,0.78],0.05), ...
    make_octagon_vertices([0.88,0.38],0.045)};
obstacle_edge = vertices_to_edges(obstacle_vertices);

elseif strcmp(environment, 'cavern')

% Cavern environment: two obstacles form a constrained passage.
obstacle_vertices = { ...
    [0.30,0.70; 1.00,0.70; 1.00,0.80; 0.30,0.80], ...
    [0.45,0.20; 0.75,0.20; 0.75,0.50; 0.45,0.50]};
obstacle_edge = vertices_to_edges(obstacle_vertices);

else
    error('Unknown environment selection.')
end
end

function vertices = make_octagon_vertices(center,radius)
angles = (0:7)'*(pi/4)+pi/8;
vertices = center+radius*[cos(angles),sin(angles)];
end

function obstacle_edge = vertices_to_edges(obstacle_vertices)
edge_index = 0;
for obstacle_index = 1:length(obstacle_vertices)
    vertices = obstacle_vertices{obstacle_index};
    for vertex_index = 1:size(vertices,1)
        edge_index = edge_index+1;
        next_vertex = mod(vertex_index,size(vertices,1))+1;
        obstacle_edge(edge_index).start = vertices(vertex_index,:); %#ok<AGROW>
        obstacle_edge(edge_index).end = vertices(next_vertex,:);
        obstacle_edge(edge_index).slope = ...
            (obstacle_edge(edge_index).start(2)-obstacle_edge(edge_index).end(2))/...
            (obstacle_edge(edge_index).start(1)-obstacle_edge(edge_index).end(1));
        obstacle_edge(edge_index).y_inter = obstacle_edge(edge_index).start(2)-...
            obstacle_edge(edge_index).slope*obstacle_edge(edge_index).start(1);
    end
end
end
