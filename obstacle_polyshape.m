function obs_poly = obstacle_polyshape(environment)
%This function defines an obstacle as a set of polyshapes
% This function is defined to use polyshape functionalities of Matlab

if strcmp(environment, 'baseline')

n=2; % number of obstacles
obs_poly(1:n)= struct('x',[], 'y',[]);

% Obstacle_1
obs_poly(1).x = [0.4 0.5 0.5 0.4];
obs_poly(1).y = [0.0 0.0 0.45 0.45];

% Obstacle_2
obs_poly(2).x = [0.4 0.5 0.5 0.4];
obs_poly(2).y = [0.65 0.65 1.0 1.0];

elseif strcmp(environment, 'visual_abstract')

% Visual abstract environment: five polygonal obstacles.
obstacle_vertices = { ...
    [0.20,0.76; 0.43,0.80; 0.40,0.60; 0.23,0.61], ...
    [0.18,0.40; 0.34,0.47; 0.39,0.26; 0.22,0.21], ...
    [0.65,0.40; 0.64,0.20; 0.76,0.25; 0.75,0.45], ...
    [0.62,0.75; 0.80,0.80; 0.81,0.63; 0.66,0.60; 0.59,0.69], ...
    [0.40,0.50; 0.60,0.45; 0.50,0.60]};
obs_poly = vertices_to_polyshape(obstacle_vertices);

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
obs_poly = vertices_to_polyshape(obstacle_vertices);

elseif strcmp(environment, 'cavern')

% Cavern environment: two obstacles form a constrained passage.
obstacle_vertices = { ...
    [0.30,0.70; 1.00,0.70; 1.00,0.80; 0.30,0.80], ...
    [0.45,0.20; 0.75,0.20; 0.75,0.50; 0.45,0.50]};
obs_poly = vertices_to_polyshape(obstacle_vertices);

else
    error('Unknown environment selection.')
end
end

function vertices = make_octagon_vertices(center,radius)
angles = (0:7)'*(pi/4)+pi/8;
vertices = center+radius*[cos(angles),sin(angles)];
end

function obs_poly = vertices_to_polyshape(obstacle_vertices)
n=length(obstacle_vertices); % number of obstacles
obs_poly(1:n)= struct('x',[], 'y',[]);
for obstacle_index = 1:n
    obs_poly(obstacle_index).x = obstacle_vertices{obstacle_index}(:,1)';
    obs_poly(obstacle_index).y = obstacle_vertices{obstacle_index}(:,2)';
end
end
