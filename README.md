# IG-RRTstar-rapid
===============================================================================================================================
Basic information
First, create a empty folder "data", where the data will be saved.
By running "main.m", you can start the information-geometric PRM path-planning algorithm.
===============================================================================================================================

Codes related with the Path-planning simulation:

main.m: The main function to run the directed information-geometric PRM

dijkstra_ekf_prm.m: performs Dijkstra relaxation, propagates covariance and elapsed time, and evaluates the `Dtotal` edge increment

dijkstra_ekf_marker_prm.m: compares the no-marker and marker-sensed layers when `marker_enabled` is true

dist_stigmergy_mat.m: defines the shared `Dtotal` edge-cost increment for both marker-on and marker-off searches

marker_encounter.m & ekf_update_covariance.m: detect marker-range entry and apply the EKF measurement update.

reconstruct_prm_path.m: Reconstructs the final source-to-target PRM path

marker_events.m: Finds unsensed markers within sensing range of a roadmap edge and orders their measurements along the direction of travel.

marker_edge_covariance.m: Propagates covariance along a roadmap edge and applies the ordered EKF marker updates. Also computes intermediate covariances for plotting.

obstacle_multi.m & obstacle_polyshape.m: specify the obstacles position and their shape for a sample "multiple obstacles" enviroment

sample_free_position.m: samples a collision-free spatial PRM vertex; does not sample covariance

error_ellipse.m: Calculate the length of the long & short axes, angle, and bounding box of a given ellipse

psuedo_obs_check_line_oct.m: It checks if a collision with obstacles during the transition between nodes occurs. psuedo_obs_check_line2_oct.m: The same with the above

Is_two_lineseg_cross.m: check if two line segment intersect make_octagon.m: generate a octagon from a bounding box for better obstacle checking minDist_two_LineSeg_in.m: compute the minimum distance between two line segments. It is used for collision pre-checking
**********************************************************
===============================================================================================================================
Codes related with the plotting
plot_for_paper_multipleObs: This function plots results for "multiple obstacles" enviroment
plot_path_cost: It plots the path length as a function of number of samples. It should be run right after main.m
===============================================================================================================================
