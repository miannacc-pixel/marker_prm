function edge_cost = dist_stigmergy_mat(Dtravel, P_current, P_next, lambda, uncertainty_reference)
%DIST_STIGMERGY_MAT Computes one edge increment for the stigmergy objective.
%
% These increments telescope to Dtotal = sum(Dtravel) + lambda*trace(P_goal)/trace(P_initial).

edge_cost = Dtravel + lambda * ...
    (trace(P_next) - trace(P_current)) / uncertainty_reference;
end
