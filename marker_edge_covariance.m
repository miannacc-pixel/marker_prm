function Pnext = marker_edge_covariance(P, distance, R, marker, ids, fractions, endpoint)
%MARKER_EDGE_COVARIANCE Predict and apply ordered, once-only measurements.
% endpoint permits plotting intermediate beliefs using the same propagation.
% Uses the random-walk model F = eye(2).
if nargin < 7, endpoint = 1; end
previous = 0;
for k = 1:numel(ids)
    if fractions(k) > endpoint, break, end
    P = P + R*((fractions(k)-previous)*distance);
    P = ekf_update_covariance(P, marker(ids(k)).H, marker(ids(k)).R);
    previous = fractions(k);
end
Pnext = P + R*((endpoint-previous)*distance);
Pnext = (Pnext+Pnext.')/2;
end
