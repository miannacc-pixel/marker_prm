function [ids, fractions] = marker_events(x0, x1, marker, sensed)
%MARKER_EVENTS Unsensed markers encountered on an edge, in travel order.
% Uses marker_encounter's closest-point measurement convention.
if nargin < 4, sensed = false(1,numel(marker)); end
ids = []; fractions = [];
for k = 1:numel(marker)
    if sensed(k), continue, end
    [hit, fraction] = marker_encounter(x0, x1, marker(k));
    if hit
        ids(end+1) = k;
        fractions(end+1) = fraction;
    end
end
% Stable tie break by marker index for overlapping measurement locations.
if ~isempty(ids)
    events = sortrows([fractions(:), ids(:)], [1 2]);
    fractions = events(:,1).'; ids = events(:,2).';
end
end
