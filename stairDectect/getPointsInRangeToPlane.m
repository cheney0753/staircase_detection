% range = [minDistGP maxDistGP maxZ]

function [isPlanePoint] = getPointsInRangeToPlane(points3d,planeCandidate,roi,range,subsample)


if nargin < 5, subsample = 1; end

if numel(range)==2
    range(3) = 1000;
end

isPlanePoint = boolean(zeros(size(points3d,1),size(points3d,2)));

x = points3d(1:subsample:end,1:subsample:end,1);
y = points3d(1:subsample:end,1:subsample:end,2);
z = points3d(1:subsample:end,1:subsample:end,3);

zeroPoints = (x + y + z) == 0; % ignore points in origin! (points masked)

distanceField = x*planeCandidate(1)+y*planeCandidate(2)+z*planeCandidate(3) + planeCandidate(4);

isPlanePoint = ~zeroPoints & distanceField < range(2) & distanceField > range(1) & z < range(3); % distance field

end

%maxDistanceFlex (float maxDistanceNorm, float z) { return maxDistanceNorm*(z>1000.0 ? z/1000 : 1.0); }
