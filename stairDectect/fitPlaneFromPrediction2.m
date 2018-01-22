function [xyzStairPlane, stairsEdges, supportPoints] = fitPlaneFromPrediction2(xyzPoints, uvdPoints, edgeCandidates,predictedPlane,uvdGP,epsilon)

global intrinsics baseline;

xyzStairPlane=[];
stairsEdges=[];
predictedPlaneuvd=uvdPlane2(predictedPlane, intrinsics, baseline);
predictedPlaneuvd=uvdPlaneExpand(predictedPlaneuvd);
%rewrite the points (x y z) into a 3*N matrix edgePoints
uvdGP4=uvdPlaneExpand(uvdGP);
disparityThreshold=10;
proportionLimit=0.8;
for n=1:length(edgeCandidates)
    
    
    
    edgePointsuvd{n}=zeros(0,3);
    for m=1:size(edgeCandidates(n).vPts_int,2)
    edgePointsuvd{n}(m,:)=reshape(uvdPoints(edgeCandidates(n).vPts_int(2,m),edgeCandidates(n).vPts_int(1,m),:),1,3);   
    end

end

    


firstPercentage=0.8;
for lts=1:8


allPointsOnEdge=zeros(0,3);
allDist=zeros(0,0);
for n=1:length(edgePointsuvd);
    
    % reject the points with small disparity value
    validID=edgePointsuvd{n}(:,3)>disparityThreshold;
    
    dist2gp=abs(edgePointsuvd{n}(:,:)*uvdGP4(1:3)'+uvdGP4(4));
    %d_plane = predictedPlaneuvd(1)*edgePointsuvd{n}(validID,1)+predictedPlaneuvd(2)*edgePointsuvd{n}(validID,2)+predictedPlaneuvd(3);
    dist=abs(edgePointsuvd{n}(:,:)*predictedPlaneuvd(1:3)'+predictedPlaneuvd(4));
    %dist=abs(edgePointsuvd{n}(validID,3)-d_plane);
   

    validPointsOneEdge=dist<epsilon&validID&dist2gp>epsilon;
    allPointsOnEdge=cat(1,allPointsOnEdge, edgePointsuvd{n}(validPointsOneEdge,:));
    allDist=cat(1, allDist, dist(validPointsOneEdge,:));
    NumInEdge=sum(validPointsOneEdge);
    NumEdgePts=size(edgePointsuvd{n}(:,:),1);

end

[inls , inlID]= sort(allDist);
inliers=1:ceil(firstPercentage*length(inlID));
LQPoints=allPointsOnEdge(inlID(inliers),:);
% LQ fit the plane with the edgelets
H= [LQPoints(:,1),LQPoints(:,2),ones(size(LQPoints,1),1)];
Plane1 = inv(H'*H)*H'*LQPoints(:,3);
%xyzStairPlane=uvdPlaneExpand(Plane1);
LQPlane=xyzPlane2(Plane1,intrinsics, baseline);
xyzStairPlane=LQPlane;

supportPoints=LQPoints;
if norm(cross(LQPlane(1:3),predictedPlane(1:3)))<1e-4% && abs(LQPlane(4)- predictedPlane(4))<1e-3
    
    break;
else
    predictedPlane=LQPlane;
    stairsEdges=edgeCandidates; 
end


end
end