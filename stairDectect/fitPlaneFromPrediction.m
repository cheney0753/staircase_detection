function [xyzStairPlane, stairsEdges] = fitPlaneFromPrediction(xyzPoints, uvdPoints, edgeCandidates,predictedPlane,uvdGP,epsilon)

global intrinsics baseline;

xyzStairPlane=[];
stairsEdges=[];
predictedPlaneuvd=uvdPlane2(predictedPlane, intrinsics, baseline);
predictedPlaneuvd=uvdPlaneExpand(predictedPlaneuvd);
%rewrite the points (x y z) into a 3*N matrix edgePoints
uvdGP=uvdPlaneExpand(uvdGP);
disparityThreshold=10;
proportionLimit=0.5;
idValidEdgelets=zeros(0,0);
for n=1:length(edgeCandidates)
    
    
    
    edgePointsuvd{n}=zeros(0,3);
    for m=1:size(edgeCandidates(n).vPts_int,2)
    edgePointsuvd{n}(m,:)=reshape(uvdPoints(edgeCandidates(n).vPts_int(2,m),edgeCandidates(n).vPts_int(1,m),:),1,3);   
    end
    
    validID=edgePointsuvd{n}(:,3)>1e-10;
    
    
    % reject the edgelets on the ground plane
    dist2gp=abs(edgePointsuvd{n}(validID,:)*uvdGP(1:3)'+uvdGP(4));
    NumInEdge2gp=sum(dist2gp<epsilon);
    NumEdgePts=size(edgePointsuvd{n}(validID,:),1);
    isOnGP=0;
    if NumInEdge2gp/NumEdgePts >proportionLimit 
        isOnGP=1;
    end
    
    % reject the edgelets that have very small disparity data
    isDisp2Small=0;
    NumSmallDisp=sum(edgePointsuvd{n}(validID,3)<disparityThreshold);
    if NumSmallDisp/size(edgePointsuvd{n}(validID,:),1) >proportionLimit
        isDisp2Small=1;
    end
        
     
    if ~(isOnGP||isDisp2Small)
    idValidEdgelets=[idValidEdgelets n];
    end
    
    
end


    edgePointsuvd2=edgePointsuvd(idValidEdgelets);




for lts=1:8
k=1;    
SupportersID=zeros(0,0);
PointsOnEdge=zeros(0,3);
distArr=zeros(0,0);


for n=1:length(edgePointsuvd2);
    % see whether the number of inliers (to the stairPlane) is more than 80
    % percent of that points one the edgelet. If yes, select out as a
    % support edgelets.
    validID=edgePointsuvd2{n}(:,3)>1e-10;
    %d_plane = predictedPlaneuvd(1)*edgePointsuvd{n}(validID,1)+predictedPlaneuvd(2)*edgePointsuvd{n}(validID,2)+predictedPlaneuvd(3);
    dist=abs(edgePointsuvd2{n}(validID,:)*predictedPlaneuvd(1:3)'+predictedPlaneuvd(4));
    %dist=abs(edgePointsuvd{n}(validID,3)-d_plane);
   
    
    NumInEdge=sum(dist<epsilon);
    
    
    NumEdgePts=length(edgePointsuvd2{n}(validID,:));
    if NumInEdge/NumEdgePts>0.8 %&& NumInEdge2gp/NumEdgePts< 0.5;
        SupportersID(k)=n;
        distArr(k)=sum(dist/NumEdgePts);
        
        k=k+1;
    end
end

[inls , inlID]= sort(distArr);
id=zeros(0,0);
idk=1;
for l=1:ceil(length( distArr)*0.7) 
    id(idk)=SupportersID(inlID(l));
    PointsOnEdge=cat(1,PointsOnEdge,edgePointsuvd2{id(idk)}(edgePointsuvd2{id(idk)}(:,3)>1e-10,:));
    idk=idk+1;
end
% LQ fit the plane with the edgelets
H= [PointsOnEdge(:,1),PointsOnEdge(:,2),ones(size(PointsOnEdge,1),1)];
Plane1 = inv(H'*H)*H'*PointsOnEdge(:,3);
%xyzStairPlane=uvdPlaneExpand(Plane1);
LQPlane=xyzPlane2(Plane1,intrinsics, baseline);
xyzStairPlane=LQPlane;
if norm(cross(LQPlane(1:3),predictedPlane(1:3)))<1e-4% && abs(LQPlane(4)- predictedPlane(4))<1e-3
    
    break;
else
    predictedPlane=LQPlane;
    stairsEdges=edgeCandidates(idValidEdgelets(id));
end


end
end