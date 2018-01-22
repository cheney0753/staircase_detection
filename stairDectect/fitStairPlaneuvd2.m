function [Planes, PlaneSupports] = fitStairPlaneuvd2(points,edgeCandidates,epsilon,trials, gp, disparityThreshold)
global intrinsics baseline;
Num_inliersMax = 0;
Num_inliers=0;
IDinliersMax=[];
loopNO=1;  
uvdGP=uvdPlane2(gp, intrinsics, baseline);
uvdGP4=uvdPlaneExpand(uvdGP);

PointsOnEdge=zeros(0,3);    
for n=1:length(edgeCandidates)
    edgePoints{n}=zeros(0,3);
    for m=1:size(edgeCandidates(n).vPts_int,2)
    %dPt=get3DPoint(pt, intrinsics, baseline );
    edgePoints{n}(m,:)=reshape(points(edgeCandidates(n).vPts_int(2,m),edgeCandidates(n).vPts_int(1,m),:),1,3);
    end
    PointsOnEdge=cat(1,PointsOnEdge,edgePoints{n}(edgePoints{n}(:,3)>1e-10,:));
end

if length(PointsOnEdge)<4
    fprintf ('Not enought valid points on the edges');
end

%reject points on the ground plane
dist2gp=abs(PointsOnEdge(:,:)*uvdGP4(1:3)'+uvdGP4(4));

%reject points with vary small disparity 
 validID=dist2gp>epsilon&PointsOnEdge(:,3)>disparityThreshold;
PointsOnEdge=PointsOnEdge( validID,:);

NumTotalPoints=size(PointsOnEdge,1);
g=1;
while (true)
while (true)

    %readomly pick 3 points and fit a plane
    p1=ceil(rand(1)*length(PointsOnEdge));
    p2=p1; 
    while (p1==p2)
        p2=ceil(rand(1)*length(PointsOnEdge));
        p3=p2;
        while (p3==p2||p3==p1)
            p3=ceil(rand(1)*length(PointsOnEdge));
        end
    end
    planeCandidate = fitPlaneTo3Points(PointsOnEdge(p1,:)',PointsOnEdge(p2,:)',PointsOnEdge(p3,:)');
    
    
    if sum(planeCandidate == [0 0 0 0]) ~= 4
      
       
        dist=abs(PointsOnEdge(:,:)*planeCandidate(1:3)'+planeCandidate(4));    
        validPointsOneEdge=dist<epsilon;
        Num_inliers=sum(validPointsOneEdge);
        
        if Num_inliers>Num_inliersMax
            Num_inliersMax=Num_inliers;
            bestPlane=planeCandidate;
            IDinliersMax=validPointsOneEdge;
        end
    end
    
    if (loopNO>trials||Num_inliers/size(PointsOnEdge,1)>0.95)
            break;
    end
    loopNO=loopNO+1;
end
if isempty(IDinliersMax)
    break;
end
LQPoints=PointsOnEdge(IDinliersMax,:);
H= [LQPoints(:,1),LQPoints(:,2),ones(size(LQPoints,1),1)];
Plane1 = inv(H'*H)*H'*LQPoints(:,3);

StairPlanes(g,:)=Plane1;
PlaneSupports{g}=LQPoints;

PointsOnEdge=PointsOnEdge(~IDinliersMax,:);
NumPlanes=size(StairPlanes,1);
NumPointsLeft=size(PointsOnEdge,1);
if (NumPlanes>4||NumPointsLeft/NumTotalPoints<0.2)
    break;
end

g=g+1;
Num_inliers=0;
IDinliersMax=[];
loopNO=0;
bestPlane=[];
Num_inliersMax=0;
end
for a=1:size(StairPlanes, 1)
    Planes(a,:)=xyzPlane2(StairPlanes(a,:),intrinsics,baseline);
end
end