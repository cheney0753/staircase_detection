function [bestPlane, SupportersID] = fitStairPlanexyz(points,edgeCandidates,epsilon,trials,intrinsics,baseline)
Num_inliersMax = 0;
Num_inliers=0;
loopNO=1;  

PointsOnEdge=zeros(0,3);    
for n=1:length(edgeCandidates)
    edgePoints{n}=zeros(0,3);
    for m=1:length(edgeCandidates(n).vPts_int')
    pt=reshape(points(edgeCandidates(n).vPts_int(2,m),edgeCandidates(n).vPts_int(1,m),:),1,3);
    %dPt=get3DPoint(pt, intrinsics, baseline );
    edgePoints{n}(m,:)=pt(1:3);
    end
    PointsOnEdge=cat(1,PointsOnEdge,edgePoints{n}(edgePoints{n}(:,3)>1e-10,:));
end

if length(PointsOnEdge)<4
    fprintf ('Not enought valid points on the edges');
end

while (length(PointsOnEdge)>=4)

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
      
      Num_inliers=sum(abs(PointsOnEdge*planeCandidate(1:3)'+planeCandidate(4))<epsilon);
        if Num_inliers>Num_inliersMax
            Num_inliersMax=Num_inliers;
            bestPlane=planeCandidate;
        end
    end
    
    if (loopNO>trials||Num_inliers/length(PointsOnEdge)>0.95)
            break;
    end
    loopNO=loopNO+1;
end

if Num_inliersMax==0
    bestPlane=zeros(3,1);
    SupportersID=zeros(0,1);
else
    k=1;SupportersID=zeros(0,0);
    for n=1:length(edgeCandidates)
        NumInEdge=sum(abs(edgePoints{n}(edgePoints{n}(:,3)>1e-10,:)*bestPlane(1:3)'+bestPlane(4))<epsilon);
        if NumInEdge/length(edgePoints{n}(edgePoints{n}(:,3)>1e-10,:))>0.7;
          SupportersID(k)=n;
          k=k+1;
        end    
    end
    %bestPlane = [bestPlane(1:2)'./-bestPlane(3); bestPlane(4)/-bestPlane(3)];
end  
end