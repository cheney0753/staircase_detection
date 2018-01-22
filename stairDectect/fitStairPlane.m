function [bestPlane, SupportersID] = fitStairPlane(points,edgeCandidates,epsilon,trials)

%global intrinsics baseline gp;

numSuppMax = 0;
numSupp=0;
loopNO=1;    

while (true)
        
    %randomly select two edgelets
        i=ceil(rand(1)*length(edgeCandidates));
        j=i;
        while(j==i)
            j=ceil(rand(1)*length(edgeCandidates));
        end
        
        size_pt=size(points);
        size_pt=size_pt(1:2);
        
        p=0; %select one central points on a line and two end points on a line for plane fitting
         
        candidate=zeros(3,1);
        planePoints=zeros(3,3);
        
        p1=round([edgeCandidates(i).vCt_un(2),edgeCandidates(i).vCt_un(1)]);
        p1=limit2(p1,size_pt);
        candidate = points(p1(1),p1(2),:);
        if candidate > 1e-10
            planePoints(:,1) =candidate;
            p=p+1;
        end
        
        p2=round([edgeCandidates(j).vPointUn1(2),edgeCandidates(j).vPointUn1(1)]);
        p2=limit2(p2,size_pt);
        candidate= points(p2(1),p2(2),:);
        if candidate > 1e-10
            planePoints(:,2) =candidate;
            p=p+1;
        end
        
        p3=round([edgeCandidates(j).vPointUn2(2),edgeCandidates(j).vPointUn2(1)]);
        p3=limit2(p3,size_pt);
        candidate= points(p3(1),p3(2),:);
        if candidate > 1e-10
            planePoints(:,3) = candidate;
            p=p+1;
        end
        
        if p>2;
           planeCandidate = fitPlaneTo3Points(planePoints(:,1),planePoints(:,2),planePoints(:,3));
           if sum(planeCandidate == [0 0 0 0]) ~= 4
              uvdPlaneCandidate = [planeCandidate(1:2)'./-planeCandidate(3); planeCandidate(4)/-planeCandidate(3)];
              %normal_uvdSC=[uvdPlaneCandidate(1) uvdPlaneCandidate(2) 1 uvdPlaneCandidate(3)].*sqrt(1/(1+uvdPlaneCandidate(1)^2+uvdPlaneCandidate(2)^2));
              k=1;
              edgeSupporters=zeros(0,0);
            for n=1:length(edgeCandidates)
                NumP=ceil(length(edgeCandidates(n).vPts_int)* rand([1,20]));
                pnt=[edgeCandidates(n).vPts_int(2,NumP)',edgeCandidates(n).vPts_int(1,NumP)'];
                
                pointsOnEdge=zeros(length(pnt),3);
                
                for u=1:length(pnt)
                pointsOnEdge(u,:)=points(pnt(u,1),pnt(u,2),:);
                end
                dist_avg=sum(abs(pointsOnEdge(pointsOnEdge(:,3)~=0,:)*planeCandidate(1:3)'+planeCandidate(4)))/length(pointsOnEdge(:,3)~=0);
                if dist_avg<epsilon;
                    edgeSupporters(k)=n;
                    k=k+1;
                end    
            end
            numSupp=length(edgeSupporters);
            end
        else
            numSupp=0;
            
        end
        if numSupp>numSuppMax
        numSuppMax=numSupp;
        bestPlane=uvdPlaneCandidate;
        SupportersID=edgeSupporters;
        end
        if numSuppMax==0;
           bestPlane=zeros(3,1);
           SupportersID=zeros(1,1);
        end
        
        if loopNO>trials
            break;
        end
        loopNO=loopNO+1;
end
end

function [p]=limit2(p_,size_m)
p=p_;
if p_(1)>size_m(1); p(1)=size_m(1);end
if p_(1)<1; p(1)=1; end
if p_(2)>size_m(2); p(2)=size_m(2); end
if p_(2)<1; p(2)=1; end
    
end


