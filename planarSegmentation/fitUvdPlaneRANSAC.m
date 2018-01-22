% roi: [x,y,w,h];

function [bestPlane,bestPlanePoints] = fitUvdPlaneRANSAC(points,roi,trials, range)

global intrinsics baseline gp;

planePoints = zeros(3,3);
numSupportersMax = 0;

imgSize = size(points);

for i=1:trials
    
    while(true)
    p=1;
    while(p <= 3)
        u=0;
        v=0;
        while (u < roi(1) || u > roi(1)+roi(3))
            u = ceil(rand(1)*imgSize(2));
        end
        while(v < roi(2) || v > roi(2)+roi(4))        
            v = ceil(rand(1)*imgSize(1));
        end

        candidate = points(v,u,:);
        
        if candidate(3) > 1e-10
       % if points(v,u,3) > 1e-10
       %     planePoints(1,p) = u;
       %     planePoints(2,p) = v;
       %     planePoints(3,p) = points(v,u,3);
            planePoints(1:3,p) = candidate;
            p = p+1;
        end
        
    end
    
    planeCandidate = fitPlaneTo3Points(planePoints(:,1),planePoints(:,2),planePoints(:,3));
    
    if sum(planeCandidate == [0 0 0 0]) ~= 4
    
        uvdPlaneCandidate = [planeCandidate(1:2)'./-planeCandidate(3); planeCandidate(4)/-planeCandidate(3)];
    
        % is it properly aligned to the floor?
        xyzPlaneCandidate = xyzPlane2(uvdPlaneCandidate,intrinsics,baseline);
        
        a =  180/pi * acos(dot(xyzPlaneCandidate(1:3),gp(1:3)));
        
        if a > 80 & a < 100 % valid
            break;
        end
        
    end
    end
    
    

    d_uvdPlaneCandidate = uvdPlaneCandidate(1)*points(:,:,1)+uvdPlaneCandidate(2)*points(:,:,2)+uvdPlaneCandidate(3);

    planeMaskCandidate = abs(points(:,:,3) - d_uvdPlaneCandidate) < 0.3 & points(:,:,3) > 0.01;
    
    numSupporters = sum(sum(planeMaskCandidate));
    
    if(numSupporters > numSupportersMax)
            bestPlane = uvdPlaneCandidate;
            bestPlanePoints = planeMaskCandidate;
            numSupportersMax = numSupporters;
            %planePoints
        end

end

end