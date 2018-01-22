function StairsMerged=mergeStairplane(uvdPoints,Stairs,eps, vsEdgesLeft, intrinsics, baseline, xyzGP)
% This function merges the stairplanes that belong to the same stair. 

for n=1:length(Stairs)
   d_uvdPlane{n}=Stairs(n).uvdPlane(1)*uvdPoints(:,:,1)+Stairs(n).uvdPlane(2)*uvdPoints(:,:,2)+Stairs(n).uvdPlane(3);
   planeMask{n}=abs(uvdPoints(:,:,3) - d_uvdPlane{n}) < 3*eps;
end

for n=1:length(Stairs)
    if n>length(Stairs); break; end
    for m=n+1:length(Stairs)
        if m>length(Stairs); break; end
        if abs(Stairs(n).xyzPlane(4)-Stairs(m).xyzPlane(4))<0.4
            if sum(sum(planeMask{m}&planeMask{n}))/sum(sum(planeMask{n}))>0.6
            Stairs(n).suppEdgelets=[Stairs(n).suppEdgelets Stairs(m).suppEdgelets];
            Stairs(n).uvdPlane=LQPlaneFit(uvdPoints,vsEdgesLeft( Stairs(n).suppEdgelets));
            Stairs(n).xyzPlane= xyzPlane2(Stairs(n).uvdPlane, intrinsics, baseline);
            Stairs(n).inclination =  180/pi * acos(dot(Stairs(n).xyzPlane(1:3),xyzGP(1:3)));
            Stairs(m)=[];
            end
        end        
    end
end

StairsMerged=Stairs;

end
