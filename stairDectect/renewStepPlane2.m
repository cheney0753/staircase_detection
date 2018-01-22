function [stepHeight, stepNormal, isObejectDetected]=renewStepPlane2(xyzGP, stepHeightCand,uvdPoints, ROIMask, eps , stairPlane,  figureHandle)
global intrinsics baseline EPS_planeRANSAC;

stepHeight=0;
isObejectDetected=false;
stepNormal=zeros(1,3);



ROI=[1 1 639 479];
uvdROI(:,1)=reshape(uvdPoints(:,:,1).*ROIMask,1,[]);
uvdROI(:,2)=reshape(uvdPoints(:,:,2).*ROIMask,1,[]);
uvdROI(:,3)=reshape(uvdPoints(:,:,3).*ROIMask,1,[]);

Num_i=stepsCalc2(ROIMask,uvdPoints, xyzGP, intrinsics, baseline, 0.005, 0.008, '');

[pks, locs] = findpeaks(Num_i, 'MINPEAKHEIGHT',70, 'MINPEAKDISTANCE', 0.10/0.005);

if length(locs)<3; 
    fprintf('No step detected');
    return; 
else
[step_locs,stepHeight]=findinterval2(0.005.* locs);
%stepPlanes=drawStepPlanes(imLeftd, ROI,uvdPoints, step_locs, xyzGP, baseline, intrinsics, 0.3);

end



for i=1:length(step_locs)
    xyzStep=[xyzGP(1:3), step_locs(i)];
    uvdStep=uvdPlane2(xyzStep, intrinsics, baseline);
    normal_uvdStep3=sqrt(1/(1+uvdStep(1)^2+uvdStep(2)^2));
    normal_uvdStep=[uvdStep(1) uvdStep(2) -1 uvdStep(3)].*normal_uvdStep3;
    dist=abs(uvdROI*normal_uvdStep(1:3)'+normal_uvdStep(4));
    uvdStepMask=((dist>0)&(dist<eps));
    StepMask=reshape(uvdStepMask,480,640);
    uvdPP(:,1) = reshape(uvdPoints(:,:,1).*StepMask,1,[]);
    uvdPP(:,2) = reshape(uvdPoints(:,:,2).*StepMask,1,[]);
    uvdPP(:,3) = reshape(uvdPoints(:,:,3).*StepMask,1,[]);
    uvdPPf = uvdPP(uvdPP(:,3)~=0,:); % filter zeros
    H= [uvdPPf(:,1),uvdPPf(:,2),ones(size(uvdPPf,1),1)];
    uvdPlaneLQ = inv(H'*H)*H'*uvdPPf(:,3);
    stepPlaneCandidate(i,:)=xyzPlane2(uvdPlaneLQ, intrinsics, baseline);
    
end


    


figure(3);
plot(1:length(stepPlaneCandidate(:,4)'), stepPlaneCandidate(:,4)');
%draw the step edge


P_step=zeros(0,3);
u_step=zeros(0,3);


for j=1:length(stepPlaneCandidate(:,1))
    stepxyzNormal=stepPlaneCandidate(j,1:3);
    if (stepxyzNormal(1:3)*xyzGP(1:3)')>0.97;
        stepNormal=stepNormal+stepPlaneCandidate(i,1:3);
        %get the intersection line of step plane and stair plane
        [P_step(j,:), u_step(j,:)]=getIntersectionLine(stepPlaneCandidate(j,:), stairPlane);
        
    end
end


stepNormal=stepNormal./norm(stepNormal);

for s=1:length(P_step)
p1=getuvdPoint(P_step(s,:),intrinsics,baseline);
p2=getuvdPoint(P_step(s,:)+u_step(s,:),intrinsics,baseline);
u1=(p2-p1)./norm(p2-p1);
draw3DLine(p1,u1,ROI, figureHandle);

end

isObejectDetected=true;
end 