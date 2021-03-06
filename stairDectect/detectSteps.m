function [stepHeight, stepNormal, isObejectDetected, stepPlaneCandidate, bBox, Numpeak]=detectSteps(xyzGP,uvdPoints, ROIMask, eps , stairPlane,  figureHandle)
global intrinsics baseline EPS_stepPlaneRANSAC colorSet topHeight;

stepHeight=0;
isObejectDetected=false;
stepNormal=zeros(1,3);
bBox=zeros(0,4);
stepPlaneCandidate=[];
Numpeak=[];

ROI=[1 1 639 479];
uvdROI(:,1)=reshape(uvdPoints(:,:,1).*ROIMask,1,[]);
uvdROI(:,2)=reshape(uvdPoints(:,:,2).*ROIMask,1,[]);
uvdROI(:,3)=reshape(uvdPoints(:,:,3).*ROIMask,1,[]);

Num_i=stepsCalc2(ROIMask,uvdPoints, xyzGP, intrinsics, baseline, 0.005, 0.008, '');

[pks, locs] = findpeaks(Num_i, 'MINPEAKHEIGHT',50, 'MINPEAKDISTANCE', 0.10/0.005);

if length(locs)<3; 
    fprintf('No step detected');
    return; 
else
[step_locs,stepHeight, stepID]=findinterval2(0.005.* locs-topHeight);
%stepPlanes=drawStepPlanes(imLeftd, ROI,uvdPoints, step_locs, xyzGP, baseline, intrinsics, 0.3);
Numpeak=pks(stepID);
end

figure(figureHandle);

numMaskPoints=zeros(length(step_locs));
stepMasks=zeros(480, 640, 0);
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
    
    numMaskPoints(i)=sum(sum(StepMask));
    step_centroid1=sum(uvdPP(:,1))/numMaskPoints(i);
    step_centroid2=sum(uvdPP(:,2))/numMaskPoints(i);
    step_halfHeight=sum(abs(uvdPP(uvdPP(:,1)>0,1)-step_centroid1))/numMaskPoints(i);
    step_halfWidth=sum(abs(uvdPP(uvdPP(:,1)>0,2)-step_centroid2))/numMaskPoints(i);
    
    step_Bbox=[step_centroid1-step_halfHeight step_centroid2-step_halfWidth...
        step_halfHeight*2 step_halfWidth*2];
    
    bBox(i,:)=step_Bbox;
    %stepMasks(:,:,i)=StepMask; 
    
    
       hold on;
     maskColor = imshow(colorSet{rem(i,5)+1});
     set(maskColor, 'AlphaData', StepMask.*0.5)
     
end

weight_i=numMaskPoints/sum(numMaskPoints);
    


figure(3);
plot(1:length(stepPlaneCandidate(:,4)'), stepPlaneCandidate(:,4)');
%draw the step edge


P_step=zeros(0,3);
u_step=zeros(0,3);


for j=1:length(stepPlaneCandidate(:,1))
    stepxyzNormal=stepPlaneCandidate(j,1:3);
    if (stepxyzNormal(1:3)*xyzGP(1:3)')>0.99;
        stepNormal=stepNormal+weight_i(j)*stepPlaneCandidate(j,1:3);
        %get the intersection line of step plane and stair plane
        [P_step(j,:), u_step(j,:)]=getIntersectionLine(stepPlaneCandidate(j,:), stairPlane);
        
    end
end


stepNormal=stepNormal./norm(stepNormal);
%edgePoints1=[ROI(1):1:ROI(1)+ROI(3)];
for s=1:size(P_step,1)
p1=getuvdPoint(P_step(s,:),intrinsics,baseline);
p2=getuvdPoint(P_step(s,:)+u_step(s,:),intrinsics,baseline);
u1=(p2-p1)./norm(p2-p1);
[pL, pR]=draw3DLine2(p1,u1,ROI, figureHandle);
%rectangle('Position',bBox(s,:),'LineWidth',1.5,'EdgeColor', [1 0 1]);
%edgePoints2=(pR(2)-pL(2))/(pR(1)-pL(1)).*(edgePoints1-pL(1))+pL(2);

%IntersectionEdges(s).vPts_un=cat(1,edgePoints1,edgePoints2);
end

isObejectDetected=true;
end 