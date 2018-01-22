function [stepHeight, stepNormal]=renewStepPlane(xyzGP, stepHeightCandidate,uvdPoints, ROIMask, eps , stairPlane,  figureHandle)
stepNormal=zeros(1,3);
stepHeight=stepHeightCandidate;
stepPlaneLQ=zeros(0,4);
stepPlanexyz=zeros(0,4);
global intrinsics baseline colorSet;
xyzStepBelow=xyzGP;
uvdROI(:,1)=reshape(uvdPoints(:,:,1).*ROIMask,1,[]);
uvdROI(:,2)=reshape(uvdPoints(:,:,2).*ROIMask,1,[]);
uvdROI(:,3)=reshape(uvdPoints(:,:,3).*ROIMask,1,[]);
%uvdROI=uvdROI(uvdROI(:,3)~=0,:);
ROI=[1 1 479 639 ];


for i=1:8
    
    xyzStepBelow=[xyzStepBelow(1:3), xyzStepBelow(4)-stepHeightCandidate];
    uvdStep=uvdPlane2(xyzStepBelow, intrinsics, baseline);
    normal_uvdStep3=sqrt(1/(1+uvdStep(1)^2+uvdStep(2)^2));
    normal_uvdStep=[uvdStep(1) uvdStep(2) -1 uvdStep(3)].*normal_uvdStep3;
    dist= abs(uvdROI*normal_uvdStep(1:3)'+normal_uvdStep(4));
    
    uvdStepMask=((dist>0)&(dist<eps));
    StepMask=reshape(uvdStepMask,480,640);
    
    figure(figureHandle);
    hold on;
    maskColor = imshow(colorSet{rem(i,5)+1});
    set(maskColor, 'AlphaData', StepMask.*0.5)

    
    uvdPP(:,1) = reshape(uvdPoints(:,:,1).*StepMask,1,[]);
    uvdPP(:,2) = reshape(uvdPoints(:,:,2).*StepMask,1,[]);
    uvdPP(:,3) = reshape(uvdPoints(:,:,3).*StepMask,1,[]);
    uvdPPf = uvdPP(uvdPP(:,3)~=0,:); % filter zeros
    H= [uvdPPf(:,1),uvdPPf(:,2),ones(size(uvdPPf,1),1)];
     planeLQ= (inv(H'*H)*H'*uvdPPf(:,3))';
     stepPlaneLQ(i,:)=uvdPlaneExpand(planeLQ);
    stepPlanexyz(i,:)=xyzPlane2( stepPlaneLQ(i,:), intrinsics, baseline);
    xyzStepBelow=stepPlanexyz(i,:);
end
hold off;

P_step=zeros(0,3);
u_step=zeros(0,3);

for j=1:length(stepPlaneLQ)
    stepxyzNormal=stepPlanexyz(j,1:3);
    if (stepxyzNormal(1:3)*xyzGP(1:3)')>0.9;
    
        stepNormal=stepNormal+stepxyzNormal(1:3);
        
        %get the intersection line of step plane and stair plane
        [P_step(j,:), u_step(j,:)]=getIntersectionLine(stepPlanexyz(j,:), stairPlane);
        
    end
end



stepNormal=stepNormal./norm(stepNormal);
[step_locs,stepHeight1]=findinterval(stepPlanexyz(:,4)');
figure(3);
plot(1:8, stepPlanexyz(:,4)');
stepHeight=abs(stepHeight1);
%draw the step edge
for s=1:length(P_step)
p1=getuvdPoint(P_step(s,:),intrinsics,baseline);
p2=getuvdPoint(P_step(s,:)+u_step(s,:),intrinsics,baseline);
u1=(p2-p1)./norm(p2-p1);
draw3DLine(p1,u1,ROI, figureHandle);

end

end 