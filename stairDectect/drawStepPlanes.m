function stepPlanes=drawStepPlanes(imLeftd, ROI,uvdPoints, step_locs, xyzGP, baseline, intrinsics, eps)

color{1}=cat(3, ones(size(imLeftd)), zeros(size(imLeftd)), zeros(size(imLeftd)));
color{2}=cat(3, zeros(size(imLeftd)), ones(size(imLeftd)), zeros(size(imLeftd)));
color{3}=cat(3, zeros(size(imLeftd)), zeros(size(imLeftd)), ones(size(imLeftd)));
color{4}=cat(3, ones(size(imLeftd)), ones(size(imLeftd)), zeros(size(imLeftd)));
color{5}=cat(3, zeros(size(imLeftd)), ones(size(imLeftd)), ones(size(imLeftd)));

figure(333);
hold on;
imshow(imLeftd,[],'Border', 'tight');
hold on;
uvdGP=uvdPlane2(xyzGP, intrinsics, baseline);
normal_uvdStep3=sqrt(1/(1+uvdGP(1)^2+uvdGP(2)^2));

ROImask=zeros(size(imLeftd));
ROImask(ROI(2):ROI(2)+ROI(4),ROI(1):ROI(1)+ROI(3)) = 1;
uvdROI(:,1)=reshape(uvdPoints(:,:,1).*ROImask,1,[]);
uvdROI(:,2)=reshape(uvdPoints(:,:,2).*ROImask,1,[]);
uvdROI(:,3)=reshape(uvdPoints(:,:,3).*ROImask,1,[]);
uvdMask=reshape(ROImask,[],1);
for i=1:length(step_locs(:,1))
   xyzStep=[xyzGP(1:3), xyzGP(4)-step_locs(i,2)];
   uvdStep=uvdPlane2(xyzStep, intrinsics, baseline);
   normal_uvdStep=[uvdStep(1) uvdStep(2) -1 uvdStep(3)].*normal_uvdStep3;
   dist=abs(uvdROI*normal_uvdStep(1:3)'+normal_uvdStep(4).*uvdMask);
   uvdStepMask=uvdMask.*((dist>0)&(dist<eps));
   StepMask=reshape(uvdStepMask,480,640);
   maskColor = imshow(color{rem(i,5)+1});
   set(maskColor, 'AlphaData', StepMask.*0.5)
   hold on;
   
   if sum(sum(StepMask))<300 
       stepPlanes{i}=uvdStep;
   else
   uvdPP(:,1) = reshape(uvdPoints(:,:,1).*StepMask,1,[]);
   uvdPP(:,2) = reshape(uvdPoints(:,:,2).*StepMask,1,[]);
   uvdPP(:,3) = reshape(uvdPoints(:,:,3).*StepMask,1,[]);
   uvdPPf = uvdPP(uvdPP(:,3)~=0,:); % filter zeros
   H= [uvdPPf(:,1),uvdPPf(:,2),ones(size(uvdPPf,1),1)];
   uvdPlaneLQ = inv(H'*H)*H'*uvdPPf(:,3);
   stepPlanes{i}=uvdPlaneLQ;
   end
end


end