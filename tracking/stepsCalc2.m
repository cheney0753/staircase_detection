function Num_inliers= stepsCalc2( ROIMask, uvdPoints, xyzGP, intrinsics, baseline, stp, eps, NumStairPlane)
% difference between stepsCalc2 and stepsCalc: ROIMask vs ROI
global topHeight;
xyzSC=xyzGP;%xyzStepCandidate
uvdGP=uvdPlane2(xyzGP,intrinsics,baseline);


%ROI=round(ROI);
uvdROI(:,1)=reshape(uvdPoints(:,:,1).*ROIMask,1,[]);
uvdROI(:,2)=reshape(uvdPoints(:,:,2).*ROIMask,1,[]);
uvdROI(:,3)=reshape(uvdPoints(:,:,3).*ROIMask,1,[]);
uvdROI2=uvdROI(uvdROI(:,3)~=0,:);
normal_uvdSC3=sqrt(1/(1+uvdGP(1)^2+uvdGP(2)^2));

NumSteps=(xyzGP(4)+topHeight)/stp;

for i=1:round(NumSteps)
xyzSC=[xyzSC(1:3),i*stp-topHeight];
uvdSC=uvdPlane2(xyzSC,intrinsics,baseline);
normal_uvdSC=[uvdSC(1) uvdSC(2) -1 uvdSC(3)].*normal_uvdSC3;
dist=abs(uvdROI2*normal_uvdSC(1:3)'+normal_uvdSC(4));
Num_inliers(i)=sum(dist<eps);
min(dist);
end
figure(100);
plot(1:round(NumSteps), Num_inliers);

end