function Num_inliers= stepsCalc( ROI, uvdPoints, xyzGP, intrinsics, baseline, stp, eps, NumStairPlane)

xyzSC=xyzGP;%xyzStepCandidate
uvdGP=uvdPlane2(xyzGP,intrinsics,baseline);
sz=size(uvdPoints);

%ROI=round(ROI);
planeMask=zeros(sz(1:2));
planeMask(ROI(2):ROI(2)+ROI(4),ROI(1):ROI(1)+ROI(3)) = 1;
uvdROI(:,1)=reshape(uvdPoints(:,:,1).*planeMask,1,[]);
uvdROI(:,2)=reshape(uvdPoints(:,:,2).*planeMask,1,[]);
uvdROI(:,3)=reshape(uvdPoints(:,:,3).*planeMask,1,[]);
uvdROI2=uvdROI(uvdROI(:,3)~=0,:);
blue = cat(3, zeros(sz(1:2)), zeros(sz(1:2)), ones(sz(1:2)));
normal_uvdSC3=sqrt(1/(1+uvdGP(1)^2+uvdGP(2)^2));
for i=1:round(xyzGP(4)/stp)
xyzSC=[xyzSC(1:3),xyzSC(4)-stp];
uvdSC=uvdPlane2(xyzSC,intrinsics,baseline);
%planeDraw(uvdSC,4+10*(NumStairPlane), uvdPoints ,0.1, blue);
normal_uvdSC=[uvdSC(1) uvdSC(2) -1 uvdSC(3)].*normal_uvdSC3;
dist=abs(uvdROI2*normal_uvdSC(1:3)'+normal_uvdSC(4));
Num_inliers(i)=sum(dist<eps);
min(dist);
end
figure(100);
plot(1:round(xyzGP(4)/stp), Num_inliers);

end