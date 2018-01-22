function [groundPlane, NumSupporters]=trackGP(GPCandidate_xyz,uvdPoints, threshold)
global intrinsics baseline;

GPCandidate_uvd=uvdPlane2(GPCandidate_xyz, intrinsics, baseline);

uvdPointsReshaped(:,1) = reshape(uvdPoints(:,:,1),1,[]);
uvdPointsReshaped(:,2) = reshape(uvdPoints(:,:,2),1,[]);
uvdPointsReshaped(:,3) = reshape(uvdPoints(:,:,3),1,[]);


d_gp=GPCandidate_uvd(1)*uvdPoints(:,:,1)+GPCandidate_uvd(2)*uvdPoints(:,:,2)+GPCandidate_uvd(3);

ROIMask=abs(uvdPoints(:,:,3)-d_gp)<threshold&uvdPoints(:,:,3)>5;
ROIMask= reshape(ROIMask,[],1);
uvdPPf(:,1)=uvdPointsReshaped(:,1).*ROIMask;
uvdPPf(:,2)=uvdPointsReshaped(:,2).*ROIMask;
uvdPPf(:,3)=uvdPointsReshaped(:,3).*ROIMask;

uvdPPf=uvdPPf(uvdPPf(:,3)~=0,:);

H= [uvdPPf(:,1),uvdPPf(:,2),ones(size(uvdPPf,1),1)];
uvdPlaneLQ = inv(H'*H)*H'*uvdPPf(:,3);

groundPlane=xyzPlane2(uvdPlaneLQ,intrinsics, baseline);
NumSupporters=size(uvdPPf,1);

end