function  uvdPlaneLQ=LQPlaneFit(uvdPoints,edgelets)


roiMask = zeros(480,640);
for i=1:length(edgelets)
    for j=1:length(edgelets(i).vPts_int)
    roiMask(edgelets(i).vPts_int(2,j),edgelets(i).vPts_int(1,j)) = 1;
    end
end
uvdPP(:,1) = reshape(uvdPoints(:,:,1).*roiMask,1,[]);
uvdPP(:,2) = reshape(uvdPoints(:,:,2).*roiMask,1,[]);
uvdPP(:,3) = reshape(uvdPoints(:,:,3).*roiMask,1,[]);

uvdPPf = uvdPP(uvdPP(:,3)~=0,:); % filter zeros

H= [uvdPPf(:,1),uvdPPf(:,2),ones(size(uvdPPf,1),1)];
uvdPlaneLQ = inv(H'*H)*H'*uvdPPf(:,3);
end