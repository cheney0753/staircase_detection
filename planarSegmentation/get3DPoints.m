function p = get3DPoints(disparity, intrinsics, baseline)

% calculate 3d points
    
offsetx = baseline*intrinsics(1,3);
offsety = baseline*intrinsics(2,3);
scalez = baseline*intrinsics(1,1);

x = repmat([1:640],480,1);
y = repmat([1:480]',1,640);


validDispElements = disparity > 1e-4;

%invalidDispIdx = find(disparity < 1e-10);
%[z,s] = ind2sub(size(disparity),invalidDispIdx);

% 3D points
p(:,:,1) = ((baseline*x-offsetx)./disparity ) .* validDispElements;
p(:,:,2) = ((baseline*y-offsety)./disparity ) .* validDispElements;
p(:,:,3) = (scalez./disparity) .* validDispElements;


end


%{

for (int v=0; v<rows; v+=subsample) {
    for (int u=0; u<cols; u+=subsample) {
      d = disparities(v,u);
      z = scalez/d;
      if (d>1e-10) {
        map(v,u)[0]=(baseline*u-offsetx)/d;
        map(v,u)[1]=(baseline*v-offsety)/d;
        map(v,u)[2]=z;
      }
    }
}
%}

