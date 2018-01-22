function [Px, nx]=xyzLine(P0, u0, intrinsics, baseline)

Px1=get3DPoint(P0, intrinsics, baseline);
Px=Px1(1:3);
n_temp1=get3DPoint(P0+u0, intrinsics, baseline);
n_temp=n_temp1(1:3)-Px;
nx=n_temp/norm(n_temp);

end