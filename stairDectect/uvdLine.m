function [Pu, nu]=uvdLine(Px, nx)
Px=intrinsics*Pu';
n_temp=intrinsics*(Px+nx)'-Px;
nu=n_temp/norm(n_temp);

end