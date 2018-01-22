function p3d = get3DPoint(p, intrinsics, baseline)

%global intrinsics baseline;

p3d(1) = ((baseline*p(1)- baseline*intrinsics(1,3))./p(3) );
p3d(2) = ((baseline*p(2)- baseline*intrinsics(2,3))./p(3) );
p3d(3) = ( baseline*intrinsics(1,1)./p(3));
p3d(4) = 1;

end