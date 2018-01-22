function Puvd=getuvdPoint(P3d, intrinsics, baseline)

Puvd(3)=( baseline.*intrinsics(1,1)./P3d(3));
Puvd(1)=(P3d(1).*Puvd(3)+baseline*intrinsics(1,3))/baseline;
Puvd(2)=(P3d(2).*Puvd(3)+baseline*intrinsics(2,3))/baseline;

end 