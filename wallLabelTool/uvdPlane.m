
function uvd_p = uvdPlane(xyzPlane,intrinsics,baseline)

uvd_p(1) = -xyzPlane(1)*baseline/xyzPlane(4);
uvd_p(2) = -xyzPlane(2)*baseline/xyzPlane(4);
uvd_p(3) = -baseline/xyzPlane(4) * (xyzPlane(3)*intrinsics(1,1)-intrinsics(1,3)*xyzPlane(1)-intrinsics(2,3)*xyzPlane(2));

end
