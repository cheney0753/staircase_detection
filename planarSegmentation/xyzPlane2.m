function p = xyzPlane2(uvdPlane,intrinsics,B)


p(1) = -uvdPlane(1);
p(2) = -uvdPlane(2);
p(3) = -(uvdPlane(1)*intrinsics(1,3)+uvdPlane(2)*intrinsics(2,3)+uvdPlane(3))/intrinsics(1,1);
p(4) = B;

p = p/norm(p(1:3));

end