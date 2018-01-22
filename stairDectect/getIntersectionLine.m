function [P0, u]=getIntersectionLine(Plane1, Plane2)


u=cross( Plane1(1:3), Plane2(1:3));
P0=[0 Plane2(4)*Plane1(3)-Plane1(4)*Plane2(3) -Plane1(2)*Plane2(4)+Plane2(2)*Plane1(4)]./u(1);


end