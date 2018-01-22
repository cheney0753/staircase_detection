function uvdPlane= uvdPlaneExpand(plane)
normal=sqrt(1/(1+plane(1)^2+plane(2)^2));
uvdPlane=[plane(1) plane(2) -1 plane(3)].*normal;
end