function [GPvector,stairDirtVect] =getGPfromVectors(stairPlane_xyz, inclination, VPt_uv)

global intrinsics baseline;

%stairPlane_uvd=uvdPlane2(stairPlane_xyz, intrinsics, baseline);

%vpt_d=stairPlane_uvd(1)*VPt_uv(1)+stairPlane_uvd(1)*VPt_uv(1);

%vpt_uvd=[VPt_uv vpt_d];

%vpt_xyz=get3DPoint(vpt_uvd, intrinsics, baseline); vptVect=vpt_xyz/norm(vpt_xyz);
vpt_xyz=[VPt_uv(1)-intrinsics(1,3); VPt_uv(2)-intrinsics(2,3); intrinsics(1,1)];
vptVect=vpt_xyz(1:3)/norm(vpt_xyz(1:3));
if vptVect(1)<0
    vptVect=-vptVect;
end

stairPlaneVect=stairPlane_xyz(1:3)'/norm(stairPlane_xyz(1:3)');

stairDirtVect=cross( stairPlaneVect,vptVect);

l =vptVect(1);
m=vptVect(2);
n=vptVect(3);
th=pi/2-inclination/180*pi;
rotMat=[...
        l*l*(1-cos(th))+cos(th)     m*l*(1-cos(th))-n*sin(th)   n*l*(1-cos(th))+m*sin(th) ;
        l*m*(1-cos(th))+n*sin(th)   m*m*(1-cos(th))+cos(th)     n*m*(1-cos(th))-l*sin(th);
        l*n*(1-cos(th))-m*sin(th)   m*n*(1-cos(th))+l*sin(th)   n*n*(1-cos(th))+cos(th)  ];

GPvector=rotMat*stairDirtVect;
GPvector=GPvector/norm(GPvector);
 %A*GPvector=B;
%A=cat(1, cat(1,stairPlaneVect', stairDirtVect'), vptVect');
%B=[cos(inclination); -sin(inclination); 0];
%GPvector2=inv(A)*B;
%GPvector2=GPvector2/norm(GPvector2);
end