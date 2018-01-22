function   stairPlot3(stairs, xyzPoints,GP,baseline, intrinsics, ROI, figureHandle)
step=2;
h1=figure(9); 
plot3(xyzPoints(1:step:end,1:step:end,1),xyzPoints(1:step:end,1:step:end,2),xyzPoints(1:step:end,1:step:end,3),'r.','MarkerSize',5)
axis([-2 2 -2 5]);
hold on; axis equal;
%plotPlane(GP,h1,[-2:0.05:2],[-2:0.5:5],[-2:0.5:10]);
[X, Y]=meshgrid(-2:0.05:2, -2:0.05:5);
[X1,Y1]=meshgrid(-2:0.02:2, -2:0.02:5);
zGP=-(GP(1).*X+GP(2).*Y+GP(4))./GP(3);
%figure (figureHandle);
%plot3(X, Y, zGP, 'r.'); hold on;


uvdGP2=uvdPlane2(GP,intrinsics,baseline);
uvdGP=uvdPlaneExpand(uvdGP2);
uvdSP=uvdPlaneExpand(stairs.uvdPlane);
u1=cross( uvdSP(1:3), uvdGP(1:3));
P0=[0 uvdGP(4)*uvdSP(3)-uvdSP(4)*uvdGP(3) -uvdSP(2)*uvdGP(4)+uvdGP(2)*uvdSP(4)]./u1(1);
s=(ROI(1):(ROI(1)+ROI(3))-P0(1))./u1(1);
pt=ones(length(s),1)*P0+s'*u1;
pt=pt(pt(:,2)>0&pt(:,2)<480,:);


P0_xyzBot=get3DPoint(P0,intrinsics,baseline);
P1_xyzStr=get3DPoint(pt(1,:),intrinsics, baseline);
P2_xyzStr=get3DPoint(pt(length(pt),:),intrinsics, baseline);
u_xyzBot=P1_xyzStr(1:3)-P2_xyzStr(1:3);
u_xyzBot=u_xyzBot./norm(u_xyzBot(1:3));

n_xyzStr=cross(stairs.xyzPlane(1:3), u_xyzBot(1:3));
n_xyzStr=n_xyzStr./norm(n_xyzStr);
%P1_xyzStr=v1(1,:);
%p2_xyzStr=v1(length(v1),:);
s1_Str=(0:0.1:2)./n_xyzStr(2);
%s2_Str=(P2_xyzStr(2):0.1:(P2_xyzStr(2)+2))./n_xyzStr(2);

pt1_xyzStr=ones(length(s1_Str),1)*P1_xyzStr(1:3)+s1_Str'*n_xyzStr;
pt2_xyzStr=ones(length(s1_Str),1)*P2_xyzStr(1:3)+s1_Str'*n_xyzStr;

plot3(pt1_xyzStr(:,1),pt1_xyzStr(:,2),pt1_xyzStr(:,3),'b');
hold on;
plot3(pt2_xyzStr(:,1),pt2_xyzStr(:,2),pt2_xyzStr(:,3),'b');
hold on;
plot3([P1_xyzStr(1) P2_xyzStr(1)],[P1_xyzStr(2) P2_xyzStr(2)],[P1_xyzStr(3) P2_xyzStr(3)],  'b');

hold on;
zSP=-(stairs.xyzPlane(1).*X+stairs.xyzPlane(2).*Y+stairs.xyzPlane(4))./stairs.xyzPlane(3);
maskSP=X>P1_xyzStr(1)&X<P2_xyzStr(1)&Y<P1_xyzStr(2);
%plot3(X.*maskSP, Y.*maskSP, zSP.*maskSP, 'r.');

n_xyzStr_Proj=[n_xyzStr(1:2) 0];
n_xyzStr_Proj=n_xyzStr_Proj/norm(n_xyzStr_Proj);
for i=1:length(stairs.stepPlanes)
    xyzStepP=xyzPlane2([GP(1:3) GP(4)-i*stairs.stepHeight],intrinsics,baseline);
    zStepP=-(xyzStepP(1).*X1+xyzStepP(2).*Y1+xyzStepP(4))./xyzStepP(3);
    stepwidth=(i*stairs.stepHeight/tan(stairs.inclination/180*pi).*n_xyzStr)*n_xyzStr_Proj';
    stepwidth2=((i+1)*stairs.stepHeight/tan(stairs.inclination/180*pi).*n_xyzStr)*n_xyzStr_Proj';
    stepheight=(i*stairs.stepHeight/tan(stairs.inclination/180*pi).*n_xyzStr)*[0 0 -1]';
    maskStepP=X1>P1_xyzStr(1)&X1<P2_xyzStr(1)&Y1>P1_xyzStr(2)-stepwidth2&Y1<P1_xyzStr(2)-stepwidth;
    plot3(X1.*maskStepP, Y1.*maskStepP, zStepP.*maskStepP, 'r.');
    %plot3([P1_xyzStr(1) P2_xyzStr(1)],[P1_xyzStr(2)+i*stepwidth P2_xyzStr(2)+i*stepwidth],[P1_xyzStr(3)+i*stepheight P2_xyzStr(3)+i*stepheight],  'b');
end
    %u_stairBotLine=cross(GP(1:3), stairs.xyzPlane(1:3));
%P0_stairBotLine=[0 GP(4)*stairs.xyzPlane(3)-stairs.xyzPlane(4)*GP(3) -stairs.xyzPlane(2)*GP(4)+GP(2)*stairs.xyzPlane(4)]./u_stairBotLine(1);

end 