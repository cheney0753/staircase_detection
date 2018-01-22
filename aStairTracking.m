
clear
close all


%imgPath = '/mrt/storage/users/lauer/2013_07_08_Bergwald';
%intrinsics = [496.6227 0 315.2575;0 499.1608 240.0210;0 0 1.0000];
%baseline = 0.1375;
%{
imgPath = '/mrt/storage/users/schwarze/OIWOB_data/2014_6_4_kaisPizza';
intrinsics = [ 361.836 0 302.187 ; 0 361.836 231.214 ; 0 0 1 ]
baseline =  0.185368;
gp = [0.0523936,-0.998589,0.00871027,1.85122];
gp_kp=load('gp_kaisPizza.txt');
leftImgStr  = [imgPath,'/cache/stream_b09d01009e411d_1_%06d.rect.png'];
dispImgStr = [imgPath,'/cache/stream_b09d01009e411d_1_%06d.rect.dispSGBM.png']

%imgPath = '/mrt/storage/users/schwarze/OIWOB_data/2013_8_20_kellertreppe';
%intrinsics = [ 470.638 0 327.175 ; 0 470.638 223.814 ; 0 0 1 ]
%baseline = 0.137522;
%leftImgStr  = [imgPath,'/stream_left_1_%06d.png'];
%dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];


frame1st=3420;
frameNo =frame1st:5:5000;%7750;%:6400; %3520;%90;%:110;%:100; %2320;


%intrinsics = [ 468.266 0 338.843 ; 0 468.266 224.094 ; 0 0 1 ]
%baseline = 0.1396954;
%leftImgStr  = '/home/schwarze/OIWOB/images/2013_6_11_office/cache/stream_b09d01009e410e_1_%06d.rect.png';
%rightImgStr = '/home/schwarze/OIWOB/images/2013_6_11_office/cache/stream_b09d01009e411d_1_%06d.rect.png';
%frameNo = 1;

%}
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
frame1st=875;
framelast=1036;
frameNo =frame1st:1:framelast;
gp_kp=zeros(length(frameNo),4);
load('gp_durlachBahnhof760.mat');
for x=frame1st:length(groundPlane)
    if ~isempty(groundPlane{x})
        gp_kp(x-frame1st+1,:)=[x groundPlane{x}];
    end
end

fid=fopen('stepHeight.txt','w');
P_left = intrinsics * [diag([1,1,1]) [0;0;0]];
%P_right = intrinsics * [diag([1,1,1]) [-baseline;0;0]];


global EPS_houghRANSAC; EPS_houghRANSAC=1;
global EPS_planeRANSAC; EPS_planeRANSAC=0.3;

%%
ARGS = FACADE_ARGS_default();
const = FACADE_const();
ARGS.minEdgeLength     = 30;    %in pixels
ARGS.linesegTOL        = 10;%2;     %pixel tolerance when selecting straight edges
ARGS.mKinv = [];
ARGS.imgS = intrinsics(1,1);

k = 1;

hLeft  = figure('Name','Left');%('visible','off');
%hRight = figure('Name','Right');

%% configure the Kalman filter.
param.motionModel='ConstantVelocity';
initialLocation=[0.135 0];
param.initialEstimateError=[1000 1000];
param.motionNoise=[0.025 0.01];
param.measurementNoise=0.25;
kalmanFilterStepHeight = configureKalmanFilter(param.motionModel, ...
          initialLocation, param.initialEstimateError, ...
          param.motionNoise, param.measurementNoise);



for iFr=1:length(frameNo)
    fprintf('%02d',iFr);
    clear EdgesStair;
    clear IntcLines;
    clear vsEdgesLeft;
    close all;
imLeft=imread(sprintf(leftImgStr,frameNo(iFr)));
dispImg = double(imread(sprintf(dispImgStr, frameNo(iFr))));
disparity = dispImg./256;

tic
    [vsEdgesLeft,edgeImLeft] = FACADE_getEdgelets2(imLeft, ARGS);
toc
nbEdgesLeft = size(vsEdgesLeft,2);
%vsEdgesRight = [];
%[vsEdgesRight,edgeImRight] = FACADE_getEdgelets2(imRight, ARGS);
toc

% round the coordinats of edgelets into integars
for i=1:length(vsEdgesLeft)
    vsEdgesLeft(i).vPts_int=RoundUV(vsEdgesLeft(i).vPts_un,[480 640]);
end

%% find the horitzontal edges
hrzEdges=zeros(1,1);
k=1;
for i=1:length(vsEdgesLeft)
    if abs(vsEdgesLeft(1,i).theta)<45
        hrzEdges(k)=i;
        k=k+1;
    end
end
if length(hrzEdges)==0
        fprintf('Error: there is no horizontal edges');
        continue;
end
  

figure(1);
cla;
%imshow(imLeft,[]);
imshow(imLeft);
hold on;
plotEdglets(vsEdgesLeft(hrzEdges),'b');

%% do a ransac line fitting to the hough transform of edge lines
%vsEdges = [vsEdgesLeft,vsEdgesRight];
%[vsVPLeft,vClusterLeft] = FACADE_getVP_JLinkage(vsEdgesLeft, imLeft, ARGS);

% build hough space of edgelets
hough_curr = zeros(length(vsEdgesLeft(hrzEdges)),2);
for i=1:length(vsEdgesLeft(hrzEdges));
   % if abs(vsEdgesLeft(1,i).theta)>10, continue; end;
    th = -(pi/180)*(vsEdgesLeft(1,hrzEdges(i)).theta);
    hough_curr(i,1) = th;
    hough_curr(i,2) = vsEdgesLeft(1,hrzEdges(i)).vCt_un(2) * cos(th) + vsEdgesLeft(1,i).vCt_un(1) * sin(th);
    if hough_curr(i,2) < 0
        hough_curr(i,2) = -hough_curr(i,2);
        hough_curr(i,1) = hough_curr(i,1)+pi;
    end
end

% do a ransac line fit to the hough transforms of edgelets
%[f_hough,iPt_hough, Id_in_hrzEdges]=RansacLineFit([hough_curr(:,2),-180/pi*hough_curr(:,1)], EPS_houghRANSAC);
[f_hough,iPt_hough, Id_in_hrzEdges]=RansacLineFit([hough_curr(:,2)./cos(hough_curr(:,1)),tan(hough_curr(:,1))], 0.005);
%[f_hough,iPt_hough, Id_in_hrzEdges]=RansacLineFit([hough_curr(:,2),tan(hough_curr(:,1))], 0.005);

if size(iPt_hough,1)==0
    fprintf('error 1 ');  
    continue;
end

% plot the ransac fiting in hough space
distance=linspace(0,640);
angle=distance*f_hough.p1+f_hough.p2;
figure(2);
clf;
hold on;
%plot(hough_curr(:,2),-180/pi*hough_curr(:,1),'.','MarkerSize',20);
plot(hough_curr(:,2)./cos(hough_curr(:,1)),tan(hough_curr(:,1)),'.','MarkerSize',20);
%plot(hough_curr(:,2),tan(hough_curr(:,1)),'.','MarkerSize',20);

hold on;
plot(distance,angle);
hold on;
plot(iPt_hough(:,1),iPt_hough(:,2),'g.');
%h1=figure(2)
%saveas(h1,sprintf('/home/zhong/images/stairsHough%04d.png',iFr+frameNo(1)));
hold off;
%plot the filtered edges after hough ransac fitting
houghEdges=hrzEdges(Id_in_hrzEdges);    
figure(1); hold on;
plotEdglets(vsEdgesLeft(houghEdges),'g');
figure( 3);
hold on;
plotEdglets(vsEdgesLeft(houghEdges),'g');
VPt=[1/f_hough.p1 -f_hough.p2/f_hough.p1];
hold on;plot(1/f_hough.p1, -f_hough.p2/f_hough.p1,'r*', 'MarkerSize', 6);


for l=1:length(houghEdges)
    ptL=vsEdgesLeft(houghEdges(l)).vPointUn1;
    ptR=vsEdgesLeft(houghEdges(l)).vPointUn2;
    slope = (ptL(2)-ptR(2))/(ptL(1)-ptR(1));
    xleft=VPt(1);
    yleft=slope*(xleft-ptR(1))+ptR(2);
    line([ptR(1),  xleft], [ptR(2), yleft], 'Color', 'y', 'LineWidth', 1);
end
axis equal;
%% fit plane 
%{
imLeftd=double(imLeft);
u = repmat([1:640],480,1);
v = repmat([1:480]',1,640);

uvdPoints(:,:,1) = u;
uvdPoints(:,:,2) = v;
uvdPoints(:,:,3) = disparity;

xyzPoints=get3DPoints(disparity, intrinsics, baseline);

%planeMask = getPointsInRangeToPlane(xyzPoints,xyzGP,[],[-0.1 0.1]); 

step=2;
%figure(6)
%u2=360; 
%plot3(uvdPoints(150:step:u2,250:step:450,1),-uvdPoints(150:step:u2,250:step:450,2),uvdPoints(150:step:u2,250:step:450,3),'r.','MarkerSize',5)
figure(8)
plot3(uvdPoints(1:step:end,1:step:end,1),-uvdPoints(1:step:end,1:step:end,2),uvdPoints(1:step:end,1:step:end,3),'r.','MarkerSize',5)


%[X, Y]=meshgrid(-2:0.05:2, -2:0.05:5);
%[X1,Y1]=meshgrid(-2:0.02:2, -2:0.02:5);
%figure(7)
%imshow(log(disparity),[]);

SupportEdges=houghEdges;
length_SEdges=length(SupportEdges);
NumStairPlane=0;

uvdPlaneGP=gp_kp(frameNo(iFr)-frame1st,2:4)';
xyzGP=xyzPlane2(uvdPlaneGP,intrinsics,baseline);
range_Inclination=[20 50]; %the accepted range for the inclination of stair plane


stairCandidate=struct([]);

loopnum=1;
while (true)
    if length(SupportEdges)<2; break; end
    tic
    [xyzStairPlane, stairsEdges] = fitStairPlanexyz(xyzPoints,vsEdgesLeft(SupportEdges),0.03,500,intrinsics, baseline);
    %SupportEdges2=SupportEdges;
    toc
    if xyzStairPlane == zeros(1,4); break; end;
        
    stairsEdges=SupportEdges(stairsEdges) ;
    
     %replace the edgelets set with the outliers set
    Outliers=zeros(0,0);
    k=1;
    for n=1:length(SupportEdges)
        if stairsEdges(:)~=SupportEdges(n)
            Outliers(k)=SupportEdges(n);
            k=k+1;
        end
    end
    SupportEdges=Outliers;  
    
    %[xyzStairPlane2, stairsEdges2] = fitStairPlanexyz(xyzPoints,vsEdgesLeft(SupportEdges2),0.01,500,intrinsics, baseline);
    % see if the angle between the plane candidate and ground
    % plane is between 25 deg to 47 deg.
    %   uvdStairPlane=LQPlaneFit(uvdPoints,vsEdgesLeft( stairsEdges));
    %xyzStairPlane= xyzPlane2(uvdStairPlane, intrinsics, baseline);
    uvdStairPlane=uvdPlane2( xyzStairPlane,intrinsics,baseline);
    inclination =  180/pi * acos(dot(xyzStairPlane(1:3),xyzGP(1:3)));
    if inclination>range_Inclination(2)||inclination<range_Inclination(1)
        fprintf('Stair plane not found: inclination too big.');
        continue;
    else
        if length(stairsEdges)<4
            fprintf('Stair plane not found: not enough edgelets.');
            continue;
        else
        NumStairPlane=NumStairPlane+1;
        stairCandidate(NumStairPlane).suppEdgelets=stairsEdges;
        stairCandidate(NumStairPlane).xyzPlane= xyzStairPlane;
        stairCandidate(NumStairPlane).uvdPlane= uvdStairPlane;
        stairCandidate(NumStairPlane).inclination=inclination;
        [ROI, stairCandidate(NumStairPlane).center]= rect_Stair(vsEdgesLeft(stairCandidate(NumStairPlane).suppEdgelets));
        stairCandidate(NumStairPlane).ROI=ROI;
        d_uvdPlane = uvdStairPlane(1)*uvdPoints(:,:,1)+uvdStairPlane(2)*uvdPoints(:,:,2)+uvdStairPlane(3);
        roiMask=zeros(480,640);
        roiMask(ROI(2):ROI(4)+ROI(2),ROI(1):ROI(1)+ROI(3))=1;
        stairCandidate(NumStairPlane).planeMask = (abs(uvdPoints(:,:,3) - d_uvdPlane) < EPS_planeRANSAC).*roiMask;
        end
    end
    
    if length(SupportEdges)/length_SEdges<0.2; break; end
    loopnum=loopnum+1;
    if loopnum>30; break; end;

end
tic
%stairCandidate=mergeStairplane(uvdPoints,stairCandidate,EPS_planeRANSAC, vsEdgesLeft, intrinsics, baseline, xyzGP);
toc
red = cat(3, ones(size(imLeftd)), zeros(size(imLeftd)), zeros(size(imLeftd)));
green=cat(3, zeros(size(imLeftd)), ones(size(imLeftd)), zeros(size(imLeftd)));

stairModel=[];
for i= 1:length(stairCandidate)
uvdPlaneLQ=stairCandidate(i).uvdPlane;


h_plane=figure(4+10*(i));
imshow(imLeftd,[],'Border', 'tight');
hold on;


plotEdglets(vsEdgesLeft(stairCandidate(i).suppEdgelets),'b');
%planeMaskLQ=planeDraw(uvdPlaneLQ, 4+10*(i), uvdPoints ,EPS_planeRANSAC, red);
figure(4+10*(i));
hold on; 
maskColor = imshow(red);
%set(maskColor, 'AlphaData', planeMask.*0.5);
set(maskColor, 'AlphaData', stairCandidate(i).planeMask.*0.5);
planeMakseGP=planeDraw(uvdPlaneGP, 4+10*(i), uvdPoints ,EPS_planeRANSAC, green);
%saveas(h_plane,sprintf('/home/zhong/images/stairsPlane_selectededges%05d.png',iFr*100+NumStairPlane));
stairCandidate(i).inclination

%figure(5+10*NumStairPlane),
%imshow(planeMaskLQ.*imLeftd,[])

%figure(8+10*NumStairPlane)
%imshow(planeMaskGp.*imLeftd,[]);



%% calculate the gravity center and the average distance to the gravity center of pixels on the 'houghedges'
[dimR, center_roi]= rect_Stair(vsEdgesLeft(stairCandidate(i).suppEdgelets));

h=figure(4+10*(i)); hold on;
ROI=round([dimR(1), 1,dimR(3), 479 ]); %dimR;%
r=rectangle('Position',ROI,'LineWidth',2);
plot(center_roi(1),center_roi(2),'*','MarkerSize', 20);
hold off;
saveas(h,sprintf('/home/zhong/images/stairsOutput%04d.png',iFr+frameNo(1)+i));

%Num_i=stepsCalc([1 1 639 479],uvdPoints, xyzGP, intrinsics, baseline, 0.005, 0.01, NumStairPlane);


Num_i=stepsCalc(ROI,uvdPoints, xyzGP, intrinsics, baseline, 0.005, 0.008, NumStairPlane);
[pks, locs] = findpeaks(Num_i, 'MINPEAKHEIGHT',25, 'MINPEAKDISTANCE', 0.10/0.005);
if length(locs)<4; continue; end
[step_locs,stairCandidate(i).stepHeight]=findinterval(0.005.* locs);
step_locs_pseudo=[(1:round(xyzGP(4)/stairCandidate(i).stepHeight))' stairCandidate(i).stepHeight.* (1:round(xyzGP(4)/stairCandidate(i).stepHeight))'];
stairCandidate(i).stepPlanes=drawStepPlanes(imLeftd, ROI,uvdPoints, step_locs, xyzGP, baseline, intrinsics, 0.3);
[stairCandidate(i).convexEdge , stairCandidate(i).concaveEdge]=stairReconstruct(ROI, stairCandidate(i).uvdPlane,stairCandidate(i).stepPlanes,uvdPlaneGP, intrinsics, baseline);
isFront=isFrontPlane(stairCandidate(i).concaveEdge,uvdPoints,0.5, intrinsics);
%stairPlot3(stairCandidate(i),xyzPoints,xyzGP,baseline, intrinsics, ROI, h);

stairModel.GP=xyzGP;
stairModel.firstPlane=xyzPlane2(stairCandidate(i).stepPlanes{1},intrinsics,baseline);
stairModel.stairPlane=stairCandidate(i).xyzPlane;
stairModel.stepHeight=stairCandidate(i).stepHeight;
stairModel.inclination=stairCandidate(i).inclination;

%% calculate the distance of each segments of two models 
stepNum=length(stairCandidate(i).stepPlanes)+1;
P_step=zeros(stepNum,3);
u_step=zeros(stepNum,3);
[P_step(1,:), u_step(1,:)]=getIntersectionLine(stairModel.GP, stairModel.stairPlane);
%[P_step(2,:), u_step(2,:)]=getIntersectionLine([stairModel.GP(1:3), stairModel.firstPlane(4)], stairModel.stairPlane);
planePredict=[stairModel.GP(1:3), stairModel.GP(4)-stairModel.stepHeight];
[P_step(2,:), u_step(2,:)]=getIntersectionLine( planePredict, stairModel.stairPlane);
for stepId=3:stepNum
    planePredict=[stairModel.GP(1:3), planePredict(4)-stairModel.stepHeight];
    [P_step(stepId,:), u_step(stepId,:)]=getIntersectionLine(planePredict, stairModel.stairPlane);    
end

hi=figure(333);
for s=1:length(P_step)
p1=getuvdPoint(P_step(s,:),intrinsics,baseline);
p2=getuvdPoint(P_step(s,:)+u_step(s,:),intrinsics,baseline);
u1=(p2-p1)./norm(p2-p1);
draw3DLine(p1,u1,ROI, hi);

end
dist=0;
for s=1:length(stairCandidate(i).convexEdge)
    pR_xyz=get3DPoint(stairCandidate(i).convexEdge(s).PtR, intrinsics, baseline);
    pL_xyz=get3DPoint(stairCandidate(i).convexEdge(s).PtL, intrinsics, baseline);
    P0=stairCandidate(i).convexEdge(s).P0_uvd;
    u0=stairCandidate(i).convexEdge(s).u_uvd;
    [P0x, u0x]=xyzLine(P0, u0, intrinsics, baseline);
    s1=(pL_xyz(1)-P0x(1))./u0x(1);
    s2=(pR_xyz(1)-P0x(1))./u0x(1);
    
    p1_measure=P0x+s1.*u0x;
    p2_measure=P0x+s2.*u0x;
    
    P1=P_step(s+1,:);
	u1= u_step(s+1,:);
    s1=(pL_xyz(1)-P1(1))./u1(1);
    s2=(pR_xyz(1)-P1(1))./u1(1);
    
	p1_predict=P1+s1.*u1;
    p2_predict=P1+s2.*u1;
    
    dist=dist+distanceSegm(p1_measure, p2_measure, p1_predict, p2_predict);
end
    distAve=dist/length(stairCandidate(i).convexEdge);
    
    stairModleFig=figure();
    stairmodelPlot( stairModel, xyzPoints, stairModleFig);
     
end
if  ~isempty(stairModel)
if ~isempty(stairModel(1).stepHeight)
    stepHeightMeasured=[stairModel(1).stepHeight 0];
    trackedStepheight=correct(kalmanFilterStepHeight, stepHeightMeasured);
end


fprintf(fid, '%6.5f %6.5f\n',stairModel(1).stepHeight,trackedStepheight);
end
%}
end
fclose(fid);
