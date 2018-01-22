clear
close all



global EPS_houghRANSAC; EPS_houghRANSAC=0.05;

global EPS_roimask; EPS_roimask=0.5;
global intrinsics;
global baseline;
global ARGS;
global colorSet;



global EPS_planeRANSAC; EPS_planeRANSAC=0.5;
global EPS_stepPlaneRANSAC; EPS_stepPlaneRANSAC=0.2;
global topHeight;topHeight=0.5;
%{
% kaisPizza dataset

imgPath = '/mrt/storage/users/schwarze/OIWOB_data/2014_6_4_kaisPizza';
intrinsics = [ 361.836 0 302.187 ; 0 361.836 231.214 ; 0 0 1 ];
baseline =  0.185368;

gp=load('gp_kaisPizza.txt');

leftImgStr  = [imgPath,'/cache/stream_b09d01009e411d_1_%06d.rect.png'];
dispImgStr = [imgPath,'/cache/stream_b09d01009e411d_1_%06d.rect.dispSGBM.png'];

frameNo =3595:1:4000;

load ('egoposes_kaisPizzaStair.mat');

frame1st=3450;
%}



%Berwald dataset
imgPath = '/mrt/storage/users/lauer/2013_07_08_Bergwald';
intrinsics = [496.6227 0 315.2575;0 499.1608 240.0210;0 0 1.0000];
baseline = 0.1375;

leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
frame1st=310;
framelast=1119;

frameNo =frame1st:1:framelast;

load ('vo_bergwald.mat');

gp=zeros(length(frameNo),4);
load('gp_bergwald.mat');
for x=frame1st:length(groundPlane)
    if ~isempty(groundPlane{x})
        gp(x-frame1st+1,:)=[x groundPlane{x}];
    end
end
%GPinitial_uvd=[ 0.0057    0.0825  -19.1654]; %280
GPinitial_uvd=[0.0014    0.0751  -13.6035]; %310

%{
% durlachBahnHof
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof760.mat');
frame1st=875;
framelast=1036;

frameNo =frame1st:1:framelast;

gp=zeros(length(frameNo),4);
load('gp_durlachBahnhof760.mat');
for x=frame1st:length(groundPlane)
    if ~isempty(groundPlane{x})
        gp(x-frame1st+1,:)=[x groundPlane{x}];
    end
end

% initial gp information
%GPinitial_uvd=[0.0038    0.1318  -14.3278]; %875

%GPinitial_uvd=[-0.0001    0.0706   -5.0307] ;%785

%}


P_left = intrinsics * [diag([1,1,1]) [0;0;0]];
imgSize=[480 640];
colorSet{1}=cat(3, ones(imgSize), zeros(imgSize), zeros(imgSize));
colorSet{2}=cat(3, zeros(imgSize), ones(imgSize), zeros(imgSize));
colorSet{3}=cat(3, zeros(imgSize), zeros(imgSize), ones(imgSize));
colorSet{4}=cat(3, ones(imgSize), ones(imgSize), zeros(imgSize));
colorSet{5}=cat(3, zeros(imgSize), ones(imgSize), ones(imgSize));

consecutiveEnd=0;
isStairTrackingActive=0;
isStepTrackingAcitve=0;
isFirstSair=0;

%%
ARGS = FACADE_ARGS_default();
const = FACADE_const();
ARGS.minEdgeLength     = 30;    %in pixels
ARGS.linesegTOL        = 10;%2;     %pixel tolerance when selecting straight edges
ARGS.mKinv = [];
ARGS.imgS = intrinsics(1,1);

u = repmat([1:640],480,1);
v = repmat([1:480]',1,640);


imgHandle=figure(1);

LostTracks=struct();
list=zeros(1);


GPinitial_xyz=xyzPlane2(GPinitial_uvd,intrinsics, baseline);
GPPredicted_global=inv(egopose{frameNo(1)})'*GPinitial_xyz';
inclination= zeros(length(frameNo),1);
gpCorrectedwithVP=zeros(length(frameNo),3);
gpDetectedWithSP=zeros(length(frameNo),3);
isStairDetected=false;

for iFr=1:length(frameNo)
    
    cla
    grayImg=imread(sprintf(leftImgStr,frameNo(iFr)));
    dispImg=double(imread(sprintf(dispImgStr,frameNo(iFr))));
    disparity=dispImg./256;
    uvdPoints(:,:,1) = u;
    uvdPoints(:,:,2) = v;
    uvdPoints(:,:,3) = disparity; 

    % the 3D points in real world coordinate
    xyzPoints=get3DPoints(disparity, intrinsics, baseline);
    
    %uvdGP=gp(frameNo(iFr)-frame1st+1,2:4)';
    %xyzGP=xyzPlane2(uvdGP,intrinsics,baseline);
    ROIMask=[];
    
    
    if isempty(egopose{frameNo(iFr)})
        egopose{frameNo(iFr)}=egopose{frameNo(iFr-1)};
    end
    
    transMatrix=egopose{frameNo(iFr)}/egopose{frameNo(1)};
    
    %Updated_xyzGP=xyzGP;
    %if iFr==1
    %    xyzGPPredicted=xyzGP;
    %end
    
    
    %% get the ground plane
    
    GPPredicted_current=(egopose{frameNo(iFr)}'*GPPredicted_global)';
    
    [gp, NumSupporters]=trackGP(GPPredicted_current, uvdPoints,0.2);
    
    if NumSupporters>20000
        gpDetected=gp;
        isGPdetected(iFr, 1)=1;
    else
        isGPdetected(iFr, 1)=0;
        gpDetected=GPPredicted_current;
    end
    gpDetectedWithSP(iFr, :)=gpDetected(1:3);
    
    gpDetected_uvd=uvdPlane2(gpDetected, intrinsics, baseline);
    GPPredicted_global=inv(egopose{frameNo(iFr)})' * gpDetected';
    groundPlane{frameNo(iFr)}=GPPredicted_global;

    
    
    Updated_xyzGP=gpDetected;
    xyzGP=Updated_xyzGP;
    if isGPdetected(iFr,1)==0
        gpCorrectedwithVP_current=(egopose{frameNo(iFr)}(1:3,1:3)'*gpCorrectedwithVP(iFr-1,:)')';
        Updated_xyzGP=[gpCorrectedwithVP_current Updated_xyzGP(4)];
        xyzGP=Updated_xyzGP;
    end
    
    %
    
    %% detect and tracking the stair plane
    if ~isStairDetected
        [detectedPlane_CurrentFrame, isStairDetected, supportEdges,vanishingPoint_uv, inclinationAngle, gpCorrected]=detectStair(imgHandle, grayImg, uvdPoints, xyzPoints, Updated_xyzGP, 'RansacFit');
        
        %measure the step height
        if ~isStairDetected   
            continue;
        end
        detectedPlane{iFr,1}=(inv(egopose{frameNo(iFr)})'*detectedPlane_CurrentFrame')';
        StairPlanexyz=[-detectedPlane_CurrentFrame(1:2)/detectedPlane_CurrentFrame(3) -detectedPlane_CurrentFrame(4)/detectedPlane_CurrentFrame(3)];
        d_xyzPlane=StairPlanexyz(1)*xyzPoints(:,:,1)+StairPlanexyz(2)*xyzPoints(:,:,2)+StairPlanexyz(3);
        ROIMask=abs(xyzPoints(:,:,3) - d_xyzPlane) < EPS_roimask;
        
        %Updated_xyzGP=[xyzGP(1:3) 2];
        
        if isStairDetected
            isStairTrackingActive=1;
            isFirstSair=1;
        end
        

        
    else  
        %update the ground plane by using the normal vector of steps
        %measured at last frame
        %if isStepTrackingAcitve
        %    Updated_xyzGP=[stepNormal{iFr-1,1}*transMatrix(1:3, 1:3), 2];
        %else
        %    xyzGPPrediction=xyzGPPredicted*transMatrix;
        %    Updated_xyzGP=trackGP(xyzGPPrediction,uvdPoints, EPS_planeRANSAC);
        %    xyzGPPredicted=Updated_xyzGP/transMatrix;
        %end
        %predict the plane of stair with the stair plane measured at last
        %frame
        predictedPlane_CurrentFrame= (egopose{frameNo(iFr)}'*detectedPlane{iFr-1,1}')';
        
        %detect the stair plane based on the plane position at last frame
        %(predicted plane), as well as the updated ground plane
        [detectedPlane_CurrentFrame, isStairDetected, supportEdges, vanishingPoint_uv, inclinationAngle, gpCorrected]=...
            detectStair(imgHandle,    grayImg, uvdPoints, xyzPoints, Updated_xyzGP, 'PlanePredicted',predictedPlane_CurrentFrame);

        if ~isStairDetected
            detectedPlane_CurrentFrame=predictedPlane_CurrentFrame;
        end
        
        detectedPlane{iFr,1}=(inv(egopose{frameNo(iFr)})'*detectedPlane_CurrentFrame')';
        
        
        
        StairPlanexyz=[-detectedPlane_CurrentFrame(1:2)/detectedPlane_CurrentFrame(3) -detectedPlane_CurrentFrame(4)/detectedPlane_CurrentFrame(3)];
        d_xyzPlane=StairPlanexyz(1)*xyzPoints(:,:,1)+StairPlanexyz(2)*xyzPoints(:,:,2)+StairPlanexyz(3);
        ROIMask=abs(xyzPoints(:,:,3) - d_xyzPlane) < EPS_roimask;  
    end   
    
        inclination(iFr,:)=inclinationAngle;
        gpCorrectedwithVP(iFr,:)=inv(egopose{frameNo(iFr)}(1:3,1:3))' *gpCorrected;
        
        %% Detect the step plane with the stair plane and the updated ground plane .
        [stepHeight{iFr, 1}, stepNormal_CurrentFrame, isStepDetected, stepDetected, bBox, NumAtPeak]=detectSteps(Updated_xyzGP,uvdPoints, ROIMask, EPS_stepPlaneRANSAC, detectedPlane_CurrentFrame, imgHandle);
        
        if ~isStepDetected
            fprintf('No steps detected');
            isStepTrackingAcitve=0;
            continue;
        else
            isStepTrackingAcitve=1;
        end 
        
        
        if abs(stepNormal_CurrentFrame*Updated_xyzGP(1:3)')>0.99
            stepNormal{iFr,1}=stepNormal_CurrentFrame/transMatrix(1:3,1:3); 
        else
            fprintf('No step detected');
            stepNormal{iFr,1}=stepNormal{iFr-1,1};      
        end
        norm(cross(xyzGP(1:3),stepNormal_CurrentFrame))
        
        
    if isFirstSair&&isStepDetected;
        
        % initialize the step tracks
        for n=1:size(stepDetected,1)
            
            featureFilter=configureKalmanFilter('ConstantVelocity',...
                stepDetected(n,:),[200, 25], [100, 12.5], 10);
            bBoxFilter=configureKalmanFilter('ConstantVelocity',...
                bBox(n,:),[200, 50], [100, 25], 100);
            
            newTrack=struct(...
            'id', length(stepDetected)-n+1, ...
            'NumAtPeak', NumAtPeak(n),...
            'bBox', bBox(n,:),...
            'bBoxFilter', bBoxFilter, ...
            'feature', stepDetected(n,:), ...
            'featureFilter', featureFilter, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0, ...
            'isActive', 1    );                   
            
            tracks(n)=newTrack;  
        end    
        
        
        LostTracks=initializeTrack();
        referencePlanePre=xyzGP;
        
        isFirstSair=0;
        
    else
        
        
        tracksPre=tracks;
        stepOrig=(egopose{frameNo(1)}'*(inv(egopose{frameNo(iFr)})'*stepDetected'))';
        numInactiveTracks=sum([tracks(:).isActive]==0);
        % get the reference plane (the ground plane)
        referencePlane=[tracks(end-1).feature(1:3),tracks(end-1).feature(4)+(1+numInactiveTracks)*stepHeight{iFr, 1}];  
           
        %tracks=trackStep(tracksPre,  stepOrig,  bBox, referencePlane, referencePlanePre);
        tracks=trackStep2(tracksPre,  stepOrig, bBox,NumAtPeak, stepHeight{iFr, 1}, referencePlane, referencePlanePre);
        
        
        transMatrixPre=transMatrix;     
        
        referencePlanePre=referencePlane;
    
       %% see if we are reaching the end of the stair
        isEnd=isStairEnd(tracks, 20, 0.7);
    
        if isEnd
            consecutiveEnd=consecutiveEnd+1;
        else
            consecutiveEnd=0;
        end 
        
        if consecutiveEnd>5
            isStairTrackingActive=0;
            xyzGPPredicted=inv(egopose{frameNo(iFr)})'*tracks(1).feature';
        end
    
    end
        

    if ~isStairTrackingActive
        break;
    end
   if iFr==53  
       a=0;
   end
   
end
 %'bBox', bBox(n,:), ...
            %'bBoxFilter', bBoxFilter, ...



