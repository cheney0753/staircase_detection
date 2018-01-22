clear
close all



global EPS_houghRANSAC; EPS_houghRANSAC=0.01;
global EPS_planeRANSAC; EPS_planeRANSAC=0.5;
global EPS_stepPlaneRANSAC; EPS_stepPlaneRANSAC=0.2;
global topHeight;topHeight=0.5;
global EPS_roimask; EPS_roimask=0.6;

global intrinsics;
global baseline;
global ARGS;
global colorSet;
global imgSize;



[imgPath, intrinsics, baseline, leftImgStr, dispImgStr, frameNo, frame1st, framelast, ...
     GPinitial_uvd, egopose]=readData('BergwaldF765');






%%

P_left = intrinsics * [diag([1,1,1]) [0;0;0]];

imgSize=[480 640];

colorSet{1}=cat(3, ones(imgSize), zeros(imgSize), zeros(imgSize));
colorSet{2}=cat(3, zeros(imgSize), ones(imgSize), zeros(imgSize));
colorSet{3}=cat(3, zeros(imgSize), zeros(imgSize), ones(imgSize));
colorSet{4}=cat(3, ones(imgSize), ones(imgSize), zeros(imgSize));
colorSet{5}=cat(3, zeros(imgSize), ones(imgSize), ones(imgSize));

ARGS = FACADE_ARGS_default();
const = FACADE_const();
ARGS.minEdgeLength     = 30;    %in pixels
ARGS.linesegTOL        = 10;%2;     %pixel tolerance when selecting straight edges
ARGS.mKinv = [];
ARGS.imgS = intrinsics(1,1);

u = repmat([1:640],480,1);
v = repmat([1:480]',1,640);

imgHandle=figure(1);


GPinitial_xyz=xyzPlane2(GPinitial_uvd,intrinsics, baseline);
GPPredicted_global=inv(egopose{frameNo(1)})'*GPinitial_xyz';
GroundPlane{1}=GPPredicted_global';
consecutiveEnd=0;
isStairDetected{1}=0;
isGPdetected{1}=1;
isStepsDetected{1}=0;

isTracksempty=1;
isStepTrackingAcitve=0;
for iFr=2:length(frameNo)
	if iFr==30
       qw=0;
    end
%% get the raw data of current frame
    cla
    grayImg=imread(sprintf(leftImgStr,frameNo(iFr)));
    dispImg=double(imread(sprintf(dispImgStr,frameNo(iFr))));
    disparity=dispImg./256;
    uvdPoints(:,:,1) = u;
    uvdPoints(:,:,2) = v;
    uvdPoints(:,:,3) = disparity; 
    % the 3D points in real world coordinate
    xyzPoints=get3DPoints(disparity, intrinsics, baseline);
    ROIMask=[];
    
    if isempty(egopose{frameNo(iFr)})
        egopose{frameNo(iFr)}=egopose{frameNo(iFr-1)};
    end

    
    
%% Detect the Gp
    
% get the prediction of Gp

	GPPredicted_current=(egopose{frameNo(iFr)}'*GroundPlane{iFr-1}')';

    
    [GpDetected, NumSupporters]=trackGP(GPPredicted_current, uvdPoints,0.4);
    
    if NumSupporters>15000
        isGPdetected{iFr}=1;
    else
        isGPdetected{iFr}=0;
    end
    
    % if gp is DETECTED
    if isGPdetected{iFr}
        GPforStairDetection=GpDetected;
    else
        GPforStairDetection=GPPredicted_current;
    end


    
    
%% detect and tracking the stair plane


    
    % if the stairplane have never been measure yet
    if ~isStairDetected{iFr-1}
        [StairPdetected, isStairDetected{iFr}, supportEdges,vanishingPoint_uv, inclinationAngle, GpCorrected]=...
            detectStair(imgHandle, grayImg, uvdPoints, xyzPoints, GPforStairDetection, 'RansacFit');
        StairPpredicted=StairPdetected;
    else  
        %predict the plane of stair with the stair plane measured at last frame
        StairPpredicted= (egopose{frameNo(iFr)}'*StairPlane{iFr-1}')';     
        %detect the stair plane based on the plane position at last frame
        %(predicted plane), as well as the updated ground plane

           [StairPdetected, isStairDetected{iFr}, supportEdges, vanishingPoint_uv, inclinationAngle, GpCorrected]=...
            detectStair(imgHandle, grayImg, uvdPoints, xyzPoints, GPforStairDetection, 'PlanePredicted',StairPpredicted, inclinationDetected);
        if length(supportEdges)<0
        if isStepTrackingAcitve
            activetracks=tracks([tracks(:).isActive]==1);
            for stp=1:length(activetracks)
                activeStepCurrent(stp,:)=(egopose{frameNo(iFr)}'*activetracks(stp).feature')';
            end
            [StairPdetected, isStairDetected{iFr}, supportEdges, vanishingPoint_uv, inclinationAngle, GpCorrected]=...
           detectStair(imgHandle, grayImg, uvdPoints, xyzPoints, GPforStairDetection, 'PlaneNstepsPredicted',StairPpredicted, inclinationDetected, activeStepCurrent);
        end
        end
    end    
        
    if  isStairDetected{iFr}
        a=180/pi * acos(abs(dot(StairPdetected(1:3),StairPpredicted(1:3))));
        if a<1
            StairCurrent=StairPdetected;
        else
            StairCurrent=StairPpredicted;
        end
        StairPlane{iFr}=(inv(egopose{frameNo(iFr)})'*StairCurrent')';
        

            
    else
        if isStairDetected{iFr-1}
            StairPlane{iFr}=StairPlane{iFr-1};
        else
            StairPlane{iFr}=[];
        end
    end 
%% update ground plane

        if isGPdetected{iFr}
            GroundPlane{iFr}=(inv(egopose{frameNo(iFr)})' *GpDetected')';

        else
            if isStairDetected{iFr}
                GroundPlane{iFr}=(inv(egopose{frameNo(iFr)})'*[GpCorrected' GPPredicted_current(4)]')';
            else
                GroundPlane{iFr}=GroundPlane{iFr-1};
            end
        end
        
        
%% update the inclination when both stairPlane and ground plane are detected
        inclinationDetected=0;
    if isStairDetected{iFr}&&isGPdetected{iFr}
        inclinationDetected=180/pi*acos(abs(dot(GpDetected(1:3),StairPdetected(1:3))));
        inclinationA{iFr}=inclinationDetected;
    end
    %{
        figure(imgHGP)
        grid on;
        hold on; 
        axis equal;
        plot(iFr, inclinationA{iFr}, '-xb','LineWidth',1);
        hold off;
    %}
%% Detect the step plane with the stair plane and the updated ground plane .

     if  isStairDetected{iFr}||isStairDetected{iFr-1}

        StairPlaneCurrent=((egopose{frameNo(iFr)})'*StairPlane{iFr}')';
        StairPlanexyz=[-StairPlaneCurrent(1:2)/StairPlaneCurrent(3) -StairPlaneCurrent(4)/StairPlaneCurrent(3)];
        d_xyzPlane=StairPlanexyz(1)*xyzPoints(:,:,1)+StairPlanexyz(2)*xyzPoints(:,:,2)+StairPlanexyz(3);
        ROIMask=abs(xyzPoints(:,:,3) - d_xyzPlane) < EPS_roimask;  
        if  isStairDetected{iFr}
        inclination(iFr,:)=inclinationAngle;
        gpVectCorrectedwithVP{iFr}=inv(egopose{frameNo(iFr)}(1:3,1:3))' *GpCorrected;
        end
         StairPforStepsDetection=(egopose{frameNo(iFr)}'*StairPlane{iFr}')';
         GpforStepsDetection=(egopose{frameNo(iFr)}'*GroundPlane{iFr}')';
        [stepHeight{iFr, 1}, stepNormal_CurrentFrame, isStepDetected{iFr}, stepDetected, bBox, NumAtPeak]=detectSteps(GpforStepsDetection,uvdPoints, ROIMask, EPS_stepPlaneRANSAC, StairPforStepsDetection, imgHandle);
        
        if ~isStepDetected{iFr}
            fprintf('No steps detected');
            isStepTrackingAcitve=0;
            continue;
        else
            isStepTrackingAcitve=1;
        end 
   
        stepOrig=(inv(egopose{frameNo(iFr)})'*stepDetected')';
        
    
if isStepDetected{iFr};
    if isTracksempty
        % initialize the step tracks
        for n=1:size(stepOrig,1)
            
            featureFilter=configureKalmanFilter('ConstantVelocity',...
                stepOrig(n,:),[200, 25], [100, 12.5], 10);
            bBoxFilter=configureKalmanFilter('ConstantVelocity',...
                bBox(n,:),[200, 50], [100, 25], 100);
            
            newTrack=struct(...
            'id', length(stepOrig)-n+1, ...
            'NumAtPeak', NumAtPeak(n),...
            'bBox', bBox(n,:),...
            'bBoxFilter', bBoxFilter, ...
            'feature', stepOrig(n,:), ...
            'featureFilter', featureFilter, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0, ...
            'isActive', 1    );                   
            
            tracks(n)=newTrack;  
            
        end    
        isTracksempty=0;
        
        referencePlanePre=StairPforStepsDetection;
        
        isFirstSair=0;
        
    else
        
        
        tracksPre=tracks;
        
        numInactiveTracks=sum([tracks(:).isActive]==0);
        % get the reference plane (the ground plane)
        referencePlane=[tracks(end-1).feature(1:3),tracks(end-1).feature(4)+(1+numInactiveTracks)*stepHeight{iFr, 1}];  
           
        %tracks=trackStep(tracksPre,  stepOrig,  bBox, referencePlane, referencePlanePre);
        tracks=trackStep2(tracksPre,  stepOrig, bBox,NumAtPeak, stepHeight{iFr, 1}, referencePlane, referencePlanePre);
        
        
        
        referencePlanePre=referencePlane;
    
       %% see if we are reaching the end of the stair
        %isEnd=isStairEnd(tracks, 20, 0.7);
        isEnd=0;
        if NumAtPeak(1)>NumAtPeak(2)&&NumAtPeak(1)>NumAtPeak(3)
            isEnd=1;
        end
        if isEnd
            consecutiveEnd=consecutiveEnd+1;
        else
            consecutiveEnd=0;
        end 
        
        if consecutiveEnd>3
            isStepTrackingAcitve=0;
            xyzGPPredicted=inv(egopose{frameNo(iFr)})'*tracks(1).feature';
        end
        
        if ~isStepTrackingAcitve
        break;
        end
        
    end
end  


    

   
     end
end


imgHGP1=figure(6);
imgHGP2=figure(10);
imgHGP3=figure(11);
imgHincl=figure(4);
imgHstepHeight=figure(5);

        figure(imgHGP1)      
        grid on;
        hold on; 
        for iGP=2:length(isGPdetected)
        if isGPdetected{iGP}
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(1), GroundPlane{iGP}(1)],'-or')
        else
            if isStairDetected{iGP}
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(1), GroundPlane{iGP}(1)],'-og')
            else
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(1), GroundPlane{iGP}(1)],'-ob')
            end
        end
        end
        hold off;
        
                figure(imgHGP2)      
        grid on;
        hold on; 
        for iGP=2:length(isGPdetected)
        if isGPdetected{iGP}
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(2), GroundPlane{iGP}(2)],'-or')
        else
            if isStairDetected{iGP}
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(2), GroundPlane{iGP}(2)],'-og')
            else
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(2), GroundPlane{iGP}(2)],'-ob')
            end
        end
        end
        hold off;
        
                figure(imgHGP3)      
        grid on;
        hold on; 
        for iGP=2:length(isGPdetected)
        if isGPdetected{iGP}
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(3), GroundPlane{iGP}(3)],'-or')
        else
            if isStairDetected{iGP}
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(3), GroundPlane{iGP}(3)],'-og')
            else
            plot([iGP-1 iGP], [GroundPlane{iGP-1}(3), GroundPlane{iGP}(3)],'-ob')
            end
        end
        end
        hold off;
        
        figure(imgHstepHeight)
        hold on; 
        for iINC=2:length(stepHeight)
            if isempty(stepHeight{iINC})
                stepHeight{iINC}=stepHeight{iINC-1};
            end
            plot([iINC-1 iINC], [stepHeight{iINC-1} stepHeight{iINC}],'-xb');
        end
        hold off;
        
        figure(imgHincl)
        hold on; 
        for iINC=2:size(inclination,1)
            if inclination(iINC,:)==0
              inclination(iINC,:)=inclination(iINC-1,:);
            end
            plot([iINC-1 iINC], [inclination(iINC-1,:) inclination(iINC,:)],'-xb');
        end
        hold off;

        
 %'bBox', bBox(n,:), ...
            %'bBoxFilter', bBoxFilter, ...



