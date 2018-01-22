function [detectedPlane, isObjectDetected, supportEdges, vanishingPoint_uv,inclinationAngle, gpCorrected]=detectStair(figHandle, grayImg, uvdPoints, xyzPoints, gp, selectMethod, predictedPlane, inclinationDetected, stepsPlanes )
% selectMethod can be 'RansacFit' or 'PlanePredicted'

% for 'RansacFit', the stair plane is not predicted by previous detection,
% then the support edgelets should be selected by a Ransac plane fitting

% for 'PlanePredicted' mode, the stair plane is roughly predicted by
% previous detection, then the support edgelets can be selected as the
% edgelets that is withing a distance to the plane.

gpCorrected=[];
isObjectDetected=false;
detectedPlane=[];
supportEdges=[];
vanishingPoint_uv=[];
inclinationAngle=[];
gpCorrected=[];
global EPS_houghRANSAC EPS_planeRANSAC ARGS EPS_roimask;
global intrinsics baseline colorSet;
red = cat(3, ones(size(grayImg)), zeros(size(grayImg)), zeros(size(grayImg)));
yellow=cat(3, ones(size(grayImg)), ones(size(grayImg)), zeros(size(grayImg)));
inclinationAngle=[];
figure(figHandle);
clf
imshow(grayImg ,[],'Border', 'tight');
hold on;


[vsEdges,edgeImg] = FACADE_getEdgelets2(grayImg, ARGS);


% round the coordinats of edgelets into integars
for i=1:length(vsEdges)
    vsEdges(i).vPts_int=RoundUV(vsEdges(i).vPts_un,[ 640 480]);
end

%% find the horitzontal edges
hrzEdges=zeros(1,1);
k=1;
for i=1:length(vsEdges)
    if abs(vsEdges(1,i).theta)<45
        hrzEdges(k)=i;
        k=k+1;
    end
end
if isempty(hrzEdges)
   fprintf('Error: there is no horizontal edges');
   return;
end
  

%% do a ransac line fitting to the hough transform of edgelets

% build hough space of edgelets
hough_curr = zeros(length(vsEdges(hrzEdges)),2);
for i=1:length(vsEdges(hrzEdges));
    th = -(pi/180)*(vsEdges(1,hrzEdges(i)).theta);
    hough_curr(i,1) = th;
    hough_curr(i,2) = vsEdges(1,hrzEdges(i)).vCt_un(2) * cos(th) + vsEdges(1,hrzEdges(i)).vCt_un(1) * sin(th);
    if hough_curr(i,2) < 0
        hough_curr(i,2) = -hough_curr(i,2);
        hough_curr(i,1) = hough_curr(i,1)+pi;
    end
end

% do a ransac line fit to the hough transforms of edgelets
[f_hough,iPt_hough, Id_in_hrzEdges]=RansacLineFit([hough_curr(:,2)./cos(hough_curr(:,1)),tan(hough_curr(:,1))], EPS_houghRANSAC);

VanishPoint_uv=[1/f_hough.p1 -f_hough.p2/f_hough.p1];
groupedEdges=hrzEdges(Id_in_hrzEdges)  ;


if isempty(groupedEdges)
    fprintf('No edgelets can be groupped ');  
    return
end


candidateEdges=groupedEdges;
%candidateEdges=1:length(vsEdges);


length_SEdges=length(candidateEdges);
NumStairPlane=0;


xyzGP=gp;
uvdGP=uvdPlane2(gp, intrinsics, baseline);
%the accepted range for the inclination of stair plan vsEdges(i).vPts_int=RoundUV(vsEdges(i).vPts_un,[ 640 480]);e
range_Inclination=[15 50]; 

stairCandidate=struct([]);
figure(9);
clf
hold on;
imshow(grayImg,[],'Border', 'tight');
hold on;
plotEdglets(vsEdges,'g');
hold on;
plotEdglets(vsEdges(candidateEdges),'r');
hold off;
figure(figHandle);
%% if there is not previous stair plane information given, fit a plane 
if strcmp(selectMethod,'RansacFit');
loopnum=1;


    %if length(candidateEdges)<2; break; end
    
    [xyzPlaneCadidates, PlaneSupports] = fitStairPlaneuvd2(uvdPoints,vsEdges(candidateEdges),EPS_planeRANSAC,1000, gp, 5);
    
    NumSupp=0;
    for a=1:size(xyzPlaneCadidates,1)
    inclination= 180/pi * acos(abs(dot(xyzPlaneCadidates(a,1:3),xyzGP(1:3))));
    
    if inclination<range_Inclination(2)&&inclination>range_Inclination(1) 
        dist=xyzPlaneCadidates(a,4);
        if size(PlaneSupports{a},1)>NumSupp;
            detectedPlane=xyzPlaneCadidates(a,:);
            isObjectDetected=true;
            PlaneID=a;
            NumSupp= size(PlaneSupports{a},1);
        end
    end
    end
        
    if isObjectDetected
    plot(PlaneSupports{PlaneID}(:,1), PlaneSupports{PlaneID}(:,2), 'y.' );  
    end
%{
while (true)
    if length(candidateEdges)<2; break; end
    %[xyzStairPlane, stairsEdges] = fitStairPlanexyz(xyzPoints,vsEdges(candidateEdges),EPS_planeRANSAC,1000,intrinsics, baseline);
    [uvdStairPlane, stairsEdges] = fitStairPlanexyz(uvdPoints,vsEdges(candidateEdges),EPS_planeRANSAC,1000,intrinsics, baseline);
    if isempty( stairsEdges) ; break; end;
    uvdStairPlane2=-1.*[uvdStairPlane(1:2) uvdStairPlane(4)]./uvdStairPlane(3);
    xyzStairPlane=xyzPlane2(uvdStairPlane2, intrinsics, baseline);
    
        
    stairsEdges=candidateEdges(stairsEdges) ;
    
     %replace the edgelets set with the outliers set
    Outliers=zeros(0,0);
    k=1;
    for n=1:length(candidateEdges)
        if stairsEdges(:)~=candidateEdges(n)
            Outliers(k)=candidateEdges(n);
            k=k+1;
        end
    end
    candidateEdges=Outliers;  
    
    % do a least square fit to with the selected edgelets
    uvdStairPlane=LQPlaneFit( uvdPoints,vsEdges( stairsEdges));
    xyzStairPlane=xyzPlane2(uvdStairPlane, intrinsics, baseline);
    
    % see if the angle between the plane candidate and ground
    % plane is between 25 deg to 47 deg.
    inclination =  180/pi * acos(dot(xyzStairPlane(1:3),xyzGP(1:3)));
    if inclination>range_Inclination(2)||inclination<range_Inclination(1)
        fprintf('Stair plane not found: inclination too big.');
        continue;
    else %else return the plane as a stair plane
        if length(stairsEdges)<4
            fprintf('Stair plane not found: not enough edgelets.');
            break;
        else
        NumStairPlane=NumStairPlane+1;
        stairCandidate(NumStairPlane).suppEdgelets=stairsEdges;
        stairCandidate(NumStairPlane).xyzPlane= xyzStairPlane;
        stairCandidate(NumStairPlane).uvdPlane= uvdStairPlane;
        stairCandidate(NumStairPlane).inclination=inclination;      
        end
    end
    if length(candidateEdges)/length_SEdges<0.2; break; end
    loopnum=loopnum+1;
    if loopnum>30; break; end;
end

% find out the stair plane which is closest to the camera as the convex
% edgelets's plane.

if isempty(stairCandidate)
    fprintf('No stair plane can be fitted');
    return

else
dist=[];
for s=1:length(stairCandidate)
    if abs(stairCandidate(s).xyzPlane(4))>1.5
        dist=[dist stairCandidate(s).xyzPlane(4)];
    end
end
[~, idMin]=min(dist);
detectedPlane=stairCandidate(idMin).xyzPlane;
isObjectDetected=true;
supportEdges=stairCandidate(idMin).suppEdgelets;
end
%}
end

%% if there is previous stair plane information given, select the edgelets within a distance to the given plane
if strcmp(selectMethod, 'PlanePredicted')
    tic
    [xyzStairPlane, stairsEdges, LQsupportPts] = fitPlaneFromPrediction2(xyzPoints, uvdPoints, vsEdges(hrzEdges), predictedPlane, uvdGP, EPS_planeRANSAC);
    toc
    if length(stairsEdges)>5
        detectedPlane=xyzStairPlane;
        %supportEdges=groupedEdges(stairsEdges);
        %supportEdges=stairsEdges;
        isObjectDetected=true;
    end
    
    hold on;
    plot(LQsupportPts(:,1), LQsupportPts(:,2), 'y.' );    
    
    
end

%%
if strcmp(selectMethod, 'PlaneNstepsPredicted')
    
    for j=1:size(stepsPlanes,1)
        [P_step(j,:), u_step(j,:)]=getIntersectionLine(stepsPlanes(j,:), predictedPlane);
    end

edgePoints1=[1:1:640];
ROI=[1 1 639 479];
for s=1:size(P_step,1)
p1=getuvdPoint(P_step(s,:),intrinsics,baseline);
p2=getuvdPoint(P_step(s,:)+u_step(s,:),intrinsics,baseline);
u1=(p2-p1)./norm(p2-p1);

%s1=(ROI(1)-p1(1))./u1(1);
%s2=(ROI(1)+ROI(3)-p1(1))./u1(1);

%pL=p1+s1.*u1;
%pR=p1+s2.*u1;

[pL, pR]=draw3DLine2(p1, u1, ROI, figHandle);
edgePoints2=(pR(2)-pL(2))/(pR(1)-pL(1)).*(edgePoints1-pL(1))+pL(2);

IntersectionEdges(s).vPts_un=cat(1,edgePoints1,edgePoints2);
IntersectionEdges(s).vPts_int=RoundUV(IntersectionEdges(s).vPts_un,[640 480]);
end


    [xyzStairPlane, stairsEdges, LQsupportPts] = fitPlaneFromPrediction3(xyzPoints, uvdPoints, IntersectionEdges, predictedPlane, uvdGP, EPS_planeRANSAC);
    
    isObjectDetected=true;
    detectedPlane=xyzStairPlane;
    hold on;
    plot(LQsupportPts(:,1), LQsupportPts(:,2), 'b.' );   
end
%% plot


if  isObjectDetected

    % get the edgelets on the stair plane
    PlaneEdges=vsEdges(hrzEdges);
    suppEdgeId=[];
    for n=1:length(PlaneEdges)
 
        edgePointsuvd{n}=zeros(0,3);
        for m=1:size(PlaneEdges(n).vPts_int,2)
            edgePointsuvd{n}(m,:)=reshape(uvdPoints(PlaneEdges(n).vPts_int(2,m),PlaneEdges(n).vPts_int(1,m),:),1,3);   
        end
    
        detectedPlane_uvd=uvdPlane2(detectedPlane, intrinsics, baseline);
    
        d_Edge2Plane=detectedPlane_uvd(1)*edgePointsuvd{n}(:,1)+detectedPlane_uvd(2)*edgePointsuvd{n}(:,2)+detectedPlane_uvd(3);
        suppPointsOnEdge=(abs(d_Edge2Plane-edgePointsuvd{n}(:,3))<EPS_planeRANSAC)&(edgePointsuvd{n}(:,3)>1e-5);
        if sum(suppPointsOnEdge)/size(edgePointsuvd{n},1)>0.3
            suppEdgeId=[suppEdgeId n];
        end
    
    end
    
    if length(suppEdgeId)<5
        isObjectDetected=0;
        return;
    end
    
    suppEdges=PlaneEdges(suppEdgeId);
    hough_curr = zeros(length(suppEdges),2);
    for i=1:length(suppEdges);
    th = -(pi/180)*(suppEdges(1,i).theta);
    hough_curr(i,1) = th;
    hough_curr(i,2) = suppEdges(1,i).vCt_un(2) * cos(th) + suppEdges(1,i).vCt_un(1) * sin(th);
    if hough_curr(i,2) < 0
        hough_curr(i,2) = -hough_curr(i,2);
        hough_curr(i,1) = hough_curr(i,1)+pi;
    end
    end
    
    % do a ransac line fit to the hough transforms of edgelets
    [f_hough,iPt_hough, Id_in_hrzEdges]=RansacLineFit([hough_curr(:,2)./cos(hough_curr(:,1)),tan(hough_curr(:,1))], EPS_houghRANSAC);

	figure(9);
    hold on;
    plotEdglets(PlaneEdges(suppEdgeId(Id_in_hrzEdges)),'b');
    hold off;
    figure(figHandle);
    
    VanishPoint_uv=[1/f_hough.p1 -f_hough.p2/f_hough.p1];

    detectedPlanexyz=[-detectedPlane(1:2)/detectedPlane(3) -detectedPlane(4)/detectedPlane(3)];
    d_xyzPlane=detectedPlanexyz(1)*xyzPoints(:,:,1)+detectedPlanexyz(2)*xyzPoints(:,:,2)+detectedPlanexyz(3);
    planeMask=abs(xyzPoints(:,:,3) - d_xyzPlane) < EPS_roimask;
    maskColor=imshow(red);
    set(maskColor, 'AlphaData', planeMask.*0.5);
    
    if strcmp(selectMethod, 'PlanePredicted')
    inclinationAngle= inclinationDetected;
    else
        inclinationAngle=180/pi * acos(abs(dot(detectedPlane(1:3),xyzGP(1:3))));
    end
    [gpVector, stairDirtVect]=getGPfromVectors(detectedPlane, inclinationAngle, VanishPoint_uv);
    
    
    vpt_xyz=[VanishPoint_uv(1)-intrinsics(1,3); VanishPoint_uv(2)-intrinsics(2,3); intrinsics(1,1)];
    vptVect=vpt_xyz(1:3)/norm(vpt_xyz(1:3)).*sign(vpt_xyz(1));

    
    pCenter=[0 0 detectedPlane(4)/abs(detectedPlane(3))]';
    ptGpVect=pCenter+gpVector/2;
    ptStairDirtVect=stairDirtVect/2+pCenter;
    ptVanishPoint=vptVect/2+pCenter;
    
    pCenter_uv=intrinsics*pCenter;
    pCenter_uv=pCenter_uv/pCenter_uv(3);
    
    ptGpVect_uv=intrinsics*ptGpVect;
    ptGpVect_uv=ptGpVect_uv/ptGpVect_uv(3);
    
    ptStairDirtVect_uv=intrinsics*ptStairDirtVect;
    ptStairDirtVect_uv=ptStairDirtVect_uv/ptStairDirtVect_uv(3);
    
    ptVanshishPoint_uv=intrinsics*ptVanishPoint;
    ptVanshishPoint_uv=ptVanshishPoint_uv/ptVanshishPoint_uv(3);
    
    u0=pCenter_uv(1); v0=pCenter_uv(2);
    arrowGPvect = annotation('arrow', [u0 ptGpVect_uv(1)]/640, [480-v0 480-ptGpVect_uv(2)]/480,'Color', 'r');
    arrowStairDirt = annotation('arrow', [u0 ptStairDirtVect_uv(1)]/640, [480-v0 480-ptStairDirtVect_uv(2)]/480,'Color', 'g');
    arrowVp = annotation('arrow', [u0 ptVanshishPoint_uv(1)]/640, [480-v0 480-ptVanshishPoint_uv(2)]/480,'Color', 'b');
    
    gpCorrected=gpVector;
    angleGP= 180/pi * acos(abs(dot(xyzGP(1:3),gpVector')));
    if angleGP>10
       gpCorrected=xyzGP(1:3)';
    end
   
   supportEdges=suppEdges;
end
    vanishingPoint_uv=VanishPoint_uv;
    uvdGP=uvdPlane2(gp, intrinsics, baseline);
    planeDraw(uvdGP,figHandle, uvdPoints, EPS_planeRANSAC, colorSet{2});
hold off;
end
