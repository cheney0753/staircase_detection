clear
close all



global EPS_houghRANSAC; EPS_houghRANSAC=1;
global EPS_planeRANSAC; EPS_planeRANSAC=1;
global EPS_stepPlaneRANSAC; EPS_stepPlaneRANSAC=0.08;
global intrinsics;
global baseline;
global ARGS;
global colorSet;



imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof760.mat');


imgHandle=figure(1);

frameNo =875:1:1036;


%{
imgPath = '/mrt/storage/users/lauer/2013_07_08_Bergwald';
intrinsics = [496.6227 0 315.2575;0 499.1608 240.0210;0 0 1.0000];
baseline = 0.1375;
leftImgStr  = [imgPath,'/stream_left_1_%06d.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_bergwald.mat');


imgHandle=figure(1);

frameNo =2:1:1119;
%}
P_left = intrinsics * [diag([1,1,1]) [0;0;0]];
imgSize=[480 640];
colorSet{1}=cat(3,  zeros(imgSize), ones(imgSize), zeros(imgSize));

u = repmat([1:640],480,1);
v = repmat([1:480]',1,640);

isGPdetected=zeros(length(frameNo),1);
isGPdetected(1, 1)=1;

%GPinitial_uvd=[   -0.0083    0.0710    2.8954];
GPinitial_uvd=[0.0038    0.1318  -14.3278];
GPinitial_xyz=xyzPlane2(GPinitial_uvd,intrinsics, baseline);
GPPredicted_global=inv(egopose{875})'*GPinitial_xyz';

groundPlane = cell(1119, 1); 
groundPlane{frameNo(1)}=GPinitial_uvd;

thresholdSupp=40000;

for iFr=1:length(frameNo)
    cla
    if isempty(egopose{frameNo(iFr)})
        egopose{frameNo(iFr)}=egopose{frameNo(iFr-1)};
    end
    
    transMatrix=egopose{frameNo(iFr)}/egopose{frameNo(1)};
    
    %GPPredicted_current=(transMatrix'*GPPredicted_global)';
    GPPredicted_current=(egopose{frameNo(iFr)}'*GPPredicted_global)';
    grayImg=imread(sprintf(leftImgStr,frameNo(iFr)));
    dispImg=double(imread(sprintf(dispImgStr,frameNo(iFr))));
    disparity=dispImg./256;
    uvdPoints(:,:,1) = u;
    uvdPoints(:,:,2) = v;
    uvdPoints(:,:,3) = disparity; 
    imgH = figure(1);
    grayImgd=double(grayImg);
    hold on;
    imshow(grayImgd,[]);
    
    xyzPoints=get3DPoints(disparity, intrinsics, baseline);
    

    [gp, NumSupporters]=trackGP(GPPredicted_current, uvdPoints, EPS_planeRANSAC);
   
    if NumSupporters>thresholdSupp
        gpDetected=gp;
        isGPdetected(iFr, 1)=1;
    else
        gpDetected=GPPredicted_current;
    end
    

    
    gpDetected_uvd=uvdPlane2(gpDetected, intrinsics, baseline);
    %groundPlane{frameNo(iFr)}=gpDetected_uvd;
    %GPPredicted_global=gpDetected/transMatrix;
    %GPPredicted_global=inv(transMatrix)' * gpDetected';
    
    GPPredicted_global=inv(egopose{frameNo(iFr)})' * gpDetected';
    groundPlane{frameNo(iFr)}=GPPredicted_global;
    
    d_plane = gpDetected_uvd(1)*uvdPoints(:,:,1)+gpDetected_uvd(2)*uvdPoints(:,:,2)+gpDetected_uvd(3);
    planeMask = abs(uvdPoints(:,:,3) - d_plane) < EPS_planeRANSAC; % plane margin. increase for more points
    green =colorSet{1};
    %red = cat(3, ones(size(inputImg)), zeros(size(inputImg)), zeros(size(inputImg)));
    hold on;
    maskGreen = imshow(green);
    set(maskGreen, 'AlphaData', planeMask.*0.5);
    hold off;
    
end

save('gp_durlachBahnhof760.mat', 'groundPlane');