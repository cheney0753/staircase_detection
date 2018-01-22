function [imgPath, intrinsics, baseline, leftImgStr, dispImgStr, frameNo, frame1st, framelast, ...
     GPinitial_uvd, egopose2]=readData( datasetName)
 
% durlachBahnHof 2670
if strcmp(datasetName, 'DurlachBahnhofF2670')
    
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof2650.mat');
frame1st=2670;
framelast=3000;
frameNo =frame1st:1:framelast;
egopose2=egopose;
% initial gp information
GPinitial_uvd=[-0.0012    0.0766  -12.4923]; %875
end
 
% durlachBahnHof 4196
if strcmp(datasetName, 'DurlachBahnhofF4196')
    
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof4100.mat');
frame1st=4196;
framelast=4300;
frameNo =frame1st:1:framelast;
egopose2=egopose;
% initial gp information
GPinitial_uvd=[-0.0016    0.0715   -5.3483]; %4158
end

% durlachBahnHof 4258
if strcmp(datasetName, 'DurlachBahnhofF4258')
    
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof4100.mat');
frame1st=4258;
framelast=4400;
frameNo =frame1st:1:framelast;
egopose2=egopose;
% initial gp information
GPinitial_uvd=[ -0.0028    0.1462  -21.1917]; %4158
end

% durlachBahnHof 5018
if strcmp(datasetName, 'DurlachBahnhofF5019')
    
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof5000.mat');
frame1st=5019;
framelast=5200;
frameNo =frame1st:1:framelast;
egopose2=egopose;
% initial gp information
GPinitial_uvd=[ -0.0072    0.0736  -10.6326]; %5019
end

if strcmp(datasetName, 'DurlachBahnhofF5019')
    
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof5000.mat');
frame1st=5019;
framelast=5200;
frameNo =frame1st:1:framelast;
egopose2=egopose;
% initial gp information
GPinitial_uvd=[ -0.0072    0.0736  -10.6326]; %5019
end

% durlachBahnHof 875
if strcmp(datasetName, 'DurlachBahnhofF5121')
    
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
load ('vo_durlachBahnhof5000.mat');
frame1st=5121;
framelast=5300;
frameNo =frame1st:1:framelast;
egopose2=egopose;
% initial gp information
GPinitial_uvd=[0.0035    0.1920  -56.3158]; %5121
end



% durlachBahnHof 785
if strcmp(datasetName, 'DurlachBahnhofF785')
    
imgPath = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
intrinsics = [464.996 0 321.785 ; 0 464.996 233.978 ; 0 0 1 ];
baseline = 0.137532 ;
leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];

load ('vo_durlachBahnhof760.mat');
frame1st=785;
framelast=1036;

frameNo =frame1st:1:framelast;

egopose2=egopose;
% initial gp information
GPinitial_uvd=[-0.0001    0.0706   -5.0307] ;%785
end





%Berwald dataset
if strcmp(datasetName, 'BergwaldF765');
imgPath = '/mrt/storage/users/lauer/2013_07_08_Bergwald';
intrinsics = [496.6227 0 315.2575;0 499.1608 240.0210;0 0 1.0000];
baseline = 0.1375;

leftImgStr  = [imgPath,'/stream_left_1_%06d.rect.png'];
dispImgStr = [imgPath,'/stream_left_1_%06d.disp.png'];
frame1st=765;
framelast=1119;

frameNo =frame1st:1:framelast;

load ('vo_bergwald.mat');
egopose2=egopose;
%GPinitial_uvd=[ 0.0057    0.0825  -19.1654]; %280
GPinitial_uvd=[0.0014    0.0751  -13.6035]; %310
end


% kaisPizza dataset
if strcmp(datasetName, 'KaisPizza3590');
imgPath = '/mrt/storage/users/schwarze/OIWOB_data/2014_6_4_kaisPizza';
intrinsics = [ 361.836 0 302.187 ; 0 361.836 231.214 ; 0 0 1 ];
baseline =  0.185368;

%gp=load('gp_kaisPizza.txt');
GPinitial_uvd=[-0.00267994000000000 0.0914462000000000 -2.76602000000000]; %3550
%GPinitial_uvd=gp(1,2:4);
leftImgStr  = [imgPath,'/cache/stream_b09d01009e411d_1_%06d.rect.png'];
dispImgStr = [imgPath,'/cache/stream_b09d01009e411d_1_%06d.rect.dispSGBM.png'];
frame1st=3590;
framelast=4500;
frameNo =frame1st:1:framelast;
load ('egoposes_kaisPizzaStair.mat');
egopose2=egopose;
end

end