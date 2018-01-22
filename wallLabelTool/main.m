clear
close all

global intrinsics baseline

intrinsics = [ 402.755 0 317.849 ; 0 402.755 230.576 ; 0 0 1 ];
baseline = 0.129515;


%img_path = '/mrt/storage/users/lauer/2013_07_08_Bergwald';
img_path = '/mrt/storage/users/lauer/2013_07_08_DurlachBahnhof';
%imgPath = '/mrt/storage/users/schwarze/OIWOB_data/2013_8_20_kellertreppe';
%imgPath = '/mrt/storage/users/schwarze/OIWOB_data/2014_6_4_kaisPizza';


img_id{1} = 'stream_left_1_004258';
img_id{2} = 'stream_b09d01009e410e_3_000002';



testsetId = 1;

dispImg = double(imread([img_path '/' img_id{testsetId} '.disp.png']));
[inputImg,inputMap] = imread([img_path '/' img_id{testsetId} '.rect.png']);

inputImg = double(inputImg);
disparity = dispImg./256;
%points3d = get3DPoints(disparity,intrinsics,baseline);

imgH = figure(1);
hold on;
imshow(inputImg,[]);%.*(disparity>0.1),[]);

disp('Pick three points to construct a plane!')
%[u,v] = getpts(imgH)
[u,v] = ginput(3);

p1 = [u(1);v(1);disparity(floor(v(1)),floor(u(1)))];
p2 = [u(2);v(2);disparity(floor(v(2)),floor(u(2)))];
p3 = [u(3);v(3);disparity(floor(v(3)),floor(u(3)))];

plane = fitPlaneTo3Points(p1,p2,p3);

uvdPlane = [plane(1:2)'./-plane(3); plane(4)/-plane(3)];


u = repmat([1:640],480,1);
v = repmat([1:480]',1,640);

uvdPoints(:,:,1) = u;
uvdPoints(:,:,2) = v;
uvdPoints(:,:,3) = disparity;


d_plane = uvdPlane(1)*uvdPoints(:,:,1)+uvdPlane(2)*uvdPoints(:,:,2)+uvdPlane(3);

planeMask = abs(uvdPoints(:,:,3) - d_plane) < 0.1; % plane margin. increase for more points


green = cat(3, zeros(size(inputImg)), ones(size(inputImg)), zeros(size(inputImg)));
red = cat(3, ones(size(inputImg)), zeros(size(inputImg)), zeros(size(inputImg)));

hold on;
maskGreen = imshow(green);
set(maskGreen, 'AlphaData', planeMask.*0.5);


% Least Squares

disp('Draw a region to fit a plane (least squares)!')
roi = round(getrect);

roiMask = zeros(480,640);
roiMask(roi(2):roi(2)+roi(4),roi(1):roi(1)+roi(3)) = 1;

uvdPP(:,1) = reshape(uvdPoints(:,:,1).*roiMask,1,[]);
uvdPP(:,2) = reshape(uvdPoints(:,:,2).*roiMask,1,[]);
uvdPP(:,3) = reshape(uvdPoints(:,:,3).*roiMask,1,[]);

uvdPPf = uvdPP(uvdPP(:,3)~=0,:); % filter zeros

H= [uvdPPf(:,1),uvdPPf(:,2),ones(size(uvdPPf,1),1)];
uvdPlaneLQ = inv(H'*H)*H'*uvdPPf(:,3);
d_uvdPlaneLQ = uvdPlaneLQ(1)*uvdPoints(:,:,1)+uvdPlaneLQ(2)*uvdPoints(:,:,2)+uvdPlaneLQ(3);


planeMaskLQ = abs(uvdPoints(:,:,3) - d_uvdPlaneLQ) < 0.5;

maskRed = imshow(red);

set(maskRed, 'AlphaData', planeMaskLQ.*0.5);

figure,
imshow(planeMaskLQ.*inputImg,[])


figure(6)
step=5;
plot3(uvdPoints(1:step:end,1:step:end,1),-uvdPoints(1:step:end,1:step:end,2),uvdPoints(1:step:end,1:step:end,3),'r.','MarkerSize',5)


figure(7)
imshow(log(disparity),[]);


