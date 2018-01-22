global intrinsics baseline gp

intrinsics = [ 402.755 0 317.849 ; 0 402.755 230.576 ; 0 0 1 ];
baseline = 0.129515;


img_path = '/mrt/storage/users/lauer/2013_07_08_Bergwald';

img_idS{1} = 'stream_left_1_000026';
gpS{1} = [0.0523936,-0.998589,0.00871027,1.85122];

testsetId = 1;


dispImg = double(imread([img_path '/' img_idS{testsetId} '.disp.png']));
[inputImg,inputMap] = imread([img_path '/' img_idS{testsetId} '.rect.png']);
gp = gpS{testsetId}
    
inputImg = double(inputImg);
disparity = dispImg./256;
points3d = get3DPoints(disparity,intrinsics,baseline);


h_bestPlanes = figure(1);%('visible','off');
h_allPlanes  = figure(2);%('visible','off');

set(0,'CurrentFigure',h_bestPlanes)
imshow(inputImg,[],'Border', 'tight');

set(0,'CurrentFigure',h_allPlanes)
imshow(inputImg,[],'Border', 'tight');

%roi = round(getrect)
%roi = [235   328    90   145];
%roi = [ 71   193   196    96];
roi = [1 1 639 479];


% uvd "2.5D" points, image coordinates along with disparity value

u = repmat([1:640],480,1);
v = repmat([1:480]',1,640);

uvdPoints(:,:,1) = u;
uvdPoints(:,:,2) = v;
uvdPoints(:,:,3) = disparity;

planeSegmentation = zeros(480,640);

% remove groundplane points

gp_uvd = uvdPlane2(gp,intrinsics,baseline);

d_gp = gp_uvd(1)*uvdPoints(:,:,1)+gp_uvd(2)*uvdPoints(:,:,2)+gp_uvd(3);


gpMask = abs(uvdPoints(:,:,3) - d_gp) < 0.5;

uvdPoints(:,:,1) = uvdPoints(:,:,1) .* ~gpMask;
uvdPoints(:,:,2) = uvdPoints(:,:,2) .* ~gpMask;
uvdPoints(:,:,3) = uvdPoints(:,:,3) .* ~gpMask;
    
%% iteratively extract 5 planes via RANSAC 
% and remove plane from uvd-points after each iteration

label = 0;
while(label < 5)
    
    label = label + 1;

    [bestPlane,bestPlanePoints] = fitUvdPlaneRANSAC(uvdPoints,roi,50,[-0.1 0.1]);

    uvdPlaneRANSAC = bestPlane;
    d_uvdPlaneRANSAC = uvdPlaneRANSAC(1)*uvdPoints(:,:,1)+uvdPlaneRANSAC(2)*uvdPoints(:,:,2)+uvdPlaneRANSAC(3);
    planeMaskRANSAC = abs(uvdPoints(:,:,3) - d_uvdPlaneRANSAC) < 0.5 & uvdPoints(:,:,3) > 0.01;

    sum(sum(planeMaskRANSAC))
    
    %figure(1);
    %imshow(planeMaskRANSAC.*inputImg,[]);

    planeSegmentation = planeSegmentation + label .* planeMaskRANSAC;
    
    xyzPlanes{label} = xyzPlane2(uvdPlaneRANSAC,intrinsics,baseline);
    d_uvdPlaneRANSACo = uvdPlaneRANSAC(1)*u+uvdPlaneRANSAC(2)*v+uvdPlaneRANSAC(3);
    planeMasks{label} = abs(disparity - d_uvdPlaneRANSACo) < 0.3 & disparity > 0.01;

    %figure(2);
    %imshow(planeSegmentation,[]);
    
    % remove plane points from uvd points

    uvdPoints(:,:,1) = uvdPoints(:,:,1) .* ~planeMaskRANSAC;
    uvdPoints(:,:,2) = uvdPoints(:,:,2) .* ~planeMaskRANSAC;
    uvdPoints(:,:,3) = uvdPoints(:,:,3) .* ~planeMaskRANSAC;

end



%% Draw all planes

% refill uvd points
uvdPoints(:,:,1) = u;
uvdPoints(:,:,2) = v;
uvdPoints(:,:,3) = disparity;

green = cat(3, zeros(size(inputImg)), ones(size(inputImg)), zeros(size(inputImg)));
red = cat(3, ones(size(inputImg)), zeros(size(inputImg)), zeros(size(inputImg)));
blue = cat(3, zeros(size(inputImg)), zeros(size(inputImg)), ones(size(inputImg)));
yellow = cat(3, ones(size(inputImg)), ones(size(inputImg)), zeros(size(inputImg)));
purple = cat(3, ones(size(inputImg)), zeros(size(inputImg)), ones(size(inputImg)));


set(0,'CurrentFigure',h_allPlanes)
hold on;

maskGreen = imshow(green);
maskRed = imshow(red);
maskBlue = imshow(blue);
maskYellow = imshow(yellow);
maskPurple = imshow(purple);

set(maskRed, 'AlphaData', (planeSegmentation==1).*0.5);
set(maskYellow, 'AlphaData', (planeSegmentation==2).*0.5);
set(maskGreen, 'AlphaData', (planeSegmentation==3).*0.5);
set(maskBlue, 'AlphaData', (planeSegmentation==4).*0.5);
set(maskPurple, 'AlphaData', (planeSegmentation==5).*0.5);



%% Find the two most parallel planes for final selection

for i=1:label
    for j=i+1:label
        distDev(i,j) = norm(xyzPlanes{i}(1:3)*xyzPlanes{i}(4)-xyzPlanes{j}(1:3)*xyzPlanes{j}(4));
        % abs(xyzPlanes{i}(4))+abs(xyzPlanes{j}(4));
        a = 180/pi * acos(dot(xyzPlanes{i}(1:3),xyzPlanes{j}(1:3)));
        angleDev(i,j) = abs(180 - a);% min(abs(a),abs(180 - a));
    end
end

distDev
angleDev

combi = (distDev>5).*angleDev;
combi(combi==0) = 1000;

[value, index] = min(reshape(combi, numel(combi), 1));
[r,c] = ind2sub(size(combi), index);

combi(r,c)

planeSegmentationBestPlanes = planeMasks{r} | planeMasks{c};

set(0,'CurrentFigure',h_bestPlanes)
hold on;
maskGreen = imshow(green);
set(maskGreen, 'AlphaData', planeSegmentationBestPlanes.*0.5);


%print(h_bestPlanes, '-r80', '-dpng', ['exports/ransacSegm_bestPlanes_' int2str(testsetId) '.png']);
%print(h_allPlanes, '-r80', '-dpng', ['exports/ransacSegm_allPlanes_' int2str(testsetId) '.png']);

    
%error('done');

%% draw 2.5D pointcloud
figure(3)
colormap(hsv(label));
cm = colormap;
hold on;
xlabel('X'),ylabel('Y'),zlabel('disparity');%grid on;
colorbar;

for i=1:label
    mask = (planeSegmentation==i);
    plot3(uvdPoints(:,:,1).*mask,-uvdPoints(:,:,2).*mask,uvdPoints(:,:,3).*mask,'.','MarkerSize',3,'Color',cm(i,:));
end
   
%}
