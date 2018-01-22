function stairmodelPlot(stairModel, xyzPoints, figHandle)
ROI=[480 640];
range=[-0.02 0.02 10];
step=2;

gpMask=getPointsInRangeToPlane(xyzPoints,stairModel.GP,ROI,range);
gpPoints=cat(3,xyzPoints(:,:,1).*gpMask,xyzPoints(:,:,2).*gpMask,xyzPoints(:,:,3).*gpMask);
figure(figHandle);
plot3(gpPoints(1:step:end,1:step:end,1),gpPoints(1:step:end,1:step:end,2),-gpPoints(1:step:end,1:step:end,3),'r.');
hold on;

for i=1:7
    stepPlane=[stairModel.GP(1:3), stairModel.GP(4)-i*stairModel.stepHeight];
    stepMask=getPointsInRangeToPlane(xyzPoints,stepPlane,ROI,range);
    stepPoints=cat(3,xyzPoints(:,:,1).*stepMask,xyzPoints(:,:,2).*stepMask,xyzPoints(:,:,3).*stepMask);
    plot3(stepPoints(1:step:end,1:step:end,1),stepPoints(1:step:end,1:step:end,2),-stepPoints(1:step:end,1:step:end,3),'g.');
end
stairPlane=[stairModel.stairPlane(1:2)  -stairModel.stairPlane(3) stairModel.stairPlane(4) ];

[x,z] = meshgrid(-0.5:0.2: 2, -1:-0.2: -4.5); % 30
y = -(stairPlane(1)*x+stairPlane(3)*z+stairPlane(4))/stairPlane(2);
m = mesh(x,y,z);
set(m,'facecolor','none');
axis equal;
hold off;
end