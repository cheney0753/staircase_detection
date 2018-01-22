function h = plotPlane(plane,figureHandle,xRange,yRange,zRange)


figure(figureHandle)

[x,y] = meshgrid(xRange, yRange);
[x,z] = meshgrid(xRange, zRange); % 30
y = -(plane(1)*x+plane(3)*z+plane(4))/plane(2);
h = mesh(x,y,z);
%colormap (ones(64,3)*0.9);


end