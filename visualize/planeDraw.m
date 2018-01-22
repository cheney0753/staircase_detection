function planeMask=planeDraw(uvdPlane, figH, uvdPoints ,eps, color)

d_uvdPlane = uvdPlane(1)*uvdPoints(:,:,1)+uvdPlane(2)*uvdPoints(:,:,2)+uvdPlane(3);

planeMask = abs(uvdPoints(:,:,3) - d_uvdPlane) < eps;

%SE = strel('rectangle',[12 16]);
%planeMask=imclose(planeMask1,SE);
%planeMask=imerode(planeMask1,SE);

figure(figH);
hold on; 
maskColor = imshow(color);
%set(maskColor, 'AlphaData', planeMask.*0.5);
set(maskColor, 'AlphaData', planeMask.*0.5);
end
