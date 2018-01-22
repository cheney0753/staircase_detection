%%
data_vo = load('vo_durlachBahnhof5000.txt'); % store global transformations!
egopose = cell(data_vo(end,1), 1); 
for i=1:size(data_vo,1)
    egopose{data_vo(i,1)} = reshape(data_vo(i,2:end),4,4)';
end
save('vo_durlachBahnhof5000.mat', 'egopose');

%%

load('egoposes_kaisPizzaStair.mat');

%%

 % in frame 3510 me measure this plane
planeLocal_fr3510 = [0,-1,0,1.8]

% transform measurement into global coordinate system
planeGlobal = inv(egopose{3510})' * planeLocal_fr3510'

% global plane will be seen in frame 4000 as
planeLocal_fr4000_prediction = egopose{3510}' * planeGlobal


%%
figure(10);
grid on; hold on;axis equal;

for n=3510:size(egopose,1)

  
  % update trajectory
  if n>3510
    plot3(  [egopose{n-1}(1,4) egopose{n}(1,4)],...
            [egopose{n-1}(2,4) egopose{n}(2,4)],...
            [egopose{n-1}(3,4) egopose{n}(3,4)],'-xb','LineWidth',1);
        
    % visualize heading
    %plot3(  [egopose{n-1}(1,4) egopose{n-1}(1,4)+egopose{n-1}(1,3)],...
    %        [egopose{n-1}(2,4) egopose{n-1}(2,4)+egopose{n-1}(2,3)],...
    %        [egopose{n-1}(3,4) egopose{n-1}(3,4)+egopose{n-1}(3,3)],'-xr','LineWidth',1);
  end
  %pause(0.01); 
  refresh;

end

view(0,0);


