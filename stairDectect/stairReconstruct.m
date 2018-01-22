function [convexEdge_ , concaveEdge_]=stairReconstruct(ROI,stairPlane,stepPlanes,GP, intrinsics, baseline)

StairNml=[stairPlane(1) stairPlane(2) -1 stairPlane(3)].*sqrt(1/(1+stairPlane(1)^2+stairPlane(2)^2));

convexEdge_=struct([]);
concaveEdge_=struct([]);
lowerPlanes{1}=GP';

for i=1:length(stepPlanes)
lowerPlanes{1+i}=stepPlanes{i};
    
%compute the intersection line of the stairplane and the step plane, which
%is considerred as the convex edges

normal_uvdStep3=sqrt(1/(1+stepPlanes{i}(1)^2+stepPlanes{i}(2)^2));
StepNml=[stepPlanes{i}(1) stepPlanes{i}(2) -1 stepPlanes{i}(3)].*normal_uvdStep3;
if StepNml(4)<0; StepNml=-StepNml; end;
u=cross(StepNml(1:3), StairNml(1:3));
%u=u./norm(u);
%[umaxId umax]=max(u)
P0=[0  StairNml(4)*StepNml(3)-StepNml(4)*StairNml(3) -StepNml(2)*StairNml(4)+StairNml(2)*StepNml(4)]./u(1); %assuming that u(1)~=0
%P0=[ StepNml(2)*StairNml(4)-StairNml(2)*StepNml(4) StepNml(4)*StairNml(1)-StairNml(4)*StepNml(1)  0]./u(3);
convexEdge_(i).edgeLine=zeros(0,3);
s=(ROI(1):(ROI(1)+ROI(3))-P0(1))./u(1);
convexEdge_(i).edgeLine=[];
pt=ones(length(s),1)*P0+s'*u;
pt=pt(pt(:,2)>0&pt(:,2)<480,:);
if ~isempty(pt)
    convexEdge_(i).edgeLine=pt;
end
figure(333); hold on; 
if ~isempty(convexEdge_(i).edgeLine)
convexEdge_(i).PtL=convexEdge_(i).edgeLine(1,:);
convexEdge_(i).PtR=convexEdge_(i).edgeLine(length(convexEdge_(i).edgeLine),:);
plot(convexEdge_(i).edgeLine(:,1), convexEdge_(i).edgeLine(:,2),'LineWidth',2,'color','r');
else
    continue;
end
convexEdge_(i).theta=acot(-u(1)/u(2));
convexEdge_(i).r=(P0(1)-u(1)/u(2)*P0(2))*sin(convexEdge_(i).theta);
convexEdge_(i).u_uvd=u;
convexEdge_(i).P0_uvd=P0;

%compute the concave edges

u_xyz=intrinsics*u';
u_xyz=u_xyz/norm(u_xyz); 
Stepxyz=xyzPlane2(stepPlanes{i},intrinsics,baseline);
n_stepWall_xyz=cross(u_xyz, Stepxyz(1:3));
n2=n_stepWall_xyz/intrinsics;
n2=n2/norm(n2);
p2=convexEdge_(i).edgeLine(ceil(rand(1)*length(convexEdge_(i).edgeLine)),:);
stepWall=[n2 -p2*n2']; % the front, vertical plane of a step
if stepWall(4)<0; 
    stepWall=-stepWall; 
end 

lowerPlaneNml3=sqrt(1/(1+lowerPlanes{i}(1)^2+lowerPlanes{i}(2)^2)); 
lowerPlaneNml=[lowerPlanes{i}(1) lowerPlanes{i}(2) -1 lowerPlanes{i}(3)].*lowerPlaneNml3; 
u3=cross(stepWall(1:3), lowerPlaneNml(1:3));    
P3=[ 0  lowerPlaneNml(4)*stepWall(3)-stepWall(4)*lowerPlaneNml(3) -stepWall(2)*lowerPlaneNml(4)+lowerPlaneNml(2)*stepWall(4)]./u3(1); %assuming that u3(2)~=0
s=(ROI(1):(ROI(1)+ROI(3))-P3(1))./u3(1);
concaveEdge_(i).edgeLine=[];
pt=ones(length(s),1)*P3+s'*u3;
pt=pt(pt(:,2)>0&pt(:,2)<480,:);
if ~isempty(pt)
    concaveEdge_(i).edgeLine=pt;
end

if ~isempty(concaveEdge_(i).edgeLine)
concaveEdge_(i).PtL=concaveEdge_(i).edgeLine(1,:);
concaveEdge_(i).PtR=concaveEdge_(i).edgeLine(length(concaveEdge_(i).edgeLine),:);
plot(concaveEdge_(i).edgeLine(:,1), concaveEdge_(i).edgeLine(:,2),'LineWidth',2,'color','b');
else
    continue;
end
concaveEdge_(i).theta=acot(-u3(1)/u3(2));
concaveEdge_(i).r=(P3(1)-u3(1)/u3(2)*P3(2))*sin(concaveEdge_(i).theta);
concaveEdge_(i).u_uvd=u3;
concaveEdge_(i).P0_uvd=P3;



end
end
