function  [inlidMax, bestMid, a]=Ransac2Centroids(vsEdgesLeft)
%inlidMax gives the id of inliers in vsEdgesLeft
%bestMid gives the best centroid among those of vsEdgesLeft
inlNoMax=0;
inlidMax=zeros(0,0);
bestMid=0;

thSum=0;edgecenters=zeros(0,2);
for i=1:length(vsEdgesLeft)
    edgecenters(i,:)=vsEdgesLeft(i).vCt_un;
    thSum=thSum+vsEdgesLeft(i).theta;
end
thAve=thSum/length(vsEdgesLeft); 
a=-cot(thAve*pi/180); 

l_sum=0;
for i=1:length(vsEdgesLeft)
   l_sum=vsEdgesLeft(i).halfLength+l_sum;
end


for i=1:length(vsEdgesLeft)
    u0=vsEdgesLeft(i).vCt_un(1);
    v0=vsEdgesLeft(i).vCt_un(2);
    inlNo=0;
    inlid=zeros(0,0);

    lhEdge=l_sum*402.7550/length(vsEdgesLeft);
    for j=1:length(vsEdgesLeft)
       u=vsEdgesLeft(j).vCt_un(1);
       v=vsEdgesLeft(j).vCt_un(2);
       d=abs(-a*u+v+a*u0-v0)/sqrt(a^2+1);
       if d<lhEdge
           inlNo=inlNo+1;
           inlid(inlNo)=j;
       end
    end
    if inlNo>inlNoMax
        inlNoMax=inlNo;
        inlidMax=inlid;
        bestMid=i;
    end
end
end