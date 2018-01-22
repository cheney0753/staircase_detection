%% do a RANSAC to the centers of edges with a distance threshold of the average length of edge lines
%{

%filter out short edges
l_sum=0;
for i=1:length(houghEdges)
    l_sum=vsEdgesLeft(houghEdges(i)).halfLength+l_sum;
end
lhEdge=l_sum*402.7550/length(houghEdges);
shortEdFilter=zeros(0,1);
k=1;
for i=1:length(houghEdges)
    if vsEdgesLeft(houghEdges(i)).halfLength*402.7550>lhEdge*0.5;
        shortEdFilter(k)=houghEdges(i);
        k=k+1;
    end
end

%outlier edges filter
[inlidMax, bestMid,a]=Ransac2Centroids(vsEdgesLeft( shortEdFilter));
%outlFilter=shortEdFilter(inlidMax);
outlFilter=shortEdFilter;

if length(outlFilter)<3
    fprintf('error 3 ');
    continue;
end

figure(1); hold on;
x=linspace(0,640);
y=a*(x-vsEdgesLeft(bestMid).vCt_un(1))+vsEdgesLeft(bestMid).vCt_un(2);
plot(x,y); axis([0 640 0 480]);

% see if the fitting line perpenticular to the average direction of edges
SupportEdges=outlFilter;
figure(1); hold on;
plotEdglets(vsEdgesLeft(SupportEdges),'r');
%}



%% plot all the edgelets
%{
figure(hLeft);
cla;
imshow(imLeft,[]);
hold on;
plotEdglets(vsEdgesLeft_all,'b');
%}


%% do a kmeans clusterring to the lines

tic
%{
midPt=zeros(length(vsEdgesLeft(hrzEdges)),2);
for i=1:length(vsEdgesLeft(hrzEdges))
    midPt(i,:)=vsEdgesLeft(1,hrzEdges(i)).vCt_un(1);
end

    [idMid,ctrMid]=kmeans(midPt,2);
 %}   
%for idM=1:max(idMid); % a loop for all the clusters obtained above


%% outlier filter after ransac fitting in hough space

%{
outlid=zeros(0,1);
k=1;
for i=1:length(outlFilter)
    u=vsEdgesLeft(outlFilter(i)).vCt_un(1);
    v=vsEdgesLeft(outlFilter(i)).vCt_un(2);
    d=abs(-a*u+v+a*center_roi(1)-center_roi(2))/sqrt(a^2+1);
    if d>lhEdge*1.2
       outlid(k)=i;
       outlNo=outlNo+1;
       k=k+1;
    end
end

if length( outlid)~=0
    outlFilter(outlid)=[];
end
if length( outlid)==0||length(outlFilter)<3
    break;
end

end
%}

%% calculate the curvature information 
%filter_G= fspecial('gaussian');
%disparityBlur=imfilter(disparity,filter_G);
%figure(102)
%step=5;
%plot3(u(1:step:end),-v(1:step:end),disparityBlur(1:step:end),'r.','MarkerSize',5)

[K,H,Pmax,Pmin]= surfature(u,v,disparity);
CI=(abs(Pmax)-abs(Pmin)).^2./((abs(Pmax)+abs(Pmin)).^2+0.01);
H(abs(H)>0.05)=0;
CI(abs(CI)>0.85)=0;
figure(100);
imshow(CI);
figure(101);
surf(H);
%{
H_stairEdges=zeros(0,0);
for l=1:length(outlFilter)
    inD=round(vsEdgesLeft(outlFilter(l)).vPts_un)';
    sumH=0;
    for p=1:length(inD)
    sumH= sumH+H(inD(p,2),inD(p,1));
    end
    H_stairEdges(l)=sumH;
end

concaveEdges=outlFilter(H_stairEdges<0);
convexEdges=outlFilter(H_stairEdges>0|H_stairEdges==0);
%}

CI_stairEdges=zeros(0,0);
for l=1:length(outlFilter)
    inD=round(vsEdgesLeft(outlFilter(l)).vPts_un)';
    sumCI=0;
    for p=1:length(inD)
    sumCI= sumCI+CI(inD(p,2),inD(p,1));
    end
    CI_stairEdges(l)=sumCI/length(inD);
end

concaveEdges=outlFilter(CI_stairEdges<0.05);
convexEdges=outlFilter(CI_stairEdges>0.05);
figure(3+10*NumStairPlane);
imshow(imLeft,[]);
hold on;
plotEdglets(vsEdgesLeft(concaveEdges),'r');
plotEdglets(vsEdgesLeft(convexEdges),'g');

%% find the intersecting points
%{
InT_length=length(vsEdgesLeft)*length(vsEdgesLeft);
InT=zeros(InT_length,9);
k=1;
for i=1:length(vsEdgesLeft);
    th1=hough_curr(i,1);
    r1=hough_curr(i,2);
    for j=1:length(vsEdgesLeft);
        th2=hough_curr(j,1);
        r2=hough_curr(j,2);
        if(abs(th2-th1)<0.1 && abs(th2-th1)~=0)
        InT(i*j,1)=(r2/cos(th2)-r1/cos(th1))/(tan(th2)-tan(th1));
        InT(i*j,2)=InT(i*j,1)*(-tan(th1))+r1/cos(th1);
        InT(i*j,3)=th1;
        InT(i*j,4)=r1;
        InT(i*j,5)=th2;
        InT(i*j,6)=r2;
        InT(i*j,7)=abs(th1-th2);
        InT(i*j,8)=i;
        InT(i*j,9)=j;
        end
 %       if(abs(InT(i*j,1))>10000||abs(InT(i*j,2))>10000)
 %           InT(i*j,1)=0;
 %           InT(i*j,2)=0;
 %       end
   % end
end
InT2_found=find(InT(:,1)~=0);
InT2=zeros(length(InT2_found),9);
for i=1:length(InT2_found)
    InT2(i,:)=InT(InT2_found(i,1),:);
end

   
%figure(31); hold on;
%plot(InT(:,1),InT(:,2),'.','MarkerSize',10);

 
%axis([400 600 300 600]);
%}

%{
%% do a RANSAC line fitting to the intecetions
IPs=InT2(:,1:2);
id= 1:length(InT2(:,1:2)); % Return the id of intersection points in matrix InT2, 
                        %so that we know which IPs they are
IPs=[IPs,id'];
IntcLines=RansacLineFit(IPs,20,0.3); 
figure(31); hold on;
plot(InT2(:,1),-InT2(:,2),'.','MarkerSize',10);

hold on;
figure(hLeft); hold on;
PsLength=zeros(1,1);
for i=1:length(IntcLines)
    PsLength(i)=length(IntcLines(i).Ps);
end
[mval, PsMax]=max(PsLength);
if length(PsMax)>1
    PsMax=PsMax(1);
end
    %idMin=find(IntcLines(PsMax).Ps(:,1)==min(IntcLines(PsMax).Ps(:,1)));
%idMax=find(IntcLines(PsMax).Ps(:,1)==max(IntcLines(PsMax).Ps(:,1)));
%x=linspace(IntcLines(PsMax).EdgesStairPs(idMin,1),IntcLines(PsMax).Ps(idMax,1));
   % plot the direction of the stairs 
    x=linspace(1,639);
    p1=IntcLines(PsMax).p1;
    p2=IntcLines(PsMax).p2;
    a=(p1(2)-p2(2))/(p1(1)-p2(1));
    b=(p1(2)*p2(1)-p2(2)*p1(1))/(p2(1)-p1(1));
    y=a*x+b;
    plot(x,y);
%% use k mean clustering to cluster the intersection points into 2 clusters
%PtX=[InT2(IntcLines(PsMax).id(:),1),-InT2(IntcLines(PsMax).id(:),2)];
%[idX,ctrs] = kmeans(PtX,2);
%figure(32); hold on;
%plot(PtX(idX==1,1),PtX(idX==1,2),'g.','MarkerSize',10);

%% find the vanishing point of the lines returned by last RANSAC step
hough_Steps=zeros(length(IntcLines),2);

%idLines=[InT2(IntcLines(PsMax).id(idX==mode(idX)),8),InT2(IntcLines(PsMax).id(idX==mode(idX)),9)];    
idLines=[InT2(IntcLines(PsMax).id(:),8),InT2(IntcLines(PsMax).id(:),9)];
idLines=unique(idLines); % the lines that intersects on the fitted line


    Sxx=sum(cos(hough_curr(idLines,1)).^2);
    Sxy=sum(sin(hough_curr(idLines,1)).*cos(hough_curr(idLines,1)));
    Syy=sum(sin(hough_curr(idLines,1)).^2);
    Sxz=sum(cos(hough_curr(idLines,1)).*(-hough_curr(idLines,2)));
    Syz=sum(sin(hough_curr(idLines,1)).*(-hough_curr(idLines,2)));

    VP=zeros(2,1); % the vanishing point for the lines returned 
    VP(1,1)=(Syz*Sxy-Sxz*Syy)/(Sxx*Syy-Sxy^2);
    VP(2,1)=(-Syz*Sxx+Sxz*Sxy)/(Sxx*Syy-Sxy^2);
    figure(32); hold on;
    plot(InT2(IntcLines(PsMax).id(:),1),-InT2(IntcLines(PsMax).id(:),2),'.','MarkerSize',10);
    plot(VP(2,1),-VP(1,1),'*','MarkerSize', 20);
    %plot(PtX(idX==mode(idX),1),PtX(idX==mode(idX),2),'g.','MarkerSize',10);


%% find out the stair edges points at the same direction as the fitted line
    thStair=atan(a);
    rStair=b*sin(thStair);
    thStair=180*thStair/pi;
    k=1;
    for i=1:length(idLines)
        %if(abs(vsEdgesLeft(idLines(i)).theta-thStair)<2);
           EdgesStair(k)= vsEdgesLeft(1,idLines(i)); 
           k=k+1;
        %end
    end

    borderLeft=zeros(length(EdgesStair),2);
    borderRight=zeros(length(EdgesStair),2);
    for i=1:length(EdgesStair)
        borderLeft(i,:)=EdgesStair(i).vPointUn2;
        borderRight(i,:)=EdgesStair(i).vPointUn1;
    end
        StLeft=borderLeft(find(borderLeft(:,2)==min(borderLeft(:,2))),:);
        StRight=borderRight(find(borderRight(:,2)==max(borderRight(:,2))),:);
    dim=[StLeft(1), StLeft(2), abs(StRight(1)-StLeft(1)), abs(StRight(2)-StLeft(2))];
    figure(hLeft); hold on;
    r=rectangle('Position',dim,'LineWidth',2);
    set(r,'edgecolor','y');
    plotEdglets(EdgesStair,'g');
%}


%% gabor filter

%imLeft = im2uint8(edge(imLeftOrig,'canny'));

%imRight= imread(sprintf(rightImgStr,frameNo));
%imRight= edge( imRightOrig, 'canny');

%read the Gabor filter
%gaborArray = gaborFilterBank(1,1,39,39);
%gaborResult=gaborFeatures(imLeft,gaborArray,1,1);
%imGaborLeft=im2uint8(abs(gaborResult{1,1}));
%imshow(imGaborLeft);