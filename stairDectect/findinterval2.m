function [inliers_locs, interval, inliers_ID]=findinterval2(locs)
step=1:length(locs);
pts=[step', locs'];
inliersMax=[];
Num_inliersMax=0;
inliersIDMax=[];
Dth=0.1;
it=0;
while (true)
    id1=ceil(rand(1)*length(pts));
    id2=id1;
    while(id2==id1)
        id2=ceil(rand(1)*length(pts));
    end
    p1=pts(id1(1,1),:); p2=pts(id2(1,1),:);
    Dx=p2(1)-p1(1); Dy=p2(2)-p1(2); Dxy=p1(1)*p2(2)-p2(1)*p1(2);
    DD=sqrt(Dx^2+Dy^2);
    Num_inliers=0;
    inliers=[];
    inliersID=[];
    for i=1:length(pts)
        dist=abs(Dy*pts(i,1)-Dx*pts(i,2)-Dxy)/DD;
        if dist<Dth;
            Num_inliers=Num_inliers+1;
            inliers(Num_inliers,:)=pts(i,:);
            inliersID(Num_inliers)=i;
        end
    end

    if  Num_inliersMax<Num_inliers
        Num_inliersMax=Num_inliers;
        inliersMax=inliers;
        inliersIDMax=inliersID;
    end
    
    it=it+1;
    if it>50; break; end
    if length(inliersMax)/length(pts)>0.95; break; end
end
    L=fit(inliersMax(:,1),inliersMax(:,2),'poly1');
    interval=L.p1;
    stepsFound=inliersMax(:,2);
    steps=stepsFound;
    %{
    steps_missed=zeros(0,2); k=1;
    for i=1:max(steps(:,1))
        if  isempty(find(steps(:,1)==i,1))
            steps_missed(k,:)=[i i*interval];
            k=k+1;
        end
    end
    steps=cat(1,steps,steps_missed); 
    steps=steps(steps(:,2)>interval/2,:); %remove the ground plane
    %}
    inliers_locs=steps; %sort the steps in an ascending order
    inliers_ID=inliersIDMax;
    
    figure(99);
	clf
    hold on;
    plot(step,locs,'b*');
    linex=0.5:1:(0.5+length(step));
    liney=linex*L.p1+L.p2;
    plot(linex, liney, 'r-')
    xlabel('Index');
    ylabel('Peak position');
    hleg1 = legend('Input points','Fitted line');
     set(hleg1,'Location','NorthWest');
    set(hleg1,'Interpreter','none');
    hold off;

end