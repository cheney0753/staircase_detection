function [dimR, center_roi]=rect_Stair(vsEdgesLeft)
% returns a rectangular representing the rigion of interest given by a
% group of interesting edgelets.

sum_vPts=0;
sum_u=0;
sum_v=0;
for i=1:length(vsEdgesLeft)
    sum_vPts=sum_vPts+length(vsEdgesLeft(i).vPts_un);
    
        sum_u=sum_u+sum(vsEdgesLeft(i).vPts_un(1,:));
        sum_v=sum_v+sum(vsEdgesLeft(i).vPts_un(2,:));
    
end
center_roi=[sum_u/sum_vPts,sum_v/sum_vPts];
sum_w=0;
sum_h=0;
for i=1:length(vsEdgesLeft)
    sum_w=sum_w+sum(abs(vsEdgesLeft(i).vPts_un(1,:)-center_roi(1)));
    sum_h=sum_h+sum(abs(vsEdgesLeft(i).vPts_un(2,:)-center_roi(2)));
end
dim_roi= [2*sum_w/sum_vPts,2*sum_h/sum_vPts];
dimR=[center_roi(1)-dim_roi(1),center_roi(2)-dim_roi(2),dim_roi(1)*2,dim_roi(2)*2];
dimR=floor(dimR);
if dimR(1)<1;dimR(1)=1; end
if dimR(2)<1;dimR(2)=1; end
if (dimR(1)+dimR(3))>639; dimR(3)=639-dimR(1); end
if (dimR(2)+dimR(4))>479; dimR(4)=479-dimR(2); end
end