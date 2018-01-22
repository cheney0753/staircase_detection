function isFront_=isFrontPlane(concaveEdge_, uvdPoints_, eps, intrinsics)
edgePoints=zeros(0,3);
for i=1:length(concaveEdge_)
    if ~isempty(concaveEdge_(i).edgeLine);
    edgePoints=cat(1, edgePoints, concaveEdge_(i).edgeLine);
    end
end
edgePoints(:,1:2)=RoundUV(edgePoints(:,1:2),[640 480]);

dispPoints=zeros(0,3);
for i=1:length(edgePoints)
    dispPoints(i,:)=uvdPoints_(edgePoints(i,2),edgePoints(i,1),:);
end

xyzedgePoints=edgePoints/intrinsics;
xyzPoints=dispPoints/intrinsics;
dist=sum(abs(xyzPoints(:,3)-xyzedgePoints(:,3)));
isFront_ = dist/length(edgePoints)<eps;
end