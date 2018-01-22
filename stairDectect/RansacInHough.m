function [L,iPt]=RansacInHough(Pt, Dth, Thr)

% Pt is the input N*2 matrix of input points. Dth is the Euclidian distance 
% tolarence. Thr is the threshold for number of inliers.
% it return a 2*1 matrix of L, which is [a,b] which defines that line
% y=a*x+b;
% as well as iPt, which is the inliers. A M*2 matrix.
iPt=zeros(0,2);
it=1;
while true
    id1=ceil(rand(1)*length(Pt));
    id2=id1;
    while(id2==id1)
        id2=ceil(rand(1)*length(Pt));
    end
    p1=Pt(id1(1,1),:); p2=Pt(id2(1,1),:);
    Dx=p2(1)-p1(1); Dy=p2(2)-p1(2); Dxy=p1(1)*p2(2)-p2(1)*p1(2);
    DD=sqrt(Dx^2+Dy^2);
    p=0;
    for i=1:length(Pt)
        dist=abs(Dy*Pt(i,1)-Dx*Pt(i,2)-Dxy)/DD;
        if dist<Dth;
            p=p+1;
        end
    end
    if (p/length(Pt))> Thr
        k=1;
        for i=1:length(Pt)
            dist=abs(Dy*Pt(i,1)-Dx*Pt(i,2)-Dxy)/DD;
            if dist<Dth;
                iPt(k,:)=[Pt(i,1),Pt(i,2)];
                k=k+1;
            end
        end
        L=fit(iPt(:,1),iPt(:,2),'poly1');
        break;
    end
    
   it=it+1;
   if it>5000
       L=[];
       break;
   end
end
end