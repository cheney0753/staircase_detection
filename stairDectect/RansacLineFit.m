function [L,ipt,ID]=RansacLineFit(Inpts, Dth)
NO_otl=zeros(1,1);
for i=1:(size(Inpts,1)-1)
    for j=(i+1):size(Inpts,1)

    p1=Inpts(i,:); p2=Inpts(j,:);
    Dx=p2(1)-p1(1); Dy=p2(2)-p1(2); Dxy=p1(1)*p2(2)-p2(1)*p1(2);
    DD=sqrt(Dx^2+Dy^2);
    k=0;
    for n=1:size(Inpts,1)
        dist=abs(Dy*Inpts(n,1)-Dx*Inpts(n,2)-Dxy)/DD;
        if dist<Dth;
            k=k+1;
        end
    end
    NO_otl(i,j)=k;
    %{
    if (length(intID)/length(outID))> percentage
        k=1;
        for i=1:length(intID)
            ipt(line_id,k,:)=[Inpts(intID(i),1),Inpts(intID(i),2)];
            k=k+1;
        end
    end
    %}      
   
    end
end

[jMaxVal, jMaxInd]=max(NO_otl);
[iMaxVal, iMaxInd]=max(jMaxVal);
maxInd=[jMaxInd(max(iMaxInd)),max(iMaxInd)];

p1=Inpts(maxInd(1),:); p2=Inpts(maxInd(2),:);
Dx=p2(1)-p1(1); Dy=p2(2)-p1(2); Dxy=p1(1)*p2(2)-p2(1)*p1(2);
DD=sqrt(Dx^2+Dy^2);
k=1;

ID=zeros(1,iMaxVal);
ipt=zeros(iMaxVal,2);

for n=1:length(Inpts)
    dist=abs(Dy*Inpts(n,1)-Dx*Inpts(n,2)-Dxy)/DD;
    if dist<Dth;
       ID(k)=n;
       ipt(k,:)=[Inpts(n,1),Inpts(n,2)];
       k=k+1;
    end
end

L=fit(ipt(:,1),ipt(:,2), 'poly1');
if isempty(L)
    error(['cannot find a line fits of points']);
end
end
