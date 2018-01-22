function draw3DLine(p, u, ROI, fHandle)
s1=(ROI(1)-p(1))./u(1);
s2=(ROI(1)+ROI(3)-p(1))./u(1);

p1=p+s1.*u;
p2=p+s2.*u;

figure(fHandle); 
hold on;

plot( [p1(1) p2(1)], [p1(2) p2(2)], 'y');

hold off;

end