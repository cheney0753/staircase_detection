function  distance=distanceSegm( p10, p11, p20, p21)
%ref: http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment%28%29
SMALL_NUM=1e-4;
u=p11-p10;
v=p21-p20;
w=p10-p20;

a=u*u';
b=u*v';
c=v*v';
d=u*w';
e=v*w';
D=a*c-b*b;
sD=D; tD=D;
if (D< SMALL_NUM)
    sN = 0;         
    sD = 1;         
    tN = e;
    tD = c;
else 
    sN = (b*e - c*d);
	tN = (a*e - b*d);
    if (sN<0)
        sN = 0;
        tN = e;
        tD = c; 
    else if (sN>sD)
        sN = sD;
        tN = e + b;
        tD = c;
        end
    end
end

if (tN<0.0)
    tN = 0.0;
    if (-d < 0.0)
        sN = 0.0;  
    else
        if (-d > a)
            sN = sD;
        else 
            sN = -d;
            sD = a;
        end
    end
    
else 
    if(tN > tD)
        tN = tD;
      if ((-d + b) < 0.0)
          sN = 0;
      else
          if ((-d + b) > a)
              sN = sD;
          else
              sN = (-d +  b);
              sD = a;
          end
      end
    end
end

if abs(sN) < SMALL_NUM
    sc=0;
else
    sc=sN/sD;
end

if abs(tN) < SMALL_NUM
    tc=0;
else
    tc=tN/tD;
end

dP=w+(sc*u)-(tc*v);

distance=norm(dP);
end 