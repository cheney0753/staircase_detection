function plane = fitPlaneTo3Points(p0,p1,p2)

    normal = cross(p1-p0,p2-p0);

    len = sqrt(normal'*normal);

    if len < 1e-10
        %error('singular point configuration in fitPlaneTo3Points');
        plane = [0 0 0 0];
    end

    plane(1) = normal(1)/len;
    plane(2) = normal(2)/len;
    plane(3) = normal(3)/len;
    plane(4) = -(p0'*normal)/len;

    if(plane(4)<0)
        plane = -plane;
    end

end