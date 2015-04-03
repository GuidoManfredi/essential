function contours = rect2contour(rect)
    r1 = floor(rect(1));
    r2 = floor(rect(2));
    w = floor(rect(3));
    h = floor(rect(4));
    contours = [r1,r2];
    for x=1:w
        %r1+x
        contours = [contours; r1+x,r2; r1+x,r2+h];
    end
    for y=1:h
        %r2+y
        contours = [contours; r1,r2+y; r1+w,r2+y];
    end
