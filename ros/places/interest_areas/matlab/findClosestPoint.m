function p2 = findClosestPoint(p1, contour_pts)
    distances = sqrt((contour_pts(:,1)-p1(1)).^2 + (contour_pts(:,2)-p1(2)).^2); % Distance to adjacent points
    [m, idx] = min(distances); % closest point from room
    p2 = contour_pts(idx,:);