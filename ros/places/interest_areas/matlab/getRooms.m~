function [rooms,contours] = getRooms(murs)
stats = regionprops(~murs,'BoundingBox');

cnt = 1;
for i=1:size(stats)
    if (stats(i).BoundingBox(3) * stats(i).BoundingBox(4) > 400) % 1m² = 400 pixels²
        rooms{cnt} = stats(i).BoundingBox;
        contours{cnt} = rect2contour(stats(i).BoundingBox);
        rectangle('Position',stats(i).BoundingBox,'LineWidth',3, 'EdgeColor', 'r'); % display
        cnt = cnt + 1;
    end
end
% Adding the exterior as a room
s = size(murs);
rooms{cnt} = [1 1 s(2)-1 s(1)-1];
contours{cnt} = rect2contour(rooms{cnt});
rectangle('Position',rooms{cnt},'LineWidth',3, 'EdgeColor', 'r'); % display
