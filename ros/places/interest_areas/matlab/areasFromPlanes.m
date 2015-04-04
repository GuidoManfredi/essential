function areas = areasFromPlanes(planes)
se = strel('square', 4);
cnt = 1;
areas = cell(1);
for a=1:length(planes)
    for p=1:length(planes{a})
        plane = planes{a}{p};
        closed = imclose(plane,se);
        stats = regionprops(closed,'BoundingBox');
        for s=1:length(stats)
            if (stats(s).BoundingBox(3) * stats(s).BoundingBox(4) > 40) % 0.1m² = 40 pixels²
                areas{cnt} = stats(s).BoundingBox;
                cnt = cnt + 1;
            end
        end
    end
end
