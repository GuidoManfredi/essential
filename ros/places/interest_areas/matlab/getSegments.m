function segs = getSegments(lines)
    cnt = 1;
    % get start and finish of segment
    start = 1;
    category = zeros(1,2);
    source_dest = zeros(1,2);
    for i=2:length(lines)
        pt = [lines{i-1}.line(1,:);lines{i}.line(1,:)];
        % if current point not adjacent, we have end of segment
        if (pdist(pt,'euclidean') > 1.0)
            finish = i-1;
            seg = getOneSegment(start, finish, lines);
            segs{cnt} = seg;        
            cnt = cnt + 1;
            start=i;
        % if reach last point, save the segment
        elseif i==length(lines)
            finish = i-1;
            seg = getOneSegment(start, finish, lines);
            segs{cnt} = seg;
        end
    end
