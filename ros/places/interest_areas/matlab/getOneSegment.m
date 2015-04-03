function S = getOneSegment(start, finish, lines)
    S = struct();
    category = zeros(1,2);
    to = zeros(1,4);
    tmp=1;
    seg = zeros(finish-start,2);
    for n=start:finish
        seg(tmp,:) = lines{n}.line(1,:); % retrieve first point
        category(lines{n}.label) = category(lines{n}.label) + 1; % upvote for a category
        to(lines{n}.to) = to(lines{n}.to) + 1;
        tmp = tmp + 1;
    end
    
    [M, Midx] = max(category);
    [m, midx] = min(category);
    if (m < 0.75 * M) % if a lot of this category
        S.label = Midx;
    else
        S.label = -1;
    end
    S.seg = seg;
    [M S.to] = max(to);
    
    
