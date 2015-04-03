function lines = getRoomLines(room_num, room, rooms, small_map)
    i = 1;
    for r=room_num:length(rooms)
    %for r=1:length(rooms)
        for p=1:length(room)
            L = struct();
            pt1= room(p,:);
            pt2 = findClosestPoint(pt1, rooms{r});
            [lx ly] = bresenham(pt1(1),pt1(2),pt2(1),pt2(2));
            l = [lx ly];
            % For lines big enough
            if size(l,1) ~= 1
                % If there is no intersection with the own room,
                % other than starting point of line.
                m = inter(l, room);
                if m == 1
                    % If the line does not go across a wall
                    n = interMap(l, small_map);
                    if n == 0
                        L.label = 0;
                    else
                        L.label = 3;
                    end           
                    L.from = room_num;
                    L.to = r;
                    L.line = l;
                    lines{i} = L;
                    i = i + 1;
                end
            end
        end
    end




