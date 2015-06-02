%
% TODO
% plus hauts HZ = plafond = murs + portes + fenetres
% minimum histogrqm MZ = murs
% plus bas BZ = sol
% occupé juste au dessus du sol AS = pas portes
% Trouver portes + fenetres = HZ - MZ
% HZ - MZ - AS = portes
% When robot go for an area and see it does not exists, it removes it from its map

close all; clear all;
visible = 'off';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Charger le cloud et remplir une voxelgrid avec.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cloud = load('../cloud.txt');
cloud = single(cloud);

mx = (min(cloud(:,1)) - 0.025)/ 0.05;
Mx = (max(cloud(:,1)) - 0.025)/ 0.05;
My = (max(cloud(:,2)) - 0.025)/ 0.05;
my = ceil((min(cloud(:,2)) - 0.025)/ 0.05);

VM = single(cloud2voxelmap(cloud));
VM = VM(1:Mx-25, my:My, :); % -25cm to remove exterior of appartment

%c = getColumn(VM, 10, 100);
%C = getAllColumns(VM);

%sliceXY = VM(:,:,i);

%{
figure;
for i=1:44
    subplot(9,5,i);
    imshow((VM(:,:,i)));
end
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute position of sol, murs et petite slice.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Z = cloud(:,3);
[H,bins] = hist(Z, 44);
%{
% display
p0 = figure('Visible',visible);
bar(H);
font_size = 18;
xlabel('Voxel index along Z (x0.05 for height)','FontSize',font_size);
ylabel('Number of occupied voxels','FontSize',font_size);
tightfig;
print(p0,'-dpng','/home/gmanfred/Desktop/tmp_sandra/height_histogram');
%}
min_idx = find(H == min(H)); % min_idx = 33
max_idx = size(H,2);
sol = VM(:,:,1) | VM(:,:,2);
pieds = VM(:,:,4); % 20cm du sol
petit = VM(:,:,min_idx);
murs = VM(:,:,max_idx) | VM(:,:,max_idx-1) | VM(:,:,max_idx-2);
tout = zeros(size(murs));
for i=5:max_idx
    tout = tout | VM(:,:,i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute window and door zones.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
se = strel('square', 2);
dil_murs = imdilate(murs, se);

se = strel('square', 3);
dil_petit = imdilate(petit, se);

%{
holes = dil_murs - dil_petit;
se = strel('square', 3);
er_holes = imerode(holes, se);
se = strel('square', 3);
dil_holes = imdilate(er_holes, se);
%}

p1bis=figure('Visible',visible);
imshow(sol);
tightfig;
print(p1bis,'-dpng','/home/gmanfred/Desktop/tmp_sandra/floor_slice');
p1=figure('Visible',visible);
imshow(dil_murs);
tightfig;
print(p1,'-dpng','/home/gmanfred/Desktop/tmp_sandra/walls_slice');
p2=figure('Visible',visible);
imshow(dil_petit);
tightfig;
print(p2,'-dpng','/home/gmanfred/Desktop/tmp_sandra/minimum_slice');
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find areas and get contours
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3=figure('Visible',visible);
imshow(dil_murs);
[bbs contours] = getRooms(dil_murs);
tightfig;
print(p3,'-dpng','/home/gmanfred/Desktop/tmp_sandra/rooms');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For each areas classify contour points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p4=figure('Visible',visible);
imshow(dil_petit); hold on;
for i=1:length(bbs)-1
    X = zeros(1,1);
    lines = getRoomLines(i, contours{i}, contours, dil_petit);

    tmp = 1;    
    for j=1:length(lines)
        if lines{j}.label == 0
            % display
            %{
            if mod(j,5) == 0
                plot([lines{j}.line(1,1) lines{j}.line(end,1)],[lines{j}.line(1,2) lines{j}.line(end,2)],'Color','g','LineWidth',2);
            end
            %}
            windoor_lines{tmp} = lines{j};
            tmp = tmp + 1;
        end
    end

    cum_lines = getCumLines(VM, windoor_lines);
    % display
    %{
    pDescsWin=figure('Visible',visible);
    plot(cum_lines{4},'LineWidth',4) %
    axis([1,44,-0.2,1.2]);
    font_size = 18;
    xlabel('Height','FontSize',font_size);
    ylabel('Occupied','FontSize',font_size);
    tightfig;
    print(pDescsWin,'-dpng','/home/gmanfred/Desktop/tmp_sandra/descriptor_window');
    
    pDescsDoor=figure('Visible',visible);
    plot(cum_lines{20},'LineWidth',4) %
    axis([1,44,-0.2,1.2]);
    font_size = 18;
    xlabel('Height','FontSize',font_size);
    ylabel('Occupied','FontSize',font_size);
    tightfig;
    print(pDescsDoor,'-dpng','/home/gmanfred/Desktop/tmp_sandra/descriptor_door');
    %}
    
    % cummulative lines to binary matrix
    D = single(cell2mat(cum_lines));
    for j=1:length(D)
        X(j) = nnz(D(:,j)); % norme 0
    end
    % classify
    C = clusterdata(X','maxclust',2,'distance','euclidean','linkage','median');
    for c=1:length(C)
        windoor_lines{c}.label = C(c);
    end
    %
    % display walls
    %{
    for j=1:length(lines)
        if lines{j}.label == 3
            pt = lines{j}.line(1,:);
            plot(pt(1),pt(2),'Marker','p','Color','b','MarkerSize',5);
        end
    end
    %}
    % display windows and doors
    %{
    for i=1:length(C)
        pt = windoor_lines{i}.line(1,:);
        if windoor_lines{i}.label == 1
            plot(pt(1),pt(2),'Marker','p','Color','r','MarkerSize',5);
        else
            plot(pt(1),pt(2),'Marker','p','Color','g','MarkerSize',5);
        end
    end
    tightfig;
    print(p4,'-dpng','/home/gmanfred/Desktop/tmp_sandra/classification_init');
    %}
    
    segments = getSegments(windoor_lines);
    % display
    %{
    for j=1:length(segments)
        k = size(segments{j}.seg,1);
        if segments{j}.label == 1
            color = 'r';
        elseif segments{j}.label == 2
            color = 'g';
        else
            color = 'm';
        end
        plot([segments{j}.seg(1,1) segments{j}.seg(k,1)],[segments{j}.seg(1,2) segments{j}.seg(k,2)],'Color',color,'LineWidth',3);
    end
    tightfig;
    print(p4,'-dpng','/home/gmanfred/Desktop/tmp_sandra/classification_final');
    %}
    %
    segments2dot(segments, i);
end
tightfig;
%print(p4,'-dpng','/home/gmanfred/Desktop/tmp_sandra/bridges');
print(p4,'-dpng','/home/gmanfred/Desktop/tmp_sandra/classification_final');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find planar areas
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for b=1:length(bbs)-1
    bb = bbs{b};
    y = (floor(bb(2)) * 0.05) + 0.025;
    x = (floor(bb(1)) * 0.05) + 0.025;
    h = (floor(bb(4)) * 0.05) + 0.025;
    w = (floor(bb(3)) * 0.05) + 0.025;
    
    idx = find( (cloud(:,2) > x) & (cloud(:,2) < x+w) ...
              & (cloud(:,1) > y) & (cloud(:,1) < y+h) );
    room_cloud = (cloud(idx,:));
    room_voxmap = single(cloud2voxelmap(room_cloud));
    
    Z = room_cloud(:,3);
    [H,bins] = hist(Z, 44);
    [pks,pidx] = findpeaks(H);
    
    % display
    pLocMax = figure('Visible',visible);
    bar(H);
    font_size = 18;
    xlabel('Voxel index along Z (x0.05 for height)','FontSize',font_size);
    ylabel('Number of occupied voxels','FontSize',font_size);
    tightfig;
    print(pLocMax,'-dpng',['/home/gmanfred/Desktop/tmp_sandra/local_maxima_histogram' num2str(b)]);
    pidx

    p5=figure('Visible',visible);
    % find planes
    for p=1:length(pks)
        idx = pidx(p);
        subplot(2,ceil(length(pks)/2),p);
        rooms{p} = room_voxmap(1:Mx-25,my:My,idx) & ~dil_murs; % remove walls
        imshow(rooms{p});
    end
    tightfig;
    print(p5,'-dpng',['/home/gmanfred/Desktop/tmp_sandra/slices' num2str(b)]);
    % merge planes
    counter = 1;
    final_rooms{1} = rooms{1};
    final_pidx{1} = pidx(1);
    for p=2:length(pks)
        if (pidx(p) - pidx(p-1)) <= 4 % 20cm between the planes
            final_rooms{counter} = final_rooms{counter} | rooms{p};
            pidx(p) = (pidx(p) + pidx(p-1))/2;
            final_pidx{counter} = pidx(p);
        else
            counter = counter + 1;
            final_pidx{counter} = pidx(p);
            final_rooms{counter} = rooms{p};
        end
    end
    %display
    p6=figure('Visible',visible);
    for t=1:length(final_rooms)
        subplot(2,ceil(length(final_rooms)/2),t);
        imshow(final_rooms{t});
    end
    %length(final_rooms)
    print(p6,'-dpng',['/home/gmanfred/Desktop/tmp_sandra/merged_slices' num2str(b)]);
    all_planes{b} = final_rooms;
    all_pidx{b} = final_pidx;
end

interest_areas = areasFromPlanes(all_planes, all_pidx);

%
% display
length(interest_areas)
for l=1:length(interest_areas)
    test = [interest_areas{l}(1) interest_areas{l}(2) interest_areas{l}(3) interest_areas{l}(4)];
    figure;
    imshow(tout); hold on;
    rectangle('Position',test,'LineWidth',3, 'EdgeColor', 'r');
end
%
%plot(meter2pixel(2.25),meter2pixel(5.85),'Marker','p','Color','r','MarkerSize',5);

%length(interest_areas)
%display
%{
p7=figure('Visible',visible);
imshow(tout);
for i=1:length(interest_areas)
    rectangle('Position',interest_areas{i},'LineWidth',3, 'EdgeColor', 'r'); % display
end
print(p7,'-dpng',['/home/gmanfred/Desktop/tmp_sandra/interest_areas']);
%}

saveAreas(interest_areas);
%{
[H,bins] = hist(Z, 44);
[pks,idx] = findpeaks(H);
size(cloud)
for b=1:cnt-1
    figure;    
    for p=1:length(pks)
        bb = bbs{b};
        pk = pks(p);
        subplot(2,2,p);
        no_walls = VM(:,:,idx(p)) - dil_murs;
        x = floor(bb(2));
        y = floor(bb(1));
        w = floor(bb(4));
        h = floor(bb(3));
        room = no_walls(x:x+w-1,y:y+h-1);
        imshow(room);
    end
end
%}

%TODO
% Generer toutes les images !
    % Faire images avec les droites liant deux point du contour d'une region
    % Faire images avec les features utilisées dans la classif
    % Plot histogram to show peaks.
% Sauvegarder les interest_areas dans un fichiers
% Faire code du robot
    % LookAt.
    % Scan (move head from ... to ...).
    % Go and see (cf sushi challenge).
    % Detection objets.
    % Find next area to explore (start with one and find closest in a greedy way)
% Remettre appartement dans l'état ou il était avant !
% Filmer !
