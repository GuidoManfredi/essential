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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Charger le cloud et remplir une voxelgrid avec.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cloud = load('../cloud.txt');
cloud = single(cloud);

mx = (min(cloud(:,1)) - 0.025)/ 0.05;
Mx = (max(cloud(:,1)) - 0.025)/ 0.05;
My = (max(cloud(:,2)) - 0.025)/ 0.05;
my = (min(cloud(:,2)) - 0.025)/ 0.05;

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
min_idx = find(H == min(H));
max_idx = size(H,2);
sol = VM(:,:,1);
pieds = VM(:,:,4); % 20cm du sol
petit = VM(:,:,min_idx);
murs = VM(:,:,max_idx) | VM(:,:,max_idx-1) | VM(:,:,max_idx-2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute window and door zones.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
se = strel('square', 2);
dil_murs = imdilate(murs, se);

se = strel('square', 3);
dil_petit = imdilate(petit, se);

holes = dil_murs - dil_petit;
se = strel('square', 3);
er_holes = imerode(holes, se);
se = strel('square', 3);
dil_holes = imdilate(er_holes, se);

%{
figure;
subplot(2,2,1);
imshow(dil_murs);
subplot(2,2,2);
imshow(dil_petit);
subplot(2,2,3);
imshow(er_holes);
subplot(2,2,4);
imshow(dil_holes);
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find areas and get contours
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%figure;
%imshow(dil_murs);
stats = regionprops(~dil_murs,'BoundingBox');

cnt = 1;
for i=1:size(stats)
    if (stats(i).BoundingBox(3) * stats(i).BoundingBox(4) > 400) % 1m² = 400 pixels²
        bbs{cnt} = stats(i).BoundingBox;
        contours{cnt} = rect2contour(stats(i).BoundingBox);
        %rectangle('Position',stats(i).BoundingBox,'LineWidth',3, 'EdgeColor', 'r'); % display
        %displ = coords2image(contours{cnt});
        cnt = cnt + 1;
    end
end
% Adding the exterior as a room
s = size(dil_murs);
bbs{cnt} = [1 1 s(2)-1 s(1)-1];
contours{cnt} = rect2contour(bbs{cnt});
%rectangle('Position',bbs{cnt},'LineWidth',3, 'EdgeColor', 'r'); % display
%displ = coords2image(contours{cnt});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For each areas classify contour points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
imshow(dil_petit); hold on;
%for i=1:cnt-1
for i=1:3
    X = zeros(1,1);
    lines = getRoomLines(i, contours{i}, contours, dil_petit);

    cnt = 1;    
    for j=1:length(lines)
        if lines{j}.label == 0
            windoor_lines{cnt} = lines{j};
            cnt = cnt + 1;
        end
    end

    cum_lines = getCumLines(VM, windoor_lines);

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
    %{
    % display walls
    for i=1:length(lines)
        if lines{i}.label == 3
            pt = lines{i}.line(1,:);
            plot(pt(1),pt(2),'Marker','p','Color','b','MarkerSize',5);
        end
    end
    %}
    %{
    % display windows and doors
    for i=1:length(C)
        pt = windoor_lines{i}.line(1,:);
        if C(i) == 1
            plot(pt(1),pt(2),'Marker','p','Color','r','MarkerSize',5);
        else
            plot(pt(1),pt(2),'Marker','p','Color','g','MarkerSize',5);
        end
    end
    %}
    
    segments = getSegments(windoor_lines);
    for j=1:length(segments)
        k = size(segments{j}.seg,1);
        if segments{j}.label == 1
            color = 'r';
        elseif segments{j}.label == 2
            color = 'b';
        else
            color = 'm';
        end
        plot([segments{j}.seg(1,1) segments{j}.seg(k,1)],[segments{j}.seg(1,2) segments{j}.seg(k,2)],'Color',color,'LineWidth',3);
        segments{j}.to
    end
    bouarg =1
end

%TODO
% Attention ! Le champ "to" des segments est remplie a l'arrache (pas de vote)
% A partir des segments obtenir une arrete classifiée entre deux rooms
% Creer graph topologique




