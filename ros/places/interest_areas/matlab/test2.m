%
close all; clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Charger le cloud et remplir une voxelgrid avec.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cloud = load('../cloud.txt');
cloud = single(cloud);

mx = round((min(cloud(:,1)) - 0.025)/ 0.05) + 1;
Mx = round((max(cloud(:,1)) - 0.025)/ 0.05) + 1;
My = round((max(cloud(:,2)) - 0.025)/ 0.05) + 1;
my = round((min(cloud(:,2)) - 0.025)/ 0.05) + 1;
VM = single(cloud2voxelmap(cloud));
VM = VM(1:Mx-25, my:My, :);

map = find_topology(cloud, VM);
