%
function C = findClusters (D)

D = single(D);

%T = clusterdata(D,'maxclust',4,'distance','emd','linkage','average'); % All white except floor
%T = clusterdata(D,'maxclust',4,'distance','jaccard','linkage','weighted'); % All white except floor
%T = clusterdata(D,'maxclust',4,'distance','hamming','linkage','weighted'); % Walls black and pieces of white
%T = clusterdata(D,'maxclust',4,'distance','hamming','linkage','average'); % Pieces of wall white
C = clusterdata(D,'maxclust',3,'distance',@DiscretFrechetDist,'linkage','average'); % All white except floor


%figure;
%imshow(T2, [1,4]);

%{
n = 7;
n2 = n * n;
figure;
for j = 1:n2
    subplot(n, n, j)
    plot(1:44, D(j,:), 'LineWidth', 3);
    axis([1 44 -0.2 1.2]);
end
%}
%{
s = size(D);
n = 5;
n2 = n*n;
for i=1:n2:s(1)
    close all;
    figure;
    for j = 1:n2
        subplot(n, n, j)
        plot(1:44, D(i+j,:), 'LineWidth', 3);
        axis([1 44 -0.2 1.2]);
    end
    k = waitforbuttonpress;
end
close all;
%}
