function img = coords2image(xy)
mx = min(xy(:,1));
Mx = max(xy(:,1));
my = min(xy(:,2));
My = max(xy(:,2));

img = zeros(Mx-mx, My-my);
for i=1:size(xy)
    img(xy(i,1), xy(i,2)) = 1;
end
