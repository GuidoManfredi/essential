function cloud = voxelmap2cloud(voxelmap)
    cloud = zeros(1, 3);
    for x=1:size(voxelmap,1)
        for y=1:size(voxelmap,2)
            for z=1:size(voxelmap,3)
                if voxelmap(x, y, z) == 1
                    a = (x-1) * 0.05 + 0.025;
                    b = (y-1) * 0.05 + 0.025;
                    c = (z-1) * 0.05 + 0.025;
                    cloud(end+1,:) = [a b c];
                end
            end
        end
    end
