function [] = segments2dot(segs, num)
    fileId = fopen('topology.dot','a');
    for i=1:length(segs)
        %length(segs{i}.seg)
        if length(segs{i}.seg) > 8
            if segs{i}.label == 1
                fprintf(fileId, 'edge [color=red]\n');
                fprintf(fileId, '%d -> %d [style=bold,label="door"]\n', num, segs{i}.to);
            elseif segs{i}.label == 2
                fprintf(fileId, 'edge [color=green]\n');
                fprintf(fileId, '%d -> %d [style=bold,label="window"]\n', num, segs{i}.to);
            else
                fprintf(fileId, 'edge [color=magenta]');
                fprintf(fileId, '%d -> %d [style=bold,label="not sure"]\n', num, segs{i}.to);
            end
        end
    end
