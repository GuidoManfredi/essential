% Compute mean over the columns (z direction) along a set of lines)
function cum_lines = getCumLines(VM, lines)
    s = size(squeeze(VM(1,1,:)));
    for i=1:length(lines)
        c = zeros(s);
        for j=1:length(lines{i}.line)
            pt = lines{i}.line(j,:);
            %c = c + squeeze(VM(pt(2),pt(1),:));
            c = c | squeeze(VM(pt(2),pt(1),:));
        end
        %c = c / length(lines{i}.line);
        cum_lines{i} = c;
        %figure;
        %plot(c);
    end
    % display
