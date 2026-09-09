function overlap = CheckOverlap(path1,path2)
    N = length(path1);
    overlap = false;
    for i=2:N
        for j=2:N
            if isequal(path1(i,:), path2(j,:))
                overlap = true;
            end
        end
    end
end