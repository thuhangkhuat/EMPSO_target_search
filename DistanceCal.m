function dist = DistanceCal(pos1,pos2,model)
    path1=PathFromMotion(pos1,model);
    path2=PathFromMotion(pos2,model);
    dist = 0;
    for i = 1:length(path1)
        dist = dist + norm(path1(i,:) - path2(i,:));
    end
end