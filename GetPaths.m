function [paths, costs, totalCost] = GetPaths(particle, model, N)
    particleTable  = struct2table(particle);
    sortedTable    = sortrows(particleTable, 'BestCost', 'ascend');
    sortedParticle = table2struct(sortedTable);
    nPop = numel(sortedParticle);

    MaxNum = model.n + 1; 

    paths = {};
    costs = [];
    for k = 1:nPop
        if numel(paths) >= N
            break;
        end
        candPath  = PathFromMotion(sortedParticle(k).BestPosition, model);
        isOverlap = false;
        for m = 1:numel(paths)
            if CheckOverlap(candPath, paths{m})
                isOverlap = true;
                break;
            end
        end
        if ~isOverlap
            paths{end+1} = candPath;
            costs(end+1) = sortedParticle(k).BestCost;
        end
    end

    nFound     = numel(paths);
    nMissing   = N - nFound;
    totalCost  = sum(costs) + nMissing * MaxNum;
end