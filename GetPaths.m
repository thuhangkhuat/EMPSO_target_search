% Marginal-gain extraction of the fleet.
% Return:
%   paths     - selected fleet (cell array of decoded paths)
%   costs     - running joint MTTD after each path is added
%   totalCost - joint MTTD of the final fleet (the reported search time)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [paths, costs, totalCost] = GetPaths(particle, model, N)

    nPop = numel(particle);

    % Decode every personal best into a candidate path once.
    cand = cell(nPop,1);
    for k = 1:nPop
        cand{k} = PathFromMotion(particle(k).BestPosition, model);
    end

    paths   = {};
    costs   = [];
    chosen  = false(nPop,1);
    curCost = JointCost({}, model);        % empty-fleet baseline = model.n

    for slot = 1:N
        bestGain = 0;                       % require strictly positive marginal gain
        bestK    = -1;
        bestCost = curCost;
        for k = 1:nPop
            if chosen(k), continue; end
            trialCost = JointCost([paths, cand(k)], model);
            gain      = curCost - trialCost;
            if gain > bestGain
                bestGain = gain;
                bestK    = k;
                bestCost = trialCost;
            end
        end
        if bestK < 0                        % no remaining candidate improves the fleet
            break;
        end
        paths{end+1} = cand{bestK};
        costs(end+1) = bestCost;
        chosen(bestK) = true;
        curCost = bestCost;
    end

    totalCost = curCost;                    % joint MTTD of the selected fleet
end