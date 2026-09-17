% Joint mean time to detection (MTTD) of a SET of paths on one shared belief.
% Implements the N-sensor product of Eqs. (2)/(5): at each step every UAV in
% the set observes the shared belief simultaneously, then the belief is
% predicted (target motion) and renormalised once. Reduces exactly to
% MyCost.costT when the set contains a single path.
%
% Return:
%   costT - joint MTTD = sum_t R_t   (fleet search time, in time steps)
%   costP - joint cumulative probability of detection = 1 - R_T
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [costT, costP] = JointCost(paths, model)

Pmap        = model.Pmap;
n           = model.n;
targetMoves = model.targetMoves;
move        = DirToMove(model.targetDir);

if isempty(paths)                 % empty fleet never detects: R_t = 1 for all t
    costT = n; costP = 0; return;
end

pNodetectionAll = 1;              % R_t : probability of no detection up to step t
MTTD = 0;
for i = 1:n
    % --- Target motion: shift the SHARED belief once per step ---
    if targetMoves ~= 0
        moveStep = n / targetMoves;
        if mod(i, moveStep) == 0
            tmp  = noncircshift(Pmap, move);
            Pmap = tmp ./ sum(tmp(:));      % scale to 1
        end
    end

    % --- All UAVs observe: zero every occupied cell (product of Eq.(2)) ---
    pSensorNoDetection = ones(size(Pmap));
    for h = 1:numel(paths)
        p  = paths{h};
        xi = p(i,1) + model.xmax + 1;       % shift to the range [1, MAPSIZE]
        yi = p(i,2) + model.ymax + 1;
        pSensorNoDetection(yi, xi) = 0;      % binary perfect sensor at UAV cell
    end

    newMap          = pSensorNoDetection .* Pmap;   % update shared belief
    scaleFactor     = sum(newMap(:));               % joint P(no detection at step i)
    pNodetectionAll = pNodetectionAll * scaleFactor;
    MTTD            = MTTD + pNodetectionAll;        % accumulate joint MTTD
    Pmap            = newMap ./ scaleFactor;         % renormalise shared belief
end

costT = MTTD;
costP = 1 - pNodetectionAll;
end