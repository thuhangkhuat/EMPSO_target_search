%
% Calculate path cost
% Return: 
% costP - Cumulative probability of detection
% costT - Mean time to Detection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [costP,costT]=MyCost(position,model)
    
    if ~CheckMotion(position, model)  % Invalid movement
        costP = 0;                              % Punish path with invalid movement
        costT = model.n + 1;
        return;
    else
        % Input solution
        % Note: Position here is actually a searching path with a number of
        % nodes
        path=PathFromMotion(position,model);
        x=path(:,1);
        y=path(:,2);

        % Input map
        Pmap=model.Pmap;
        N = model.n; % path length
        
        % Target movement
        targetMoves = model.targetMoves; % total moves of target - Zero means static
        targetDir = model.targetDir;
        
        pNodetectionAll = 1; % Initialize the Probability of No detection at all
        pDetection = zeros(N); % Initialize the Probability of detection at each time step
        % Calculate cost
        MTTD = 0;
        for i=1:N
            location.x = x(i) + model.xmax + 1;  % The location is shifted to the range of [1,MAPSIZE]
            location.y = y(i) + model.ymax + 1;
            [scaleFactor,Pmap] = UpdateMap(i,N,targetMoves,targetDir,location,Pmap); % Update the probability map
            pNoDetection = scaleFactor; % Probability of No Detection at time t is Exactly the scaling factor
            pDetection(i) = pNodetectionAll*(1 - pNoDetection); % Probability of Detection for the first time at time i
            pNodetectionAll = pNodetectionAll * pNoDetection;   
            MTTD = MTTD + pNodetectionAll;  % Calculate the mean time to detection (MTTD)
        end
        costP = 1 - pNodetectionAll;  % Return Cumulative Probability of detection up to now (P = 1 - R)
        costT = MTTD;
    end
end