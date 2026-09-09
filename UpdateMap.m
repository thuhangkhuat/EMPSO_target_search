% Update the belief map given the current position of UAV
%***********************************************
function [scaleFactor, newMap] = UpdateMap(currentStep,pathLength,totalMoves,dir, location, map)
    mapSize = size(map);
    mapSizeX = mapSize(1);
    mapSizeY = mapSize(2);
    
    % Target moves in east direction (totalMoves in the total path)
    move = DirToMove(dir);
    
    if totalMoves ~= 0
        moveStep = pathLength/totalMoves;
        if mod(currentStep,moveStep) == 0   % Shift the belief map every N steps
            tmp_map = noncircshift(map, move);
            map = tmp_map ./sum(tmp_map(:));    % Scale it to 1
        end
    end
    
    pSensorNoDetection = ones(mapSizeX,mapSizeY); % Initialize probability of no detection with ones
    pSensorNoDetection(location.y,location.x) = 0; % For binary sensor model, probability of no detection at UAV location is zero
    newMap = pSensorNoDetection .* map;   % update belief map
    scaleFactor = sum(newMap(:));    % Calculate the scaling factor
    newMap = newMap ./ scaleFactor;  % Scale the updated belief map   
end