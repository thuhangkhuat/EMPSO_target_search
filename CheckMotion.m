% Check if the encoded motion creates a valid path
%
function valid = CheckMotion(position, model)
    
    n=model.n;
    xs = model.xs;
    ys = model.ys;
    path = zeros(n,2);  % Include n nodes, each node is (x,y)
    currentNode = [xs ys];
    valid = true;
    
    for i=1:n
        motion = position(i,:);
        nextMove = MotionDecode(motion);  
        nextNode = currentNode + nextMove;
        % Out of map boundary
        if nextNode(1) > model.xmax || nextNode(1) < model.xmin...
            || nextNode(2) > model.ymax || nextNode(2) < model.ymin
            valid = false;
            return
        end            
        path(i,:) = nextNode;
        currentNode = nextNode;
    end
   
    % Check duplicate rows
   [u,I,J] = unique(path, 'rows', 'first');
    if size(u,1) < size(path,1)
        valid = false;
        return
    end

end