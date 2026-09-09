
% Create the searching map with initial belief
%

function model=CreateModel4()
    
    % Create grid map
    MAP_SIZE = 40;
    x = 1:1:MAP_SIZE; y = 1:1:MAP_SIZE; % x & y from 0 to 20
    [X,Y] = meshgrid(x,y); % replicate x & y to create a rectangular grid (X,Y) 

    % Generate Probability map
    mu1 = [MAP_SIZE/2 MAP_SIZE/2];  % Mean of first distribution (Potential target position)
    Sigma1 = MAP_SIZE*[0.05 0;0 0.05]; % Covarian of the distribution
    F1 = mvnpdf([X(:) Y(:)],mu1,Sigma1);
    F1 = reshape(F1,length(y),length(x));  % Convert F to matrix
    F1 = F1/sum(F1(:)); % Scale to total 1
  %{  
    mu2 = [MAP_SIZE/2 3*MAP_SIZE/4];
    Sigma2 = MAP_SIZE*[0.1 0;0 0.1];
    F2 = mvnpdf([X(:) Y(:)],mu2,Sigma2);
    F2 = reshape(F2,length(y),length(x));  % Convert F to matrix
    F2 = F2/sum(F2(:));
%}
    Pmap = F1; % Standardise the map with two target info sources

   % pcolor(X,Y,Pmap);
    
    %Plot probabilistic map
    figure();
    surf(x,y,Pmap);
    caxis([min(Pmap(:))-.5*range(Pmap(:)),max(Pmap(:))]); % Set colour range
    axis([0 MAP_SIZE 0 MAP_SIZE 0 max(Pmap(:))]);
    xlabel('x'); ylabel('y'); zlabel('Probability Density');
    
   
    
    % Map limits
    xmin= -floor(MAP_SIZE/2);
    xmax= floor(MAP_SIZE/2);
    
    ymin= -floor(MAP_SIZE/2);
    ymax= floor(MAP_SIZE/2);
    
    % Initial searching position
    xs=0;
    ys=-10;
    
    % Number of path nodes (not including the start position (start node))
    n=20;
    
    % Motion range
    MRANGE = 4;
    
    % Incorporate map and searching parameters to a model
    model.xs=xs;
    model.ys=ys;
    model.Pmap=Pmap;
    model.n=n;
    model.xmin=xmin;
    model.xmax=xmax;
    model.ymin=ymin;
    model.ymax=ymax;
    model.MRANGE = MRANGE;
    model.MAPSIZE = MAP_SIZE;
    model.X = X;
    model.Y = Y;
    model.targetMoves = 10; % Must be divisible by the path Length (e.g, mod(N,move)=0)
    model.targetDir = 'SW';
end