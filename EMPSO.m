
clc;
clear;
close all;
tic
%% Problem Definition

model = CreateModel(); % Create search map and parameters
PlotModel(model);

CostFunction=@(x) MyCost(x,model);    % Cost Function

nVar = model.n;       % Number of Decision Variables = searching dimension of PSO = number of movements

VarSize=[nVar 2];   % Size of Decision Variables Matrix

VarMin=-model.MRANGE;           % Lower Bound of particles (Variables) (Neighbour weight)
VarMax = model.MRANGE;           % Upper Bound of particles 

N = 3;  % number of UAVs
%% PSO Parameters

MaxIt=100;          % Maximum Number of Iterations

nPop=200;           % Population Size (Swarm Size)


beta_min=0.2;   % Lower Bound of Scaling Factor
beta_max=0.8;   % Upper Bound of Scaling Factor2

pCR=0.8;        % Crossover Probability


w=1;                % Inertia Weight
wdamp=0.98;         % Inertia Weight Damping Ratio
c1=2;             % Personal Learning Coefficient
c2=2;             % Global Learning Coefficient

alpha= 2;
VelMax=alpha*(VarMax-VarMin);    % Maximum Velocity
VelMin=-VelMax;                    % Minimum Velocity

%% Initialization

% Create Empty Particle Structure
empty_particle.Position=[];
empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.BestPosition=[];
empty_particle.BestCost=[];
empty_particle.nBestPosition=[];
empty_particle.nBestCost=[];

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle=repmat(empty_particle,nPop,1);
% Initialization Loop
for i=1:nPop
    
    % Initialize Position
    particle(i).Position=CreateRandomSolution(model);
    
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    % Evaluation
    [costP,costT] = CostFunction(particle(i).Position);
    particle(i).Cost= costT;
    % Update Personal Best
    particle(i).BestPosition=particle(i).Position;
    particle(i).BestCost=particle(i).Cost;
    
    % Update Neighbour Best
    particle(i).nBestPosition=particle(i).Position;
    particle(i).nBestCost=particle(i).Cost;
end

finalBestCost = inf;

%% PSO Main Loop
for it=1:MaxIt

    % Update neighbours based on distance
     for i=1:nPop
        idx = 1;
        % Calculate distances of particle i to all its neighbours
        for j = 1:nPop
            if (j~=i)
                dist(idx).value = DistanceCal(particle(i).BestPosition,particle(j).Position,model); % particle distance
                dist(idx).index = j; % Particle index
                idx = idx + 1;
            end
        end

        % Sort the distances
        distTable = struct2table(dist);
        sortedTable = sortrows(distTable, 'value');
        sortedDist = table2struct(sortedTable);
        % Calculate the best nearest neighbour lbest
        nsize = 5;  % The number of neigbours used
        randNums = rand([1 nsize])*4.1/nsize;
        phi(i) = sum(randNums);
        nBestPosition = zeros(VarSize); 
        for j = 1:nsize
           nidx = sortedDist(j).index;
           nBestPosition = nBestPosition + randNums(j)*particle(nidx).BestPosition;
        end
        particle(i).nBestPosition = nBestPosition/(phi(i)*nsize);
        [costP,costT] = CostFunction(particle(i).nBestPosition);
        particle(i).nBestCost = costT;   
     end
     newParticle = particle;
     for i=1:nPop
        if rand < 0.5
            % Update Position
            newParticle(i).Position = 0.5*particle(i).Position + 0.5*particle(i).nBestPosition;
        else
            A=randperm(nPop); % Generate a random permutation of nPop integers

            A(A==i)=[]; % Remove the current position

            % Choosing random dimension positions
            a=A(1);
            b=A(2);
            c=A(3);

            % Mutation
            beta=unifrnd(beta_min,beta_max,VarSize);    
            newParticle(i).Position=particle(a).Position+beta.*(particle(b).Position-particle(c).Position);
        end

        % Update Position Bounds
        newParticle(i).Position = max(newParticle(i).Position,VarMin);
        newParticle(i).Position = min(newParticle(i).Position,VarMax);

        % Crossover
        for j=1:nVar
            if rand > pCR
                newParticle(i).Position(j)=particle(i).Position(j);
            end
        end

        % Evaluation
        [costP,costT] = CostFunction(newParticle(i).Position);
        newParticle(i).Cost = costT;

        % Replacement
        if newParticle(i).Cost < particle(i).Cost
            particle(i).Position=newParticle(i).Position;
            particle(i).Cost=newParticle(i).Cost;
        end

        % Update Personal Best
        if particle(i).Cost < particle(i).BestCost
            particle(i).BestPosition=particle(i).Position;
            particle(i).BestCost=particle(i).Cost;           
        end
    end

    % Get Paths of mUAVs

    [paths, costs, totalCost] = GetPaths(particle, model,N);

    BestCost(it)      = costs(1);   
    TotalBestCost(it) = totalCost;
    if totalCost < finalBestCost
        finalBestCost = totalCost;
        bestPaths     = paths;       
    end
    FinalBestCost(it) = finalBestCost;

    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    disp(['Iteration ' num2str(it) ': Total Best Cost = ' num2str(TotalBestCost(it))]);
    disp(['Iteration ' num2str(it) ': Final Best Cost = ' num2str(FinalBestCost(it))]);
end
timeElapsed = toc
%% Results
% Plot Solution
% Updade the map with target moves
targetMoves = model.targetMoves; % total moves of target - Zero means static
moveDir = DirToMove(model.targetDir);
moveArr = targetMoves*moveDir;
updatedMap = noncircshift(model.Pmap, moveArr); % Move left
newModel = model;
newModel.Pmap = updatedMap;

% Plot best paths
figure();
hold on
PlotSolution(bestPaths, newModel);

% Plot Total Best cost of n paths over iteration
figure();
plot(FinalBestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Final Best Cost');
grid on;

