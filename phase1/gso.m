function path = gso(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end
path = [];
num_expanded = 0;


% clear all
% filename = 'C:\Users\PC\Desktop\MS\Spring 2015\Advanced Robotics\Project-1\Phase-3\studentcode\maps\map1.txt';
% xy_res = 0.1;
% z_res = 2;
% margin = 0.25;
% astar = true;
% map = load_map(filename,xy_res,z_res,margin);
% start = [0.0  -4.9 0.2];
% goal = [6.0  18 3.0];

xy_res = map(3,4);
z_res = map(4,4);
startcopy = start;
goalcopy = goal;
startcollisioncheck = collide(map,start);
goalcollisioncheck = collide(map,goal);
if startcollisioncheck == 1 || goalcollisioncheck == 1
    fprintf('Start or Goal within obstacle\n');
    path = [];
    return;
end

Boundaryinitial = map(1,4:6);
Boundaryfinal = map(2,4:6);
if (Boundaryfinal(3)-Boundaryinitial(3))<z_res
    z_res = Boundaryfinal(3)-Boundaryinitial(3);
end
AllPoints = mygrid(Boundaryinitial,Boundaryfinal,xy_res,z_res);
DummyX = (Boundaryinitial(:,1):xy_res:Boundaryfinal(:,1))';
DummyY = (Boundaryinitial(:,2):xy_res:Boundaryfinal(:,2))';
DummyZ = (Boundaryinitial(:,3):z_res:Boundaryfinal(:,3))';
CollisionTest = collide(map,AllPoints);

if mod(goal(:,1),xy_res)
    goal(:,1) = goal(:,1) - mod(goal(:,1),xy_res);
end
if mod(goal(:,2),xy_res)
%     goal(:,2) = goal(:,2) + (xy_res-mod((goal(:,2)-start(:,2)),xy_res));
    goal(:,2) = goal(:,2) - mod(goal(:,2),xy_res);
end
if mod(goal(:,3),z_res)
    goal(:,3) = goal(:,3) - mod(goal(:,3),z_res);
%     goal(:,3) = goal(:,3) - mod((goal(:,3)-start(:,3)),z_res);
end
if mod(start(:,1),xy_res)
    start(:,1) = start(:,1) - mod(start(:,1),xy_res);
end
if mod(start(:,2),xy_res)
    start(:,2) = start(:,2) - mod(start(:,2),xy_res);
end
if mod(start(:,3),z_res)
    start(:,3) = start(:,3) - mod(start(:,3),z_res);
%     start(:,3) = start(:,3) - mod((start(:,3)-start(:,3)),z_res);
end
% DistanceFromStart = sqrt(sum(bsxfun(@minus, AllPoints, start).^2, 2));
MaxNoofNodes = size(AllPoints,1);
GenerateNodes = (1:MaxNoofNodes)';
% StartNode = GenerateNodes(ismember(AllPoints,start,'rows'));
StartNode = GenerateNodes(sum(abs(AllPoints(:,:)-ones(size(AllPoints,1),1)*start),2)<eps);
% GoalNode = GenerateNodes(ismember(AllPoints,goal,'rows'));
GoalNode = GenerateNodes(sum(abs(AllPoints(:,:)-ones(size(AllPoints,1),1)*goal),2)<eps);
NodesEvaluated = zeros(MaxNoofNodes,1);
NodesTobeEvaluated = zeros(MaxNoofNodes,1);
TotalScore(1:MaxNoofNodes,1) = (Inf);
GoalScore(1:MaxNoofNodes,1) = (Inf);
GoalScore(StartNode) = 0;
NodesTobeEvaluated(StartNode) = 1;
PreviousNodes = zeros(MaxNoofNodes,1);
NodeCounter = 1;
TotalScore(StartNode,1) = GoalScore(StartNode,1) + Heuristic(StartNode,GoalNode,AllPoints,astar);


%First Task it to generate population of solutions

nPop = 20;
MaxIt = 100;
nVar = 5;

%RANGE
range_init = 5.0;
range_boundary = 50.2;

%LUCIFERIN
luciferin_init = 25;
luciferin_decay = 0.4;
luciferin_enhancement = 0.6;

%Neighbors
k_neigh = 20;
beta = 0.5;

step_size = 5;

% Create Empty Glowworm Structure
empty_glowworm.Position=[];
empty_glowworm.range=[];
empty_glowworm.luciferin=[];

% Initialize Global Best
GlobalBest.Cost=inf;
GlobalBest.Position = [];

% Create glowworms Matrix
glowworm=repmat(empty_glowworm,nPop,1);
nodes = 100;
i = 1;
while i<=nPop
    [paths,CurrentNode,NVNodes] = getPath(CollisionTest,StartNode,GoalNode,AllPoints(:,:),nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1));
    %fprintf('%d %d\n',curNodeIndex, nodes);
    paths = removeLoops(paths);
       if CurrentNode == GoalNode 
           glowworm(i).Position = AllPoints(paths,:);
       elseif CurrentNode ~= GoalNode && NVNodes == (nodes + 1)
               glowworm(i).Position = AllPoints(paths,:);
       elseif NVNodes < nodes
           continue;
       else 
           glowworm(i).Position = AllPoints(paths,:);
       end
       glowworm(i).Position(:,4) = paths(:,1);
       glowworm(i).range = range_init;
       glowworm(i).luciferin = luciferin_init;
       glowworm(i).Cost = size(paths,1) + 1000*pdist2(AllPoints(CurrentNode,:),AllPoints(GoalNode,:));
       %fprintf('%d %d %d %d %d %d %d \n',AllPoints(CurrentNode,:),AllPoints(GoalNode,:),pdist2(AllPoints(CurrentNode,:),AllPoints(GoalNode,:)));
       if glowworm(i).Cost < GlobalBest.Cost
           GlobalBest.Cost = glowworm(i).Cost; 
           GlobalBest.Position = glowworm(i).Position;
       end
       
       i = i + 1;
end
% Algorithm initialization

for it=1:MaxIt
    
    for i=1:nPop
        % Update luciferin
        %disp(glowworm(i).luciferin.x);
        glowworm(i).luciferin = (1-luciferin_decay).*glowworm(i).luciferin + luciferin_enhancement.*(glowworm(i).Cost/10);
        neighbors = [];
        for k =1:nPop
            dist = pdist2(glowworm(i).Position(end,:),glowworm(k).Position(end,:));
			%if it is in it's range of sigth and it's brightness is higher
            if (dist(1) ~= 0) && (dist(1) <= glowworm(i).range(1)) && (glowworm(i).luciferin(1) <= glowworm(k).luciferin(1))
                neighbors = [neighbors ; k];
            end
        end
        
        if size(neighbors,1) > 0
         % find the node in the direction of which the glowworm should
         % follow
            li = glowworm(i).luciferin;
            sum_lk = sum(glowworm(i).luciferin);
            neighbors_index = size(neighbors,1);
            %calc probabilties for each neighbor been followed
            probs = zeros(1,neighbors_index);
            for j = 1:neighbors_index
                probs(j) = abs(sum(glowworm(j).luciferin) - sum(li));
            end
            probs = probs./abs( sum_lk - sum(size(probs,2)*li));
            
            %calc prob range
            acc = 0;
            wheel = [];
            for val = 1:size(probs,2)
                acc = acc + probs(val);
                wheel = [wheel ; acc];
            end

            %wheel(-1) = 1 ;

            %randomly choice a value for wheel selection method
            rand_val = rand;
            following = i;
            for k = 1:size(wheel,1)
                if rand_val <= wheel(k)
                    following = k;
                    break;
                end
            end

            toward_index = following;
            
            %Position update 
            glowworms = glowworm(i).Position;
            toward = glowworm(toward_index).Position;
            
            
            newPath(:,:) = changePath(glowworms(:,:),toward(:,:),AllPoints(:,:),CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1));
            %glowworm(i).Position(:,:) = newPath(:,:);       
            %new_position = glowworms + step_size.*(toward-glowworms)./normV;
           % glowworm(i).Position.x = new_position;
        end
        if size(neighbors)== 0
          
            
            
        end
        glowworm(i).range = min(range_boundary,max(0.1,glowworm(i).range + (beta*(k_neigh-size(neighbors,1)))));
        
        
    end 
end
path = GlobalBest.Position(:,1:3);

%fprintf('%d %d %d',size(path));
plot_path(map,path);
% plot3(path(:,1), path(:,2), path(:,3),'b');
% hold on;
% grid on;    
end

