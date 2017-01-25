function [ newPath ] = changePath( weakSol, strongSol, AllPoints,CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,s1,s2,s3 )
%CHANGEPATH Summary of this function goes here
%   Detailed explanation goes here
spath1 = size(weakSol,1);
spath2 = size(strongSol,1);
pos1 = rand; 
pos1 = ceil(rand*spath2);

point = AllPoints(pos1,:);

newpath(:,:) = getPath(CollisionTest,StartNode,point,AllPoints,nodes,Boundaryinitial,Boundaryfinal,s1,s2,s3 );
sz = size(newpath,1);
newpath(s1:end,:) = getPath(CollisionTest,point,GoalNode,AllPoints,nodes,Boundaryinitial,Boundaryfinal,s1,s2,s3 );

end

