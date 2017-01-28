function [ newpath ] = changePath( weakSol, strongSol, AllPoints,CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,s1,s2,s3 )
%CHANGEPATH Summary of this function goes here
%   Detailed explanation goes here
    if ~isempty(strongSol)
        spath1 = size(weakSol,1);
        spath2 = size(strongSol,1);
        pos1 = rand; 
        pos1 = ceil(rand*spath2);
        point = weakSol(pos1,1);

        [newpath(:,:),~,~] = getPath(CollisionTest,StartNode,point,AllPoints(:,:),ceil(nodes/2),Boundaryinitial,Boundaryfinal,s1,s2,s3 );
        sz = size(newpath,1);
        [newpath(sz:end,:),~,~] = getPath(CollisionTest,point,GoalNode,AllPoints(:,:),ceil(nodes/2),Boundaryinitial,Boundaryfinal,s1,s2,s3 );
    else
        spath1 = size(weakSol,1);
        pos = rand; 
        pos = ceil(pos*spath1);
        point = weakSol(pos,1);
        newpath(1:pos,:) = weakSol(1:pos,:);
       	[newpath(pos:end,:),~,~] = getPath(CollisionTest,point,GoalNode,AllPoints,ceil(nodes/2),Boundaryinitial,Boundaryfinal,s1,s2,s3 );
    end
end

