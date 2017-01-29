close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map1.txt', 0.1, 0.5, 0.25);
start = {[1 -4 6]};
stop  = {[9 17 2]};
nquad = length(start);
for qn = 1:nquad
    v = cputime;
    path{qn} = gso(map, start{qn}, stop{qn}, true);
    c = cputime - v;
    fprintf('Algo Execution time = %d \n',c);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
