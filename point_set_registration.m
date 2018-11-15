%% Load all the ply files to register
% ply_read.m is written by Gabriel Peyré
%  - https://github.com/gpeyre/matlab-toolboxes/blob/master/toolbox_graph/read_ply.m

% Point set registration is already performed.
all_models = dir('...\*.ply');
N_allmodels = length(all_models);
% chose the number of training models, the rest will become the test set
N_trainingmodels=35;

% Create a structure fv (faces & vertices). Rows are the number of models. 
% Each row has two colums consisting the vertices and faces.
t0=clock;
fv = struct('vertices', cell(1,N_allmodels), 'faces', cell(1,N_allmodels));
for i = 1:N_allmodels
    [fv(i).vertices, fv(i).faces] = read_ply(all_models(i).name); %make struct
end
et = etime(clock,t0);

% We are now using the ply model 'N' as source for the nonrigid ICP. The 
% whole trainingset will have the same number of vertices and faces as fv(N).  
N=1; % reference model (i.e. template mesh)
source = fv(N);
%% Loop for the non-rigid ICP for the trainingdata
% nonrigidICP.m is written by Manu
%  - https://www.mathworks.com/matlabcentral/fileexchange/41396-nonrigidicp

% Use a reference mesh and morph it to match the other meshes, thus keeping
% a consistent number of points.
% Model number 'N' is the reference mesh.
% registered = a vertices struct where all the models have the same
% vertices. A 3nxM struct.
% flag: 0 or 1.  
    %  0 if the data still need to be roughly alligned
    %  1 if the data is already alligned (manual or landmark based)

iterations = 10; % between 10 and 30
flag = 0;

t0=clock;
registered = struct('vertices', cell(1,N_trainingmodels));
h = waitbar(0,'Please wait...');
for i = 1:N_trainingmodels
    if i == N
        i = i + 1;
        target = fv(i);
        [registered(i).vertices] = nonrigidICP(target.vertices, source.vertices, ...
            target.faces, source.faces, iterations, flag);
    else
        target = fv(i);
        [registered(i).vertices] = nonrigidICP(target.vertices, source.vertices, ...
            target.faces, source.faces, iterations, flag);
    end
    waitbar(i / N_trainingmodels)
end
%import the source model to the trainingdata
registered(N).vertices = source.vertices;


delete(h)
clear h;

clc
et = etime(clock,t0);
fprintf('Elapsed time is %1.3f minutes\n',et/60); %~20 min
% save('registered.mat','registered')
