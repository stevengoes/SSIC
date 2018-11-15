%%%%%%%%%%%%% info %%%%%%%%%%%%%%
% Steven Goes                   %
% MSc TU Delft, The Netherlands %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ply_read.m is written by Gabriel Peyré
%  - https://github.com/gpeyre/matlab-toolboxes/blob/master/toolbox_graph/read_ply.m
% plywrite.m written by Ahmet Cecen and MINED@Gatech
%  - https://www.mathworks.com/matlabcentral/fileexchange/55171-plywrite
% stlwrite.m written by Sven Holcombe
%  - https://www.mathworks.com/matlabcentral/fileexchange/20922-stlwrite-write-ascii-or-binary-stl-files

%% Clear, add subfolders and define globals
clear; close all; clear global; clc;
delete *.png

% add subfolders to the path
cd('C:\Users\...'); 
p = genpath('C:\Users\...'); addpath(p);
clear p
clc

global PC meanShape Ncam triangleIDs silhouettes P
%% Load outputdata from Blender
local_dir = 'C:\Users\...\BlenderOutputParams\';

RTdata = importdata([local_dir 'RTmat.txt']);
Kdata = importdata([local_dir 'Kmat.txt']);
CamMatrix = importdata([local_dir 'Pmat.txt']);

f = Kdata(1); %[pix]
u0 = Kdata(3); %[pix]
v0 = Kdata(6); %[pix]
% K = reshape(Kdata(1:9), 3, 3)';

Ncam = size(CamMatrix,1); %number of cameras used in Blender
P = ones(3,4,Ncam);
for i=1:Ncam
    tmp = CamMatrix(i,:);
    P(:,:,i) = reshape(tmp(1:12), 4, 3)';
end

% calculates the Viewing Width of the camera at distance 'camDis'
camDis = P(:,:,1);
camDis = camDis(12); %[m]
VW = 2*camDis*tan(atan(u0/f));

% silhouettes from Blender (patient)
threshold = 10; %threshold value to seperate object from background
[silhouettes,~] = make_silhouettes(threshold, Ncam, local_dir, Kdata);
fprintf('all silhouettes are loaded\n')

% RT = ones(3,4,Ncam);
% Rmatrix = ones(3,3,Ncam);
% for i=1:Ncam
%     tmp = RTdata(i,:);
%     RT(:,:,i) = reshape(tmp(1:12), 4, 3)';
%     Rmatrix(:,:,i) = RT(:,1:3,i);
% end 

%% read the ply files of the database of shapes
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
triangleIDs = fv(1).faces;
fprintf('Models are loaded in %1.3f seconds\n',et);

% Generalized Procrustes Analysis
for i=1:N_trainingmodels %all models or only the training models
    [~,fv(i).vertices]=procrustes(fv(1).vertices, fv(i).vertices, 'Scaling', false);
end
%% Matrix of Shapes & meanShape
% The shape is described by n points, each having x,y,z coordinates.
% The dimension of space is therefore 3n*m dimensions, where n in the 
% number of vertices and m is the number of shapes.

n = size(fv(1).vertices,1); % number of vertices per shape
M = NaN(3*n,N_trainingmodels);
for i = 1:N_trainingmodels
    tmp = fv(i).vertices';
    tmp = tmp(:);
    M(:,i) = tmp;         
end
clear tmp
M; % matrix of shapes M. 
meanShape = mean(M,2);

%% PCA by SVD
% function 'pca' has the svd as default method. PC is the Principal
% Component vector matrix and Eval are the corresponding eigenvalues. The 
% signs of the values in the matrix are not influential. 
[PC,~,Eval]=pca(M');

% scree plot 
figure;
plot(Eval,'-bo')
title('Scree Plot')
xlabel('Principal Component number [-]')
ylabel('Eigenvalue [-]')

% %calculating the number of PC's to cover 95% of the variance
% sums=Eval(1);
% boundary=0;
% for i = 1:length(Eval)
%     boundary = sums/sum(Eval);
%     sums = sums + Eval(i+1);
%     if boundary>.95
%         break
%     end
% end
% fprintf('The first %i PCs account for %1.3f percent of the variation \n',i, (boundary*100));

%% fmincon (Interior-point method)
% More information of the algorithm: >> optimoptions('fmincon')

tr_0 = zeros(6,1);              % intital values of the translation and rotation
b_0 = zeros(length(Eval),1);    % initial values of "b"

% non-linear equality constraints
lb_b = -3*sqrt(Eval);           % lower bound of all the eigenvalues
ub_b = 3*sqrt(Eval);            % upper bound of all the eigenvalues
lb_t = ones(3,1)*(-FOVx/2);     % lower bound of the translation in world coordinates
lb_r = zeros(3,1);              % lower bound of the rotation in radians
ub_t = ones(3,1)*(FOVx/2);      % upper bound of the translation in world coordinates
ub_r = ones(3,1)*2*pi;          % upper bound of the rotation in radians

% parameters for the fmincon function
iniP = [tr_0;b_0];
UB = [ub_t;ub_r;ub_b];
LB = [lb_t;lb_r;lb_b];
Stepsize = 20;
StepVec = 2*UB/Stepsize;

% Termination values that can be changed:
% - options.OptimalityTolerance
% - options.FunctionTolerance
% - options.StepTolerance
% - options.ObjectiveLimit
% - options.MaxIterations
% - options.MaxFunctionEvaluations

%----fmincon--------------------------------------------------------------%
t0=clock;
options = optimoptions('fmincon','Display','iter','CheckGradients',true,'FiniteDifferenceStepSize',StepVec,'StepTolerance',1e-6);
[P_opt,error,exitflag,output] = fmincon(@silhouetteERROR,iniP,[],[],[],[],LB,UB,[],options);
et = etime(clock,t0);
fprintf('Elapsed time is %1.3f minutes\n',et/60);
%-------------------------------------------------------------------------%

X = meanShape + PC*P_opt(7:length(P_opt));
Xssm = transpose(reshape(X(:),3,[])); %3295x3
realArm =  fv(1).vertices; %3295x3

% figure to show SSM
figure;
scatter3(Xssm(:,1),Xssm(:,2),Xssm(:,3),'.')
hold on 
scatter3(realArm(:,1),realArm(:,2),realArm(:,3),'r.')
title("Statistical Shape Model")
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
axis equal
legend("Modified SSM","Real Model")

%% make PLY or STL of the Xssm
save('Xssm.mat','Xssm')
filename = 'Xssm.ply';
faces = triangleIDs;
verts = Xssm;
plywrite(filename,faces,verts);

% %STL
% filename = 'Xssm82.stl';
% Xssm_struct.faces = triangleIDs;
% Xssm_struct.vertices = Xssm;
% stlwrite(filename,Xssm_struct);

