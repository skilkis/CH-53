close all
clc

% Trim Condition Input
initial_velocity = 5.14444; % [m/s]

% Maneuver Parameters

%% Running statespace_matlabwrap.py if Necessary

trigger_python = 0;
if exist('ss.mat', 'file') == 2
    import = load('ss.mat');
    if import.velocity ~= initial_velocity
       trigger_python = 1;
    end
else
    trigger_python = 1;
end

    
if trigger_python
    [status,result] = system(['python statespace_fetcher.py '...
    sprintf('%1.5f', initial_velocity)]);
    import = load('ss.mat');
end

%% Transferring Data to Workspace

A=import.A;
B=import.B;
C=double(import.C);
D=import.D;

u_initial = import.u;
w_initial = import.w;
q_initial = 0; % Due to Linearized State-Space being obtained at trim
thetaf_initial = import.thetaf;
theta0_initial = import.theta0;
thetac_initial = import.thetac;

%[K,S,E] = lqr(A,B(:,1),eye(4),1); %one input of collective
%[K2,S2,E3] = lqr(A,B(:,2),eye(4),1)




%create controller matricies
Ac = A(1:3,1:3);
Bc = B(1:3,:);
Cc = C(1:3,1:3);
Dc = D(1:3,:);
%sys = ss(Ac,Bc,Cc,Dc);

[K,S,E] = lqr(A,B,0.001*eye(4),eye(2));

%[K,S,E] = lqr(Ac,Bc,eye(3),eye(2));
%Nbar = rscale(Ac,Bc,Cc,Dc,K);


%[5.14399965391, -0.00188695927223, 0, -0.000366827238443]
%% Running Simulink Model

open_system('simulink/controller_sim_compat_control_saturated')

%dist = strsplit(version,' '); dist = dist{1, 2};
%if ~isempty(strfind(dist,'2018a'))
%    open_system('simulink/controller_sim') %to open your model
%    sim('controller_sim')
%else
%    open_system('simulink/controller_sim_compat') %to open your model
%    sim('controller_sim_compat')
%end
%else
%    open_system('simulink/controller_sim_compat_control') %to open your model
%    %sim('controller_sim_compat_control')
%end




% states = {'u' 'w' 'q' 'thetaf'};
% inputs = {'collective' 'cyclic'};
% outputs = {'u_dot' 'w_dot' 'q_dot' 'theta_f_dot'};


% sys_mimo = ss(A,B,C,D,'statename',states,...
% 'inputname',inputs,...
% 'outputname',outputs);
% untitled is the name of your simulink model, 
%State-Space is the name of your State space block
% tf(sys_mimo)

