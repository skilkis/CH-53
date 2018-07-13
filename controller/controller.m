close all
clc

% Trim Condition Input
initial_velocity = 5.14444; % [m/s]

% Maneuver Parameters
maneuver = 'hover'; % Options: ['hover', 'hold']

% Controllers (1 = On, 0 = Off)
surge = 1;
althold = 1;
compensator = 1;

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

% Imported Trim Conditions
u_initial = import.u;
w_initial = import.w;
q_initial = 0; % Due to Linearized State-Space being obtained at trim
thetaf_initial = import.thetaf;
theta0_initial = import.theta0;
thetac_initial = import.thetac;

% Creating Controller Matricies
Ac = A(1:3,1:3);
Bc = B(1:3,:);
Cc = C(1:3,1:3);
Dc = D(1:3,:);

% Performing Linear Quadratic Regulator Optimization
[K,S,E] = lqr(A,B,0.001*eye(4),eye(2));

% Setting Desired Values for the Controller
switch maneuver
    case 'hover'
        u_wish = 0; w_wish = 0; q_wish = 0; thetaf_wish = 0; h_wish=0;
    case 'hold'
        u_wish = u_initial; w_wish = w_initial; q_wish = q_initial;...
            thetaf_wish = thetaf_initial; h_wish=0;
end

%% Running Simulink Model

open_system('simulink/controller_sim_compat_control_saturated');
handle=get_param('controller_sim_compat_control_saturated','handle');
print(handle,'-dpdf','schematic');

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

