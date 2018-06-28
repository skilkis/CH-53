close all

% Trim Condition Input
initial_velocity = 5.3; % [m/s]

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
    [status,result] = system(['python statespace_matlabwrap.py '...
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


%% Running Simulink Model
open_system('controller_sim') %to open your model
sim('controller_sim')

% states = {'u' 'w' 'q' 'thetaf'};
% inputs = {'collective' 'cyclic'};
% outputs = {'u_dot' 'w_dot' 'q_dot' 'theta_f_dot'};


% sys_mimo = ss(A,B,C,D,'statename',states,...
% 'inputname',inputs,...
% 'outputname',outputs);
% untitled is the name of your simulink model, 
%State-Space is the name of your State space block
% tf(sys_mimo)

