clc
close all

% Author: San Kilkis

% Trim Condition Input
initial_velocity = 5.14444; % [m/s]

% Maneuver Parameters (NOTE: Compensator is always turned on for hover)
% _c: Compensator
% _s: Surge Hold Controller & Comp.
% _a: Altitude Hold Controller & Comp.
% _as: Altitude Hold & Surge Controller & Comp.
maneuver = 'phughoid_a'; % Options: ['hover', 'hover_s', hover_a', 'hover_as'
                                  %'phughoid', 'phughoid_c', 'phughoid_s'
                                  %'phughoid_a', 'phughoid_as']

% Toggle Dispalying Simulink Controller & Printing Schematic of Model
show_simulink = 0;
print_schematic = 0;

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

%% Transferring Data from Python Stability Derivatives to Workspace

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

%% Computing Ideal Compensator for Stability at Minimum Cost

% Creating Compensator Matricies
Ac = A(1:3,1:3);
Bc = B(1:3,:);
Cc = C(1:3,1:3);
Dc = D(1:3,:);

% Performing Linear Quadratic Regulator Optimization
[K,S,E] = lqr(A,B,0.001*eye(4),eye(2));

%% Setting Toggle Cases for the Controller as well as Disturbances

% Setting Desired Values for the Controller
switch maneuver
    case 'hover'
        u_wish = 0; w_wish = 0; q_wish = 0; thetaf_wish = 0; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 0;
        althold = 0;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 0;
        
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 1;
        
        % Simulation Time
        sim_time = 70;
        
    case 'hover_s'
        u_wish = 0; w_wish = 0; q_wish = 0; thetaf_wish = 0; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 1;
        althold = 0;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 0;
        
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 0;
        
        % Simulation Time
        sim_time = 70;
        
    case 'hover_a'
        u_wish = 0; w_wish = 0; q_wish = 0; thetaf_wish = 0; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 0;
        althold = 1;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 0;
        
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 0;

        % Simulation Time
        sim_time = 70;
        
    case 'hover_as'
        u_wish = 0; w_wish = 0; q_wish = 0; thetaf_wish = 0; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 1;
        althold = 1;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 0;
 
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 0;
        
        % Simulation Time
        sim_time = 70;
        
    case 'phughoid'
        u_wish = u_initial; w_wish = w_initial; q_wish = q_initial;...
            thetaf_wish = thetaf_initial; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 0;
        althold = 0;
        compensator = 0;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 1;
 
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 1;
        
        % Simulation Time
        sim_time = 40;
        
        % Creating LTI System
        sys = ss(A, B, C, D);
        
    case 'phughoid_c'
        u_wish = u_initial; w_wish = w_initial; q_wish = q_initial;...
            thetaf_wish = thetaf_initial; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 0;
        althold = 0;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 1;
 
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 0;
        
        % Simulation Time
        sim_time = 40;
        
    case 'phughoid_s'
        u_wish = u_initial; w_wish = w_initial; q_wish = q_initial;...
            thetaf_wish = thetaf_initial; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 1;
        althold = 0;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 1;
 
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 0;
        
        % Simulation Time
        sim_time = 40;
        
    case 'phughoid_a'
        u_wish = u_initial; w_wish = w_initial; q_wish = q_initial;...
            thetaf_wish = thetaf_initial; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 0;
        althold = 1;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 1;
 
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 0;
        
        % Simulation Time
        sim_time = 40;
        
    case 'phughoid_as'
        u_wish = u_initial; w_wish = w_initial; q_wish = q_initial;...
            thetaf_wish = thetaf_initial; h_wish=0;
        
        % Controllers (1 = On, 0 = Off)
        surge = 1;
        althold = 1;
        compensator = 1;

        % Disturbances (For Phughoid)
        collective_dist = 0;
        cyclic_dist = 1;
 
        % Setting Plot States
        plot_responses = 1;
        plot_poles = 1;
        
        % Simulation Time
        sim_time = 40;
        
        % Loading Control LTI System
        sys = load('ss_control.mat'); sys = sys.lincontrol;

end

%% Running Simulink Model & Printing Schematic

% Checks MATLAB version to determine correct simulink file:
dist = strsplit(version,' '); dist = dist{1, 2};
if ~isempty(strfind(dist,'2018a'))
    folder = 'simulink';
    model = 'ch53_sim';
else
    folder = 'simulink';
    model = 'ch53_sim_compat';
end

% Loads System, Sets Simulation Stop Time, and Runs Simulation
load_system(sprintf('%s/%s', folder, model))
set_param(model,'StopTime',sprintf('%d', sim_time))
sim(model);

% User-Requested Actions for Opening the Simulink File & Printing Schematic
handle=get_param(model,'handle');
if show_simulink; open_system(model); end
if print_schematic; print(handle,'-dpdf','Figures/schematic'); end

%% Plotting

AR=[8 5]; %Aspect Ratio

% Fetching Controller Parameters to Label Title
controller_state = sprintf('COMP = %s, ALT. HOLD = %s, SRG. HOLD = %s',...
    toggle_str(compensator), toggle_str(althold), toggle_str(surge));

if plot_responses

    figure('Name',sprintf('response_%s', maneuver));

        % Surge Velocity
        subplot(4,1,1)
        plot(tout,xout.signals(4).values(:,1) + u_initial)
        grid on;
        title(sprintf('Hover Maneuver Response (%s)', controller_state))
        ylabel('u [m/s]');

        % Heave Velocity
        subplot(4,1,2)
        plot(tout,xout.signals(4).values(:,2) + w_initial)
        grid on;
        ylabel('w [m/s]');
        ax = gca;
        ax.YAxis.Exponent = -2;

        % Pitch Rate
        subplot(4,1,3)
        plot(tout,rad2deg(xout.signals(4).values(:,3) + q_initial))
        grid on;
        ylabel('q [rad/s]');

        % Fuselage Pitch
        subplot(4,1,4)
        plot(tout,rad2deg(xout.signals(4).values(:,4) + thetaf_initial))
        grid on;
        xlabel('Time [s]')
        ylabel('\theta_f [rad]')

        set(gcf, 'InvertHardCopy', 'off',...
        'PaperPosition', [0 0 AR(1) AR(2)],...
        'PaperSize', [AR(1) AR(2)]);

    figure('Name',sprintf('position_%s', maneuver));

        % Position
        subplot(2,1,1)
        plot(tout,xout.signals(6).values)
        grid on;
        ylabel('\Deltax [m]');
        title(sprintf('Hover Maneuver Position (%s)', controller_state))

        % Altitude
        subplot(2,1,2)
        plot(tout,xout.signals(1).values)
        grid on;
        ylabel('\Deltah [m]');

        set(gcf, 'InvertHardCopy', 'off',...
        'PaperPosition', [0 0 AR(1) AR(2)],...
        'PaperSize', [AR(1) AR(2)]);

    figure('Name',sprintf('control_%s', maneuver));

        % Collective Pitch
        subplot(2,1,1)
        plot(tout, theta0_control + rad2deg(theta0_initial))
        grid on;
        ylabel('C. Pitch \theta_{0} [deg]');
        title(sprintf('Hover Control Inputs (%s)', controller_state))

        % Longitudinal Cyclic
        subplot(2,1,2)
        plot(tout, thetac_control + rad2deg(thetac_initial))
        grid on;
        ylabel('L. Cyclic \theta_{ls} [deg]');

        set(gcf, 'InvertHardCopy', 'off',...
        'PaperPosition', [0 0 AR(1) AR(2)],...
        'PaperSize', [AR(1) AR(2)]);
end

if plot_poles
    
    if plot_responses

    figure('Name',sprintf('pzmap_%s', maneuver));
    
        h = pzplot(sys);
        grid on;
        
        p = getoptions(h); 
        p.Title.String = sprintf('%s (%s)', p.Title.String, controller_state);
        p.XLabel.String = 'Re(\lambda)';
        p.YLabel.String = 'Im(\lambda)';
        p.GridColor = [0.5, 0.5, 0.5];
        setoptions(h,p);  
        
        set(gcf, 'InvertHardCopy', 'off',...
        'PaperPosition', [0 0 AR(1) AR(2)],...
        'PaperSize', [AR(1) AR(2)]);
    
    end
    
end

%% Saving/Overwriting Figures in the Images Folder as a .pdf

choice=questdlg('Would you like to close and save all figures to ../Figures?',...
    'Figure Save Dialog', ...
    'Yes','Just Save','Specify Directory','Specify Directory');
switch choice
    case 'Yes'
        if exist('Figures','dir')==0
        mkdir('Figures')
        end
        cd('Figures')
        b1=waitbar(0,'1','Name','Please Wait');
        H=gcf;
        i_prime=H.Number;
        for i=1:i_prime
            waitbar(i/i_prime,b1,sprintf('Saving Figure (%d/%d)',i,i_prime))
            saveas(i, get(i,'Name'), 'pdf')
        end
        cd('../')
        close all
        delete(b1)
        b2=msgbox('Operation Completed','Success');
    case 'Just Save'
        if exist('Figures','dir')==0
        mkdir('Figures')
        end
        cd('Figures')
        b1=waitbar(0,'1','Name','Please Wait');
        H=gcf;
        i_prime=H.Number;
        for i=1:i_prime
            waitbar(i/i_prime,b1,sprintf('Saving Figure (%d/%d)',i,i_prime))
            saveas(i, get(i,'Name'), 'pdf')
        end
        cd('../')
        delete(b1)
        b2=msgbox('Operation Completed','Success');
    case 'Specify Directory'
        old_dir=cd;
        new_dir=uigetdir('','Select Figure Saving Directory');
        cd(new_dir)
        b1=waitbar(0,'1','Name','Please Wait');
        H=gcf;
        i_prime=H.Number;
        for i=1:i_prime
            waitbar(i/i_prime,b1,sprintf('Saving Figure (%d/%d)',i,i_prime))
            saveas(i, get(i,'Name'), 'pdf')
        end
        cd(old_dir)
        close all
        delete(b1)
        b2=msgbox('Operation Completed','Success');
end

%% Methods
function str = toggle_str(value)
% Returns ON or OFF depending on the current value of the toggle switch
    if value == 0
        str = 'OFF';
    else
        str = 'ON';
    end
end


