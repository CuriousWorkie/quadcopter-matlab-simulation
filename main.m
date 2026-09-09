close all;
clc; clear;

projectFolder = fileparts(mfilename('fullpath'));
addpath(fullfile(projectFolder, 'lib'));

%% DEFINE
R2D = 180/pi;
D2R = pi/180;

%% INIT. PARAMS.
drone1_params = containers.Map({'mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
                               {1.25, 0.265, 0.0232, 0.0232, 0.0468});

%watch out for the transpose symbol in every martix. 
drone1_initStates = [0, 0, -6, ...      % X, Y, Z
                     0, 0, 0, ...      % dX, dY, dZ
                     0, 0, 0, ...      % phi, theta, psi
                     0, 0, 0]';        % p, q, r

drone1_initInputs = [0, 0, 0, 0]';     % u1, u2, u3, u4 (T, M1, M2, M3)

drone1_body = [ 0.265,     0,      0,    1; ...
                    0, -0.265,     0,    1; ...
               -0.265,     0,      0,    1; ...
                    0,  0.265,     0,    1; ...
                    0,      0,     0,    1; ...
                    0,      0, -0.15,    1]';     %Co ordinates of every motor and COM of the drone

drone1_gains = containers.Map(...
    {'P_phi', 'I_phi', 'D_phi', ...
     'P_theta', 'I_theta', 'D_theta', ...
     'P_psi', 'I_psi', 'D_psi', ...
     'P_zdot', 'I_zdot', 'D_zdot'}, ...
    {1, 0, 0, ...
     0, 0, 0, ...
     0, 0, 0, ...
     0, 0, 0}); %contains all the K values needed for PID control

simulationTime = 5;

drone1 = Drone(drone1_params, ...
               drone1_initStates, ...
               drone1_initInputs, ...
               drone1_gains, ...
               simulationTime);    %Calls the class Drone and indirectly calls the function obj

%% Init. 3D Fig.

fig1 = figure('pos',[50 200 800 800]);
h = gca;   %stores the graph axes to a variable h
view(3);   %show the 3D view

fig1.CurrentAxes.ZDir = 'Reverse';
fig1.CurrentAxes.YDir = 'Reverse';

axis equal;
grid on;

xlim([-5 5]);
ylim([-5 5]);
zlim([-8 0]);

xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');

hold(gca, 'on');  %makes sure previous plot stays visible while the next one is being plotted

drone1_state = drone1.GetState();

wHb = [RPY2Rot(drone1_state(7:9))'  drone1_state(1:3);
       0 0 0 1];           %transformation matrix used to convert the drone body cordinates to ground cordinates

drone1_world = wHb * drone1_body;
drone1_atti = drone1_world(1:3,:);  %just removing all homogenous terms

fig1_ARM13 = plot3(gca, ...
    drone1_atti(1,[1 3]), ...
    drone1_atti(2,[1 3]), ...
    drone1_atti(3,[1 3]), ...  %general syntax for plot3(gca,(x1,x2),(y1,y2),(z1,z2)) where 1 and 2 are the points to connect
    '-ro', 'MarkerSize', 5);   %each column of 3x6 atti matrix has the three cordinates of each motor, COM and payload

fig1_ARM24 = plot3(gca, ...
    drone1_atti(1,[2 4]), ...
    drone1_atti(2,[2 4]), ...
    drone1_atti(3,[2 4]), ...
    '-bo', 'MarkerSize', 5);

fig1_payload = plot3(gca, ...
    drone1_atti(1,[5 6]), ...
    drone1_atti(2,[5 6]), ...
    drone1_atti(3,[5 6]), ...
    '-k', 'LineWidth', 3);

fig1_shadow = plot3(gca, 0, 0, 0, 'xk', 'LineWidth', 3);

hold(gca, 'off');

%% Init. Data Fig.

%% Position and Velocity Figure

fig2 = figure('Position', [600 200 1000 600]);

subplot(2,3,1)
title('x [m]');
grid on;
hold on;

subplot(2,3,2)
title('y [m]');
grid on;
hold on;

subplot(2,3,3)
title('z [m]');
grid on;
hold on;

subplot(2,3,4)
title('xdot [m/s]');
grid on;
hold on;

subplot(2,3,5)
title('ydot [m/s]');
grid on;
hold on;

subplot(2,3,6)
title('zdot [m/s]');
grid on;
hold on;

%MATLAB has no variable declarations. Just start using
commandSig(1) = 5.0*D2R; %phi
commandSig(2) = 0.0*D2R; %theta
commandSig(3) = 0.0*D2R; %psi
commandSig(4) = 0.0; %zdot

for i = (1:simulationTime/0.01)

    drone1.AttitudeCtrl(commandSig);
    drone1.UpdateState();   %update every 0.01 second

    drone1_state = drone1.GetState();
    %% 3D Plot
    
    if ~ishandle(fig1)
        disp("Simulation stopped by user.");
        break;
    end
    
    
    wHb = [RPY2Rot(drone1_state(7:9))'  drone1_state(1:3);
           0 0 0 1];
    
    drone1_world = wHb * drone1_body;
    drone1_atti  = drone1_world(1:3, :); %removing all the homogenous terms
    
    set(fig1_ARM13, ...
        'XData', drone1_atti(1,[1 3]), ...   %All three rows have X Y Z cordinates of all 4 motors, COM and Payload
        'YData', drone1_atti(2,[1 3]), ...   %row 1 -x row 2-y row 3 -z
        'ZData', drone1_atti(3,[1 3]));      %"Take the line called fig1_ARM13 and move its two endpoints to these new x, y, z coordinates."
    
    set(fig1_ARM24, ...
        'XData', drone1_atti(1,[2 4]), ...
        'YData', drone1_atti(2,[2 4]), ...
        'ZData', drone1_atti(3,[2 4]));
    
    set(fig1_payload, ...
        'XData', drone1_atti(1,[5 6]), ...
        'YData', drone1_atti(2,[5 6]), ...
        'ZData', drone1_atti(3,[5 6]));
    
    set(fig1_shadow, ...
        'XData', drone1_state(1), ...
        'YData', drone1_state(2), ...
        'ZData', 0);
    
    if ~ishandle(fig2)
        disp("Simulation stopped by user.");
        break;
    end

    subplot(2,3,1)
    plot(i/100, drone1_state(1), '.');
    
    subplot(2,3,2)
    plot(i/100, drone1_state(2), '.');
    
    subplot(2,3,3)
    plot(i/100, drone1_state(3), '.');
    
    subplot(2,3,4)
    plot(i/100, drone1_state(4), '.');
    
    subplot(2,3,5)
    plot(i/100, drone1_state(5), '.');

    subplot(2,3,6)
    plot(i/100, drone1_state(6), '.');
   

    if drone1_state(3) >= 0      % because z = 0 is the ground (Z-axis is reversed)
        disp("Drone has crashed!");
    break;
    end
    drawnow ;
end
    
