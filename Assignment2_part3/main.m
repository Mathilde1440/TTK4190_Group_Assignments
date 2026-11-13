% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           Odin Sandlie Mellingsæter and Mathilde Skaset-Haarr
% Study program:    MTTK

% Add folder for 3-D visualization files
addpath(genpath('flypath3d_v3'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;clear functions;
T_final = 6300;	        % Final simulation time (s)
h = 0.1;                % Sampling time (s)

%Problem 4a - Measurement noise
sigma_psi = deg2rad(0.5);
sigma_r = deg2rad(0.1);

psi_ref =-110 * pi/180;  % desired yaw angle (rad)
U_ref   = 9;            % desired surge speed (m/s)

Vc = 1;                 % current speed (m/s)
beta_Vc = 45*pi/180;    % current direction (rad)

%wind parameters
Vw = 10;
beta_w = 135*pi/180;

rho_a = 1.247;
cy = 0.95;
cn = 0.15;

L_oa = 161;
A_Lw = 10*L_oa;

% initial states
eta_0 = [0 0 deg2rad(-110)]';
nu_0  = [0 0 0]';
delta_0 = 0;
n_0 = 0;
x = [nu_0' eta_0' delta_0 n_0]'; % The state vector can be extended with addional states here

xd =[0 0 0]';
e_int = 0;

last       = 0;
xk         = 0;
yk         = 0;
xk1        = 0;
yk1        = 0;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0:h:T_final;                % Time vector
nTimeSteps = length(t);         % Number of time steps

simdata = zeros(nTimeSteps, 13); % Pre-allocate matrix for efficiency
chi_d_vect = zeros(nTimeSteps,1);

psi_meas_vect = zeros(nTimeSteps,1);
r_meas_vect = zeros(nTimeSteps,1);

% Problem 4b - Kalman filter init

T_nomoto = -99.47;
K_nomoto = -0.004883;

Ad = [1, h, 0;
      0, 1-h/T_nomoto, - h*K_nomoto/T_nomoto;
      0, 0, 1];

Bd = [0;
      h*K_nomoto/T_nomoto;
      0];

Cd = [1, 0, 0];
Ed = [0; 0; h];

x_prd = zeros(3,1);

P_prd = diag([deg2rad(30)^2, deg2rad(1)^2, deg2rad(5)^2]);  %tuning parameter

Rd = deg2rad(1)^2;
Qd = diag([1e-9, 5e-9, 1e-10]);   %tuning parameter

x_est_vect = zeros(nTimeSteps, 3);

for i = 1:nTimeSteps
    % if t(i) >= 500
    %     psi_ref = -20 * pi/180;
    % end

     t_n = t(i);

    %-----No current---------
    % uc = 0;
    % vc = 0;
    % nu_c = [ uc vc 0 ]';

    % -----Current ----------
    psi = x(6);

    uc = Vc*cos(beta_Vc - psi);
    vc = Vc*sin(beta_Vc - psi);
    nu_c = [ uc vc 0 ]';

    % Add measurement noise to heading and yaw-rate measurements
    psi_meas = psi + sigma_psi*randn();
    r_meas = x(3) + sigma_r*randn();

    [x_pst, P_pst, x_prd, P_prd] = KF(x_prd,P_prd,Ad,Bd,Ed,Cd,Qd,Rd,psi_meas,x(7));

    x_est_vect(i,:) = x_pst';

    psi_meas_vect(i) = psi_meas;
    r_meas_vect(i) = r_meas;
    
    % --------Wind------------- 
    if t(i) >= 200
        u_ship = x(1);
        v_ship = x(2);

        uw = Vw*cos(beta_w - psi);
        vw = Vw*sin(beta_w - psi);

        urw = uw - u_ship;
        vrw = vw - v_ship;

        Vrw = sqrt(urw^2 + vrw^2);
        gamma_rw = -atan2(vrw, urw);

        Ywind = 0.5*rho_a*Vrw^2*A_Lw*cy*sin(gamma_rw);
        Nwind = 0.5*rho_a*Vrw^2*A_Lw*L_oa*cn*sin(gamma_rw);

    else
        Ywind = 0;
        Nwind = 0;
    end

    tau_wind = [0; Ywind; Nwind];

    %----- Guidance law ----------- 

    [xk1,yk1,xk,yk,last] = WP_selector(x(4),x(5));
    [pi_h, y_ep]  = crossTrackError(xk1,yk1,xk,yk,x(4),x(5));
    
    % chi_d = LOS_guidance(pi_h, y_ep);
    chi_d = ILOS_guidance(pi_h, y_ep,h);
    beta_c_i = atan2(x(2), x(1));
    psi_ref = chi_d - beta_c_i;

    %crab angle compansation
    % beta_c_i = atan2(x(2), x(1));
    % psi_ref = psi_ref-beta_c_i;


    chi_d_vect(i) = rad2deg(chi_d);

    % -----Reference model--------  

    % psi_d = psi_ref;
    % r_d = 0;
    % u_d = U_ref;
    xd_dot = ref_model(xd, psi_ref);

    psi_d = xd(1);
    r_d = xd(2);

    xd = xd + h*xd_dot;
    u_d = U_ref;
    

    % --------Heading controller ----------- 

    %delta_c = 0.1;              % rudder angle command (rad)
    % e_psi = psi_d - psi;
    % e_r = r_d - x(3);
    
    %Problem 4c
    % e_psi = ssa(psi_meas - psi_d);
    % e_r = r_meas - r_d;

    %Kalman filter 4d
    e_psi = ssa(x_pst(1) - psi_d);
    e_r = x_pst(2) - r_d;
   
    e_u = x(1) - u_d;

    %delta_c = PID_heading(e_psi,e_r,e_int);
    delta_c = PD_heading_KF(e_psi,e_r); %Kalman filter 4b
    % delta_c = PID_heading(e_psi,e_r,e_int);
    delta_c = PD_heading(e_psi,e_r);

    e_int = e_int + h*e_psi;


    %--------open loop speed control----------                     
    % n_c = 10;                   % propeller speed [radians per second (rps)]
    n_c = open_loop_speed_control(U_ref);

    % Closed loop speed controller
    
    % ship dynamics
    u = [delta_c n_c]';
    [xdot,tau_total] = ship(x,u,nu_c,tau_wind);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d psi_d r_d];     
 
    % Euler integration
    % x = euler2(xdot,x,h); 
    % Runge Kutta 4 integration
    x = rk4(@ship,h,x,u,nu_c,tau_wind);

    % --- Progress Update Logic ---
    if mod(i, floor(nTimeSteps / 10)) == 0 % Print an update every 10%
        progress = (i/nTimeSteps) * 100;
        fprintf('  %d%% complete\n', round(progress));
    elseif i == 1
        disp("  Simulation in progress:")
        fprintf('  %d%% complete\n', 0);
    end

end
simdata = simdata(1:i,:);
t = t(1:i);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u           = simdata(:,1);                 % m/s
v           = simdata(:,2);                 % m/s
r           = simdata(:,3);                 % rad/s
r_deg       = (180/pi) * r;                 % deg/s
x           = simdata(:,4);                 % m
y           = simdata(:,5);                 % m
psi         = simdata(:,6);                 % rad
psi_deg     = (180/pi) * psi;               % deg
delta_deg   = (180/pi) * simdata(:,7);      % deg
n           = (30/pi) * simdata(:,8);       % rpm
delta_c_deg = (180/pi) * simdata(:,9);      % deg
n_c         = (30/pi) * simdata(:,10);      % rpm
u_d         = simdata(:,11);                % m/s
psi_d       = simdata(:,12);                % rad
psi_d_deg   = (180/pi) * psi_d;             % deg
r_d         =  simdata(:,13);               % rad/s
r_d_deg     = (180/pi) * r_d;               % deg/s
%%
figure(3)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi_deg,t,psi_d_deg,'linewidth',2);
title('Actual and desired yaw angle'); xlabel('Time (s)');  ylabel('Angle (deg)'); 
legend('actual yaw','desired yaw')
subplot(313)
plot(t,r_deg,t,r_d_deg,'linewidth',2);
title('Actual and desired yaw rates'); xlabel('Time (s)');  ylabel('Angle rate (deg/s)'); 
legend('actual yaw rate','desired yaw rate')

figure(2)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('actual surge','desired surge')
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed'); xlabel('Time (s)'); ylabel('Motor speed (RPM)');
legend('actual RPM','commanded RPM')
subplot(313)
plot(t,delta_deg,t,delta_c_deg,'linewidth',2);
title('Actual and commanded rudder angle'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('actual rudder angle','commanded rudder angle')
%% Create objects for 3-D visualization 
% Since we only simulate 3-DOF we need to construct zero arrays for the 
% excluded dimensions, including height, roll and pitch
% z = zeros(length(x),1);
% phi = zeros(length(psi),1);
% theta = zeros(length(psi),1);
% 
% % create object 1: ship (ship1.mat)
% new_object('flypath3d_v3/ship1.mat',[x,y,z,phi,theta,psi],...
% 'model','royalNavy2.mat','scale',(max(max(abs(x)),max(abs(y)))/1000),...
% 'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
% 'path','on','pathcolor',[.89 .0 .27],'pathwidth',2);
% 
% % Plot trajectories 
% flypath('flypath3d_v3/ship1.mat',...
% 'animate','on','step',500,...
% 'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
% 'font','Georgia','fontsize',12,...
% 'view',[-25 35],'window',[900 900],...
% 'xlim', [min(y)-0.1*max(abs(y)),max(y)+0.1*max(abs(y))],... 
% 'ylim', [min(x)-0.1*max(abs(x)),max(x)+0.1*max(abs(x))], ...
% 'zlim', [-max(max(abs(x)),max(abs(y)))/100,max(max(abs(x)),max(abs(y)))/20]); 


%% Problem 1b

% Current velocity in the body-fixed frame
uc = Vc*cos(beta_Vc - psi);
vc = Vc*sin(beta_Vc - psi);

% Calculate angles
beta = atan2(v - vc, u - uc);  % Sideslip angle
beta_c = atan2(v, u);         % Crab angle

% Plot
figure
plot(t, rad2deg(beta), 'LineWidth', 1.5)
hold on
plot(t, rad2deg(beta_c), '--', 'LineWidth', 1.5)
grid on

xlabel('Time (s)')
ylabel('Angle (deg)')
legend('Sideslip angle', 'Crab angle')
title('Sideslip and Crab Angles')

%% Problem 2e

figure
plot(t, psi_deg, 'LineWidth', 1.5)
hold on
plot(t, psi_d_deg, '--', 'LineWidth', 1.5)

grid on
xlabel('Time (s)')
ylabel('Heading angle (deg)')
legend('Actual heading', 'Desired heading')
title('Heading Control')

%% heading plot
chi = psi + atan2(v-vc, u-uc);

figure
plot(t, rad2deg(chi), 'LineWidth', 1.5)
hold on
plot(t, chi_d_vect, '--', 'LineWidth', 1.5)

grid on
xlabel('Time (s)')
ylabel('Course (deg)')
legend('Actual course', 'Desired course')
title('Course')

% figure 
pathplotter(x,y)


%% Problem 4a - Noisy measurements

figure

subplot(2,1,1)
plot(t, rad2deg(psi), 'LineWidth', 1.5)
hold on
plot(t, rad2deg(psi_meas_vect), 'LineWidth', 0.5)
grid on
xlabel('Time (s)')
ylabel('Yaw angle (deg)')
legend('True yaw angle', 'Noisy measurement')
title('Yaw Angle Measurements')

subplot(2,1,2)
plot(t, rad2deg(r), 'LineWidth', 1.5)
hold on
plot(t, rad2deg(r_meas_vect), 'LineWidth', 0.5)
grid on
xlabel('Time (s)')
ylabel('Yaw rate (deg/s)')
legend('True yaw rate', 'Noisy measurement')
title('Yaw Rate Measurements')



%% Problem 4b - Kalman filter results

figure

subplot(3,1,1)
plot(t,rad2deg(simdata(:,6)),'LineWidth',1.5)
hold on
plot(t,rad2deg(x_est_vect(:,1)),'--','LineWidth',1.5)
ylabel('Yaw angle (deg)')
legend('True','Estimated')
title('Yaw angle estimation')
grid on

subplot(3,1,2)
plot(t,rad2deg(simdata(:,3)),'LineWidth',1.5)
hold on
plot(t,rad2deg(x_est_vect(:,2)),'--','LineWidth',1.5)
ylabel('Yaw rate (deg/s)')
legend('True','Estimated')
title('Yaw rate estimation')
grid on

subplot(3,1,3)
plot(t,rad2deg(x_est_vect(:,3)),'LineWidth',1.5)
ylabel('Rudder bias (deg)')
xlabel('Time (s)')
title('Estimated rudder bias')
grid on


