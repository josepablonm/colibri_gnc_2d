%% 2DOF state space: 2D translation

%%

% Cd = 2
H = 375000;
R = 6.3781*10^6 + H;
M = 5.972*10^(24);
G = 6.67408*10^(-11);
w_orb = sqrt(M*G/(R^3));
T = 2*pi/w_orb;
%% Cube Sat's parameters
m_sat = 4;
Amin = 0.01;
rho = 2.64*10^(-12);
Fp0 = rho*Amin*(R*w_orb)^2;
%% Cube Sat's analysis

% states
% x = [ theta(t) ; theta_dot(t) ; r(t) ; r_dot(t) ]

A = [0, 1, 0, 0;...
    0, -2*rho*Amin*w_orb*R/m_sat, -rho*Amin*(w_orb^2)/m_sat - Fp0/(m_sat*R^2), -2*w_orb/R;...
    0, 0, 0, 1;...
    0, 2*w_orb*R, (w_orb^2)+2*M*G/(R^3), -rho*Amin*w_orb*R/m_sat];

% input
% u(t) = [ Fp_c(t) ] 

B = [0;1/(R*m_sat);0;0];

% outputs
% y(t) = [theta(t) ; r(t)]
C = [1,0,0,0;0,0,1,0];

D = [0;0];

% labels (these are not required)
states = {'theta_sat','thetadot_sat','r_sat','rdot_sat'};
inputs = {'Fp'};
outputs = {'theta_sat','r_sat'};

% linear system
sys_mod = ss(A,B,C,D,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

% if is's equal to n (as in n-inputs) all our states are observable
state_obsv = rank(obsv(sys_mod));
if state_obsv ~= length(A(:,1))
    disp('Not all states are observable')
else
    disp('OBSERVABLE')
end

% if it's equal to n (as in n-inputs) all our states are controllable
state_ctrl = rank(ctrb(sys_mod));
if state_ctrl ~= length(A(:,1))
    disp('Not all stetes are controllable')
    
    Cout = [C*B C*A*B C*(A^2)*B C*(A^3)*B D];
    out_ctrl = rank(Cout);
    
    % if it's equal to m (as in m-outputs) all our outputs are controllable
    if out_ctrl == length(C(:,1))
        disp('Outputs are controllable')
    end
else
    disp('CONTROLLABLE')    
end

% poles, frquency, ...
damp(sys_mod)

%% Control design

% state weights
%Q = C'*C*0.5;
Q = [100,0,0,0;...
    0,0,0,0;...
    0,0,0.5,0;...
    0,0,0,0];

% input weights
Ra = [5000];

% K matrix is calculated with LQR
[K,S,e] = lqr(A,B,Q,Ra);

%% Control evaluation
open_system('linear_ss_control_reg.slx');

set_param('linear_ss_control_reg/A','Gain',mat2str(A));
set_param('linear_ss_control_reg/B','Gain',mat2str(B));
set_param('linear_ss_control_reg/C','Gain',mat2str(C));
set_param('linear_ss_control_reg/K','Gain',mat2str(K));
set_param('linear_ss_control_reg/Fp_n','Value',num2str(Fp0));

set_param('linear_ss_control_reg/saturation','UpperLimit',num2str(20*10^-6 - Fp0));
set_param('linear_ss_control_reg/saturation','LowerLimit',num2str(-Fp0));

set_param('linear_ss_control_reg', 'StopTime', '1500');%num2str(T));%

out = sim('linear_ss_control_reg.slx');

tiempo = out.tout;
Fp = out.simout.data(:,1);
dtheta = out.simout.data(:,2);
dr = out.simout.data(:,3);

figure
ax1 = subplot(3,1,1);
plot(ax1, tiempo, Fp)
ylabel(ax1, 'Thrust [N]')
title(ax1, 'Linearized cubesat model')

ax2 = subplot(3,1,2);
plot(ax2, tiempo, dr, tiempo, zeros(length(tiempo),1),'--')
ylabel(ax2, 'radial deviation [m]')

ax3 = subplot(3,1,3);
plot(ax3, tiempo, dtheta, tiempo, zeros(length(tiempo),1),'--')
ylabel(ax3, 'transverse deviation [rad]')

xlabel('time [seconds]')

%% State observer

% closed-loop system poles
poles = eig([(A-B*K)]);

% you should look at the closed-loop poles and
% pick new ones accordingly
% for Pakal we want to observe faster than the control (aprox 10 times)
% and we don't want oscilation in our response
obs_poles = [-0.2 0 -0.09 -0.089];

Ke = place(A',C',obs_poles)';

open_system('linear_ss_control_reg_obs.slx');

set_param('linear_ss_control_reg_obs/A','Gain',mat2str(A));
set_param('linear_ss_control_reg_obs/B','Gain',mat2str(B));
set_param('linear_ss_control_reg_obs/C','Gain',mat2str(C));
set_param('linear_ss_control_reg_obs/K','Gain',mat2str(K));
set_param('linear_ss_control_reg_obs/Fp_n','Value',num2str(Fp0));

set_param('linear_ss_control_reg_obs/A1','Gain',mat2str(A));
set_param('linear_ss_control_reg_obs/B1','Gain',mat2str(B));
set_param('linear_ss_control_reg_obs/C1','Gain',mat2str(C));
set_param('linear_ss_control_reg_obs/Ke','Gain',mat2str(Ke));

set_param('linear_ss_control_reg_obs/saturation','UpperLimit',num2str(20*10^-6 - Fp0));
set_param('linear_ss_control_reg_obs/saturation','LowerLimit',num2str(-Fp0));

set_param('linear_ss_control_reg_obs', 'StopTime', '1500');%num2str(T));%

out_obs = sim('linear_ss_control_reg.slx');

tiempo_obs = out_obs.tout;
Fp_obs = out_obs.simout.data(:,1);
dtheta_obs = out_obs.simout.data(:,2);
dr_obs = out_obs.simout.data(:,3);

figure
ax1 = subplot(3,1,1);
plot(ax1, tiempo, Fp, tiempo_obs, Fp, '-.r')
ylabel(ax1, 'Thrust [N]')
title(ax1, 'Linearized cubesat model')

ax2 = subplot(3,1,2);
plot(ax2, tiempo, dr, tiempo, zeros(length(tiempo),1),'--',tiempo_obs, dr_obs, '-.r')
ylabel(ax2, 'radial deviation [m]')

ax3 = subplot(3,1,3);
plot(ax3, tiempo, dtheta, tiempo, zeros(length(tiempo),1),'--',tiempo_obs, dtheta_obs, '-.r')
ylabel(ax3, 'transverse deviation [rad]')

xlabel('time [seconds]')
legend(ax3,'full state','target','estimated')
