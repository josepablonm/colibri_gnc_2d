%% 2DOF state space: 2D translation

%%

% Cd = 2
H = 409000;
R = 6.3781*10^6 + H;
T = 5564.813794;
w_orb = 2*pi/T;
Amin = 0.01;
Fp0 = rho*Amin*(R*w_orb)^2;
M = 5.972*10^(24);
G = 6.67408*10^(-11);

%% Cube Sat's parameters
m_sat = 4;
rho = 2.19*10^(-12);
%% Cube Sat analysis
A_sat = [0, 1, 0, 0;...
    0, -2*rho*Amin*w_orb*R/m_sat, -rho*Amin*(w_orb^2)-Fp0/(m_sat*R^2), -2*w_orb/R;...
    0, 0, 0, 1;...
    0, 2*w_orb*R, (w_orb^2)+2*M*G/(R^3), -rho*Amin*w_orb*R/m_sat];

B_sat = [0,0,0;1/(R*m_sat),0,-rho*R*(w_orb^2)/m_sat;0,0,0;0,1/m_sat,0];

C_sat = [1,0,0,0;0,0,1,0];

D_sat = [0,0,0;0,0,0];

states = {'theta_sat','thetadot_sat','r_sat','rdot_sat'};
inputs = {'Fp_t','Fp_r','At'};
outputs = {'theta_sat','r_sat'};

sys_sat = ss(A_sat,B_sat,C_sat,D_sat,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

% if it's equal to n (as in n-inputs) all our states are controllable
rank(ctrb(sys_sat))
% if is's equal to n (as in n-inputs) all our states are observable
rank(obsv(sys_sat))

% poles, frquency, ...
damp(sys_sat)

% To analyze the outputs. Useful if rank(ctrb(sys)) < n
% if its rank is equal to m (as in m-outputs) our outputs are controllable
Cout = [C_sat*B_sat C_sat*A_sat*B_sat C_sat*(A_sat^2)*B_sat C_sat*(A_sat^3)*B_sat D_sat];
rank(Cout)

%% Payload's parameters
m_p = 4;
rho_p = 0;
%% Payload analysis
A_p = [0, 1, 0, 0;...
    0, 0, 0, -2*w_orb/R;...
    0, 0, 0, 1;...
    0, 2*w_orb*R, (w_orb^2)+2*M*G/(R^3), 0];

B_p = zeros(4,1);
C_p = [1,0,0,0;0,0,1,0];
D_p = zeros(2,1);

states = {'theta_sat','thetadot_sat','r_sat','rdot_sat'};
inputs = {'-'};
outputs = {'theta_p','r_p'};

sys_p = ss(A_p, B_p, C_p, D_p,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

damp(sys_p)