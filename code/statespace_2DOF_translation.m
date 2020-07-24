%% 2DOF state space: 2D translation

% Cd = 2

m = 4;
rho = 2.19*10^(-12);
H = 409000;
R = 6.3781*10^6 + H;
T = 5564.813794;
w_orb = 2*pi/T;
Amin = 0.01;
Fp0 = rho*Amin*(R*w_orb)^2;
M = 5.972*10^(24);
G = 6.67408*10^(-11);

A = [0, 1, 0, 0;...
    0, -2*rho*Amin*w_orb*R/m, -rho*Amin*(w_orb^2)-Fp0/(m*R^2), -2*w_orb/R;...
    0, 0, 0, 1;...
    0, 2*w_orb*R, (w_orb^2)+2*M*G/(R^3), -rho*Amin*w_orb*R/m];

B = [0,0,0;1/(R*m),0,-rho*R*(w_orb^2)/m;0,0,0;0,1/m,0];

C = [1,0,0,0;0,0,1,0];

D = [0,0,0;0,0,0];

states = {'theta','theta_dot','r','r_dot'};
inputs = {'Fp_t','Fp_r','At'};
outputs = {'theta','r'};

sys = ss(A,B,C,D,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

% if it's equal to n (as in n-inputs) all our states are controllable
rank(ctrb(sys))
% if is's equal to n (as in n-inputs) all our states are observable
rank(obsv(sys))

% poles, frquency, ...
damp(sys)

% To analyze the outputs. Useful if rank(ctrb(sys)) < n
% if its rank is equal to m (as in m-outputs) our outputs are controllable
Cout = [C*B C*A*B C*(A^2)*B C*(A^3)*B D];
rank(Cout)