%% 2DOF state space: 2D translation

%%

% Cd = 2
H = 409000;
R = 6.3781*10^6 + H;
M = 5.972*10^(24);
G = 6.67408*10^(-11);
w_orb = sqrt(M*G/(R^3));
T = 2*pi/w_orb;
%% Cube Sat's parameters
m_sat = 4;
Amin = 0.01;
rho = 2.19*10^(-12);
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
end

% poles, frquency, ...
damp(sys_mod)

%% Control design

% state weights
%Q = C_sat'*C_sat;
Q = [1,0,0,0;...
    0,1,0,0;...
    0,0,1,0;...
    0,0,0,1];

% input weights
Ra = [1];

% K matrix is calculated with LQR
[K,S,e] = lqr(A,B,Q,Ra);


%rk = -1/(C*((A-B*K)^-1)*B);

% closed-loop linear system
sys = ss(A-B*K, B, C, D);

%% Control evaluation

t = 0:0.01:1000;
%fp = 0.01*ones(size(t));
%lsim(sys, u, t, x0);

% initial conditions
x0 = [0;0;-1*10^(-6);0];

% system response
[y,t,x]=initial(sys,x0,t);

fig = figure();
h = subplot(2, 1, 1);
% output plot
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','theta deviation (rad)')
set(get(AX(2),'Ylabel'),'String','r deviation (m)')
title('Step Response with LQR Control')
xlabel('time [s]')


u = zeros(length(x),1);
for i=2:length(x)
    dt = t(i)-t(i-1);
    dx2 = x(i,2) - x(i-1,2);
    tmp1 = R*rho*w_orb*Amin*(2*R*x(i,2)+w_orb*x(i,3));
    u(i)= tmp1 + Fp0*x(i,3)/R + 2*w_orb*x(i,4)*m_sat;
end

h = subplot(2, 1, 2);
plot(t,u)
xlabel('time [s]')
ylabel('Fp [N]')
title('Change in propulsion force')