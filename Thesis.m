%% Appendix A ? MATLAB Simulation Code

% Capt M. LUKE GARGASZ
% OPTIMAL SPACECRAFT ATTITUDE CONTROL USING AERODYNAMIC TORQUES
% MARCH 2007
% MASTER'S THESIS: AFIT/GA/ENY/07-M08
% Contact email: luke_gargasz@hotmail.com

function[out]=Thesis(); close all;clear all;clc;
global Case TrueAnomalyInitial Inclination OrbitalRadius
%% CHOOSE CASE TO RUN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Case = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Case 1 is an extreme tumbling/initial offset to explore the uncommanded performance of the spacecraft % CAUTION: TAKES A LONG TIME TO RUN
% Case 2 is a non-tumbling spacecraft in equatorial orbit without saturation avoidance commanded to equilibrium
% Case 3 is a non-tumbling spacecraft in equatorial orbit with saturation avoidance commanded to equilibrium
% Case 4 is a non-tumbling spacecraft in equatorial orbit with saturation avoidance commanded to final offset
% Case 5 is a non-tumbling spacecraft in inclined orbit with saturation avoidance commanded to final offset
% Case 6 is a slightly tumbling spacecraft in equatorial orbit with saturation avoidance commanded to final offset
% CAUTION: TAKES A LONG TIME TO RUN % Orbit Propagation begin coincident with I vector
TrueAnomalyInitial = deg2rad(0);
% PARAMETERS ASSIGNED BASED ON THE CASE CHOSEN ABOVE
if Case == 1
    % Orbital Inclination
    Inclination = deg2rad(0);
    % Initial Euler Angles
    Theta1 = deg2rad(-13); 
    Theta2 = deg2rad(101); 
    Theta3 = deg2rad(-26); % Final Euler Angles
    Theta1C = deg2rad(0); 
    Theta2C = deg2rad(0); 
    Theta3C = deg2rad(0);
    % Initial Rotation Rates
    Omega1 = deg2rad(-3); 
    Omega2 = deg2rad(-2); 
    Omega3 = deg2rad(2.5); 
    % Orbital Altitude in kilometers
    OrbitalAltitude = 300;
    % Number of Orbital Periods to Propogate
    NumberOfPeriods = 1000;
elseif Case == 2 || Case == 3
    % Orbital Inclination
    Inclination = deg2rad(0);
    % Initial Euler Angles
    Theta1 = deg2rad(1); 
    Theta2 = deg2rad(1); 
    Theta3 = deg2rad(1);
    % Final Euler Angles
    Theta1C = deg2rad(0); 
    Theta2C = deg2rad(0); 
    Theta3C = deg2rad(0);
    % Initial Rotation Rates
    Omega1 = deg2rad(0); 
    Omega2 = deg2rad(0); 
    Omega3 = deg2rad(0);
    % Orbital Altitude in kilometers
    OrbitalAltitude = 300;
    % Number of Orbital Periods to Propogate
    NumberOfPeriods = 1; 
elseif Case == 4
    % Orbital Inclination
    Inclination = deg2rad(0);
    % Initial Euler Angles
    Theta1 = deg2rad(7); 
    Theta2 = deg2rad(5); 
    Theta3 = deg2rad(-5);
    % Final Euler Angles
    Theta1C = deg2rad(-3); 
    Theta2C = deg2rad(2); 
    Theta3C = deg2rad(4);
    % Initial Rotation Rates
    Omega1 = deg2rad(0); 
    Omega2 = deg2rad(0); 
    Omega3 = deg2rad(0);
    % Orbital Altitude in kilometers
    OrbitalAltitude = 300;
    % Number of Orbital Periods to Propogate
    NumberOfPeriods = 1; 
elseif Case == 5
    % Orbital Inclination
    Inclination = deg2rad(45);
    % Initial Euler Angles
    Theta1 = deg2rad(0.2); 
    Theta2 = deg2rad(0.5); 
    Theta3 = deg2rad(-0.1);
    % Final Euler Angles
    Theta1C = deg2rad(-0.2); 
    Theta2C = deg2rad(0.5); 
    Theta3C = deg2rad(0.1);
    % Initial Rotation Rates
    Omega1 = deg2rad(0); 
    Omega2 = deg2rad(0); 
    Omega3 = deg2rad(0);
    % Orbital Altitude in kilometers
    OrbitalAltitude = 300;
    % Number of Orbital Periods to Propogate
    NumberOfPeriods = 1; 
elseif Case == 6
    % Orbital Inclination
    Inclination = deg2rad(0);
    % Initial Euler Angles
    Theta1 = deg2rad(7); 
    Theta2 = deg2rad(5); 
    Theta3 = deg2rad(-5);
    % Final Euler Angles
    Theta1C = deg2rad(-3); 
    Theta2C = deg2rad(2); 
    Theta3C = deg2rad(4);
    % Initial Rotation Rates
    Omega1 = deg2rad(0.1); 
    Omega2 = deg2rad(0.1); 
    Omega3 = deg2rad(0.1);
    % Orbital Altitude in kilometers
    OrbitalAltitude = 300;
    % Number of Orbital Periods to Propogate
    NumberOfPeriods = 1;
end


%% Set the initial control panel angles
% ThC1 = deg2rad(-45); ThC2 = deg2rad(-45); 
% ThC3 = deg2rad( 45); ThC4 = deg2rad( 45);

%% Calculate the initial quaternion
% ROTATION MATRIX - ORBITAL TO PRINCIPAL AXES - BODY-THREE 1-2-3 ROTATION
c1 = cos(Theta1); s1 = sin(Theta1); 
c2 = cos(Theta2); s2 = sin(Theta2); 
c3 = cos(Theta3); s3 = sin(Theta3);

R_ORB_PRIN = [ c2*c3 s1*s2*c3+s3*c1 -c1*s2*c3+s3*s1; -c2*s3 -s1*s2*s3+c3*c1 c1*s2*s3+c3*s1; s2 -s1*c2 c1*c2 ];
% Calculate quaternion components
q4 = .5*sqrt(1+trace(R_ORB_PRIN));
q1 = 1/(4*q4)*(R_ORB_PRIN(2,3)-R_ORB_PRIN(3,2)); 
q2 = 1/(4*q4)*(R_ORB_PRIN(3,1)-R_ORB_PRIN(1,3)); 
q3 = 1/(4*q4)*(R_ORB_PRIN(1,2)-R_ORB_PRIN(2,1));

%% Calculate the commanded quaternion

% ROTATION MATRIX - ORBITAL TO PRINCIPAL AXES - BODY-THREE 1-2-3 ROTATION
c1c = cos(Theta1C); s1c = sin(Theta1C); 
c2c = cos(Theta2C); s2c = sin(Theta2C); 
c3c = cos(Theta3C); s3c = sin(Theta3C);

R_ORB_PRIN = [ c2c*c3c s1c*s2c*c3c+s3c*c1c -c1c*s2c*c3c+s3c*s1c; -c2c*s3c -s1c*s2c*s3c+c3c*c1c c1c*s2c*s3c+c3c*s1c;
s2c -s1c*c2c c1c*c2c ];

% Calculate quaternion components
q4C = .5*sqrt(1+trace(R_ORB_PRIN));
q1C = 1/(4*q4)*(R_ORB_PRIN(2,3)-R_ORB_PRIN(3,2)); 
q2C = 1/(4*q4)*(R_ORB_PRIN(3,1)-R_ORB_PRIN(1,3)); 
q3C = 1/(4*q4)*(R_ORB_PRIN(1,2)-R_ORB_PRIN(2,1));

qc = [q1C q2C q3C q4C]';

%% Calculate the orbital period
mu = 398600e9; % m^3/sec^2
OrbitalRadius = 6378000 + OrbitalAltitude*1000; % meters -- 6378 km earth radius + orbit altitude in km 
T = 2*pi*sqrt(OrbitalRadius^3/mu); % Only good for circular orbits

%% ODE45 Call

t_init = 0; t_final = NumberOfPeriods*T;
x0 = [Omega1 Omega2 Omega3 q1 q2 q3 q4 ]';
% ThC1 ThC2 ThC3 ThC4
[t,x]=ode45(@eom,[t_init t_final],x0,[],qc);
%% Plot Results
w = x(:,1:3); q = x(:,4:7); ThC = x(:,8:11);

% Back Euler Angles out of Quaternions
for n = 1:length(t)
    R_ORB_PRIN = [ 1-2*(q(n,2)^2+q(n,3)^2) 2*(q(n,1)*q(n,2)+q(n,3)*q(n,4)) 2*(q(n,1)*q(n,3)- q(n,2)*q(n,4));2*(q(n, 2)*q(n,1)-q(n,3)*q(n,4)) 1-2*(q(n,1)^2+q(n,3)^2) 2*(q(n,2)*q(n,3)+q(n,1)*q(n,4));
        2*(q(n,3)*q(n,1)+q(n,2)*q(n,4)) 2*(q(n,3)*q(n,2)-q(n,1)*q(n,4)) 1- 2*(q(n,1)^2+q(n,2)^2)];
    theta2 = asin(R_ORB_PRIN(3,1));
    theta1 = asin(-R_ORB_PRIN(3,2)/cos(theta2)); 
    theta3 = asin(-R_ORB_PRIN(2,1)/cos(theta2)); 
    theta(n,1:3) = [theta1 theta2 theta3];
    n=n+1;
end

% Select the plots of interest for the Case being analyzed
if Case == 1
    plot(t,w(:,1),'b-','LineWidth',0.5);
    title({'Spacecraft Angular Rates';['',num2str((OrbitalRadius-6378000)/1000), ' km altitude, ',num2str(rad2deg(Inclination)), '^o inclination circular orbit']});
    legend('\omega_1'); xlabel('time (seconds)');ylabel('Angular Rates (rad/sec)');figure; plot(t,w(:,2),'k-','LineWidth',0.5);
    title({'Spacecraft Angular Rates';['',num2str((OrbitalRadius-6378000)/1000), ' km altitude, ',num2str(rad2deg(Inclination)), '^o inclination circular orbit']});
    legend('\omega_2'); xlabel('time (seconds)');ylabel('Angular Rates (rad/sec)');figure; plot(t,w(:,3),'r-','LineWidth',0.5);
    title({'Spacecraft Angular Rates';['',num2str((OrbitalRadius-6378000)/1000), ' km altitude, ',num2str(rad2deg(Inclination)), '^o inclination circular orbit']});
    legend('\omega_3'); xlabel('time (seconds)');ylabel('Angular Rates (rad/sec)'); 
elseif Case == 2 || Case == 3 || Case == 4 || Case == 5 || Case == 6
    plot(t,rad2deg(theta(:,1)),'b--',t,rad2deg(theta(:,2)),'k-',t,rad2deg(theta(:,3)),'r-.','LineWidth',1.75); title({'Spacecraft Euler Angles';['',num2str((OrbitalRadius-6378000)/1000), ' km altitude,',num2str(rad2deg(Inclination)), '^o inclination circular orbit']}); legend('\theta_1','\theta_2','\theta_3');
    xlabel('time (seconds)');ylabel('Euler Angles (Degrees)');xlim([0 5500]);figure;
    plot(t,w(:,1),'b--',t,w(:,2),'k-',t,w(:,3),'r-.','LineWidth',1.75);
    title({'Spacecraft Angular Rates';['',num2str((OrbitalRadius-6378000)/1000), ' km altitude,',num2str(rad2deg(Inclination)), '^o inclination circular orbit']}); legend('\omega_1','\omega_2','\omega_3');
    xlabel('time (seconds)');ylabel('Angular Rates (rad/sec)');xlim([0 5500]);figure;
    plot(t,rad2deg(ThC(:,1)),'g-',t,rad2deg(ThC(:,2)),'b--',t,rad2deg(ThC(:,3)),'k:',t,rad2deg(ThC(:,4)),'r- .','LineWidth',1.75);
    title({'Control Panel Positions';['',num2str((OrbitalRadius-6378000)/1000), ' km altitude, ',num2str(rad2deg(Inclination)), '^o inclination circular orbit']});
    legend('\theta_c_1','\theta_c_2','\theta_c_3','\theta_c_4','Location','East');
    xlabel('time (seconds)');ylabel('Control Panel Position (Degrees)');xlim([0 5500]);ylim([-90 90]); 
end

out=[t,rad2deg(theta),w,q,rad2deg(ThC)]; % DATA FOR OUTPUT TO THE WORKSPACE

%% Equations of Motion Function
function [xdot] = eom(t,x,qc)
    
global Case TrueAnomalyInitial Inclination OrbitalRadius

% Unpack variables
w = [x(1); x(2); x(3)];
q1=x(4);
q2=x(5);
q3=x(6);
q4=x(7); 
q=[q1;q2;q3]; 
% ThC1 = x(8); 
% ThC2 = x(9); 
% ThC3 = x(10); 
% ThC4 = x(11);

%% CALCULATE ATMOSPHERIC DENSITY

% 300 km orbit values taken from Vallado % this needs to be changed to a lookup table so the orbital altitude can be varied
rho_not = 0.219e-11;
h = (OrbitalRadius-6378000)/1000; h_not = 450;
H = 60.8228;

rho = rho_not*exp((h-h_not)/H);
%% ROTATION MATRIX - ORBITAL TO PRINCIPAL AXES - BODY-THREE 1-2-3 ROTATION

R_ORB_PRIN = [1-2*(q2^2+q3^2) 2*(q1*q2+q3*q4) 2*(q1*q3-q2*q4); 2*(q2*q1-q3*q4) 1-2*(q1^2+q3^2) 2*(q2*q3+q1*q4); 2*(q3*q1+q2*q4) 2*(q3*q2-q1*q4) 1-2*(q1^2+q2^2)];
%% Calculate the local atmospheric velocity vector
% Constants
we = 7.27E-5; % radians/sec 
mu = 398600e9; % m^3/sec^2

% Calculate the Orbital Velocity
OrbitalVelocity = sqrt(mu/OrbitalRadius); % m/s % only good for circular orbits

%Calculate the true anomaly
mean_motion = sqrt(mu/OrbitalRadius^3); 
nu = mean_motion*t+TrueAnomalyInitial;

R = OrbitalRadius; V = OrbitalVelocity;
VR = V*(1-(we*R/V)*cos(Inclination))*R_ORB_PRIN*[-1; (we*R/V)*sin(Inclination)*cos(nu); 0];

VRunit = VR/norm(VR);

Vb = 0.05*VR; % This is just an estimate and can be expanded 

%% Spacecraft Measurements

% Spacecraft Bus Measurements
d1 = 0.15; % distance from Center of Mass to the edge of the spacecraft bus 
d2=0.05; 
Acube1 = 2*d1*2*d2;
Acube2 = 2*d1*2*d1;% Area of one side of the cube

% Control Panel Measurements
%L = .2; % half the length of the control panels
%f = .25/2; % distance from the center of mass to the center of the control panels 
%width = .25; % width of the control panels
%Apanel = width*2*L; % Area of one control panel
%% Spacecraft Bus Geometry
% Front Side (+ x-axis side)
r1 = [d1;0;0]; n1 = [-1;0;0]; alpha1 = acos(dot(VRunit,n1));
% Left Side (+ y-axis side)
r2 = [0;d2;0]; n2 = [0;-1;0]; alpha2 = acos(dot(VRunit,n2));
% Right Side
r3 = [0;-d2;0]; n3 = [0;1;0]; alpha3 = acos(dot(VRunit,n3));
% Top Side
r4 = [0;0;-d2]; n4 = [0;0;1]; alpha4 = acos(dot(VRunit,n4));
% Bottom Side (+ z-axis side)
r5 = [0;0;d2]; n5 = [0;0;-1]; alpha5 = acos(dot(VRunit,n5));
% Back Side
r6 = [-d1;0;0]; n6 = [1;0;0]; alpha6 = acos(dot(VRunit,n6));
%% Calculate Ap
Ap1 = heaviside(cos(alpha1 ))*cos(alpha1 )*Acube1; 
Ap2 = heaviside(cos(alpha2 ))*cos(alpha2 )*Acube1; 
Ap3 = heaviside(cos(alpha3 ))*cos(alpha3 )*Acube2; 
Ap4 = heaviside(cos(alpha4 ))*cos(alpha4 )*Acube2; 
Ap5 = heaviside(cos(alpha5 ))*cos(alpha5 )*Acube2; 
Ap6 = heaviside(cos(alpha6 ))*cos(alpha6 )*Acube2; 
% Ap7f = heaviside(cos(alpha7f ))*cos(alpha7f )*Apanel; 
% Ap7r = heaviside(cos(alpha7r ))*cos(alpha7r )*Apanel; 
% Ap8f = heaviside(cos(alpha8f ))*cos(alpha8f )*Apanel; 
% Ap8r = heaviside(cos(alpha8r ))*cos(alpha8r )*Apanel; 
% Ap9f = heaviside(cos(alpha9f ))*cos(alpha9f )*Apanel; 
% Ap9r = heaviside(cos(alpha9r ))*cos(alpha9r )*Apanel; 
% Ap10f = heaviside(cos(alpha10f))*cos(alpha10f)*Apanel; 
% Ap10r = heaviside(cos(alpha10r))*cos(alpha10r)*Apanel;
%% Calculate Cp
% if/else statement necessary to account for Ap being zero and then trying to divide by zero
if Ap1 <= 0; Cp1 = [0;0;0];
else
    Cp1=heaviside(cos(alpha1))*cos(alpha1)*Acube1*r1/Ap1; 
end

if Ap2 <= 0; Cp2 = [0;0;0];
else
    Cp2 = heaviside(cos(alpha2))*cos(alpha2)*Acube1*r2/Ap2; 
end

if Ap3 <= 0; Cp3 = [0;0;0];
else
    Cp3 = heaviside(cos(alpha3))*cos(alpha3)*Acube2*r3/Ap3; 
end

if Ap4 <= 0; Cp4 = [0;0;0];
else
    Cp4 = heaviside(cos(alpha4))*cos(alpha4)*Acube2*r4/Ap4; 
end

if Ap5 <= 0; Cp5 = [0;0;0];
else
    Cp5 = heaviside(cos(alpha5))*cos(alpha5)*Acube2*r5/Ap5; 
end

if Ap6 <= 0; Cp6 = [0;0;0];
else
    Cp6 = heaviside(cos(alpha6))*cos(alpha6)*Acube2*r6/Ap6; 
end

% if Ap7f <= 0; Cp7f = [0;0;0];
% else Cp7f = heaviside(cos(alpha7f))*cos(alpha7f)*Apanel*r7/Ap7f; end
% 
% if Ap7r <= 0; Cp7r = [0;0;0];
% else Cp7r = heaviside(cos(alpha7r))*cos(alpha7r)*Apanel*r7/Ap7r; end
% 
% if Ap8f <= 0; Cp8f = [0;0;0];
% else Cp8f = heaviside(cos(alpha8f))*cos(alpha8f)*Apanel*r8/Ap8f; end
% 
% if Ap8r <= 0; Cp8r = [0;0;0];
% else Cp8r = heaviside(cos(alpha8r))*cos(alpha8r)*Apanel*r8/Ap8r; end
% 
% if Ap9f <= 0; Cp9f = [0;0;0];
% else Cp9f = heaviside(cos(alpha9f))*cos(alpha9f)*Apanel*r9/Ap9f; end 
% 
% if Ap9r <= 0; Cp9r = [0;0;0];
% else Cp9r = heaviside(cos(alpha9r))*cos(alpha9r)*Apanel*r9/Ap9r; end
% 
% if Ap10f <= 0; Cp10f = [0;0;0];
% else Cp10f = heaviside(cos(alpha10f))*cos(alpha10f)*Apanel*r10/Ap10f; end
% 
% if Ap10r <= 0; Cp10r = [0;0;0];
% else Cp10r = heaviside(cos(alpha10r))*cos(alpha10r)*Apanel*r10/Ap10r; end

%% Calculate Gp
Gp1 = heaviside(cos(alpha1 ))*cos(alpha1 )*Acube1* cross(r1, n1);
Gp2 = heaviside(cos(alpha2 ))*cos(alpha2 )*Acube1* cross(r2, n2);
Gp3 = heaviside(cos(alpha3 ))*cos(alpha3 )*Acube2* cross(r3, n3);
Gp4 = heaviside(cos(alpha4 ))*cos(alpha4 )*Acube2* cross(r4, n4);
Gp5 = heaviside(cos(alpha5 ))*cos(alpha5 )*Acube2* cross(r5, n5);
Gp6 = heaviside(cos(alpha6 ))*cos(alpha6 )*Acube2* cross(r6, n6); 
% Gp7f = heaviside(cos(alpha7f ))*cos(alpha7f )*Apanel*cross(r7, n7f); 
% Gp7r = heaviside(cos(alpha7r ))*cos(alpha7r )*Apanel*cross(r7, n7r); 
% Gp8f = heaviside(cos(alpha8f ))*cos(alpha8f )*Apanel*cross(r8, n8f); 
% Gp8r = heaviside(cos(alpha8r ))*cos(alpha8r )*Apanel*cross(r8, n8r); 
% Gp9f = heaviside(cos(alpha9f ))*cos(alpha9f )*Apanel*cross(r9, n9f); 
% Gp9r = heaviside(cos(alpha9r ))*cos(alpha9r )*Apanel*cross(r9, n9r); 
% Gp10f = heaviside(cos(alpha10f))*cos(alpha10f)*Apanel*cross(r10,n10f); 
% Gp10r = heaviside(cos(alpha10r))*cos(alpha10r)*Apanel*cross(r10,n10r);
%% Calculate Gpp
Gpp1 = heaviside(cos(alpha1 ))*cos(alpha1 )^2*Acube1* cross(r1, n1); 
Gpp2 = heaviside(cos(alpha2 ))*cos(alpha2 )^2*Acube1* cross(r2, n2); 
Gpp3 = heaviside(cos(alpha3 ))*cos(alpha3 )^2*Acube2* cross(r3, n3); 
Gpp4 = heaviside(cos(alpha4 ))*cos(alpha4 )^2*Acube2* cross(r4, n4); 
Gpp5 = heaviside(cos(alpha5 ))*cos(alpha5 )^2*Acube2* cross(r5, n5); 
Gpp6 = heaviside(cos(alpha6 ))*cos(alpha6 )^2*Acube2* cross(r6, n6); 
% Gpp7f = heaviside(cos(alpha7f ))*cos(alpha7f )^2*Apanel*cross(r7, n7f); 
% Gpp7r = heaviside(cos(alpha7r ))*cos(alpha7r )^2*Apanel*cross(r7, n7r); 
% Gpp8f = heaviside(cos(alpha8f ))*cos(alpha8f )^2*Apanel*cross(r8, n8f); 
% Gpp8r = heaviside(cos(alpha8r ))*cos(alpha8r )^2*Apanel*cross(r8, n8r); 
% Gpp9f = heaviside(cos(alpha9f ))*cos(alpha9f )^2*Apanel*cross(r9, n9f); 
% Gpp9r = heaviside(cos(alpha9r ))*cos(alpha9r )^2*Apanel*cross(r9, n9r); 
% Gpp10f = heaviside(cos(alpha10f))*cos(alpha10f)^2*Apanel*cross(r10,n10f); 
% Gpp10r = heaviside(cos(alpha10r))*cos(alpha10r)^2*Apanel*cross(r10,n10r);
%% Calculate the disturbance torque and control torque % ACCOMODATION COEFFICIENTS - estimates
sig_t = 0.8; sig_n = 0.8;
gd1 = rho*norm(VR)^2*(sig_t*Ap1* cross(Cp1,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp1 +(2- sig_n-sig_t)*Gpp1);
gd2 = rho*norm(VR)^2*(sig_t*Ap2* cross(Cp2,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp2 +(2- sig_n-sig_t)*Gpp2);
gd3 = rho*norm(VR)^2*(sig_t*Ap3* cross(Cp3,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp3 +(2- sig_n-sig_t)*Gpp3);
gd4 = rho*norm(VR)^2*(sig_t*Ap4* cross(Cp4,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp4 +(2- sig_n-sig_t)*Gpp4);
gd5 = rho*norm(VR)^2*(sig_t*Ap5* cross(Cp5,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp5 +(2- sig_n-sig_t)*Gpp5);
gd6 = rho*norm(VR)^2*(sig_t*Ap6* cross(Cp6,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp6 +(2- sig_n-sig_t)*Gpp6);
% gc7f = rho*norm(VR)^2*(sig_t*Ap7f* cross(Cp7f,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp7f +(2- sig_n-sig_t)*Gpp7f);
% gc7r = rho*norm(VR)^2*(sig_t*Ap7r* cross(Cp7r,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp7r +(2- sig_n-sig_t)*Gpp7r);
% gc8f = rho*norm(VR)^2*(sig_t*Ap8f* cross(Cp8f,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp8f +(2- sig_n-sig_t)*Gpp8f);
% gc8r = rho*norm(VR)^2*(sig_t*Ap8r* cross(Cp8r,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp8r +(2- sig_n-sig_t)*Gpp8r);
% gc9f = rho*norm(VR)^2*(sig_t*Ap9f* cross(Cp9f,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp9f +(2- sig_n-sig_t)*Gpp9f);
% gc9r = rho*norm(VR)^2*(sig_t*Ap9r* cross(Cp9r,VRunit) +sig_n*(norm(Vb)/norm(VR))*Gp9r +(2- sig_n-sig_t)*Gpp9r);
% gc10f = rho*norm(VR)^2*(sig_t*Ap10f*cross(Cp10f,VRunit)+sig_n*(norm(Vb)/norm(VR))*Gp10f+(2- sig_n-sig_t)*Gpp10f);
% gc10r = rho*norm(VR)^2*(sig_t*Ap10r*cross(Cp10r,VRunit)+sig_n*(norm(Vb)/norm(VR))*Gp10r+(2- sig_n-sig_t)*Gpp10r);

% Disturbance torque on the spacecraft bus
gd = gd1 + gd2 + gd3 + gd4 + gd5 + gd6
% % Control torque generated by the control panels
% gc=gc7f+gc7r + gc8f+gc8r + gc9f+gc9r + gc10f+gc10r;
% %% Select Gains for Case Being Analyzed 
% if Case == 1 % Uncontrolled Case
%     K =0; C =0; kf = 0; kp = 0;
% elseif Case == 2 % Controlled Case without Saturation Avoidance K = 0.001*eye(3);
%     C = 0.2*eye(3);
%     kf = 0.02;
%     kp = 0.0;
% elseif Case == 3 || Case == 4 || Case == 5 || Case == 6 % Controlled Cases with Saturation Avoidance
%     K = 0.001*eye(3); C = 0.2*eye(3); kf = 0.02;
%     kp = 0.02;
% end