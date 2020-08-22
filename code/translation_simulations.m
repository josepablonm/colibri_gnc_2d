%% SOME SETTINGS
M = 5.972*10^(24);
G = 6.67408*10^(-11);

H = 409000;
R = 6.3781*10^6 + H;
w_orb = sqrt(M*G/(R^3));
Amin = 0.01;
rho = 2.19*10^(-12);
Fp0 = rho*Amin*(R*w_orb)^2;
%Fp0 = 0;
%% SIMULATIONS

open_system('sat_orbit_no_control_SIM.slx');

set_param('sat_orbit_no_control_SIM/F_p','Value',num2str(Fp0));

set_param('sat_orbit_no_control_SIM/theta_E','Value','0');
e0 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','0.02*pi/180');
e002 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','0.05*pi/180');
e005 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','pi/1800');
e01 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','0.2*pi/180');
e02 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','0.25*pi/180');
e025 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','0.5*pi/180');
e05 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','0.6*pi/180');
e06 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','0.75*pi/180');
e075 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','pi/180');
e1 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','1.25*pi/180');
e125 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','1.5*pi/180');
e15 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','1.6*pi/180');
e16 = sim('sat_orbit_no_control_SIM.slx');
set_param('sat_orbit_no_control_SIM/theta_E','Value','2*pi/180');
e2 = sim('sat_orbit_no_control_SIM.slx');

%% PLOTS
ax1 = subplot(2,1,1);
hold (ax1, 'on')
ax2 = subplot(2,1,2);
hold (ax2, 'on')

time0 = e0.tout(:,1);
r0 = e0.sat_data(:,1);
th0 = mod(e0.sat_data(:,2),2*pi);
time002 = e002.tout(:,1);
r002 = e002.sat_data(:,1);
th002 = mod(e002.sat_data(:,2),2*pi);
time005 = e005.tout(:,1);
r005 = e005.sat_data(:,1);
th005 = mod(e005.sat_data(:,2),2*pi);
time01 = e01.tout(:,1);
r01 = e01.sat_data(:,1);
th01 = mod(e01.sat_data(:,2),2*pi);
time02 = e02.tout(:,1);
r02 = e02.sat_data(:,1);
th02 = mod(e02.sat_data(:,2),2*pi);
time025 = e025.tout(:,1);
r025 = e025.sat_data(:,1);
th025 = mod(e025.sat_data(:,2),2*pi);
time05 = e05.tout(:,1);
r05 = e05.sat_data(:,1);
th05 = mod(e05.sat_data(:,2),2*pi);
time06 = e06.tout(:,1);
r06 = e06.sat_data(:,1);
th06 = mod(e06.sat_data(:,2),2*pi);
time075 = e075.tout(:,1);
r075 = e075.sat_data(:,1);
th075 = mod(e075.sat_data(:,2),2*pi);
time1 = e1.tout(:,1);
r1 = e1.sat_data(:,1);
th1 = mod(e1.sat_data(:,2),2*pi);
time125 = e125.tout(:,1);
r125 = e125.sat_data(:,1);
th125 = mod(e125.sat_data(:,2),2*pi);
time15 = e15.tout(:,1);
r15 = e15.sat_data(:,1);
th15 = mod(e15.sat_data(:,2),2*pi);
time16 = e16.tout(:,1);
r16 = e16.sat_data(:,1);
th16 = mod(e16.sat_data(:,2),2*pi);
time2 = e2.tout(:,1);
r2 = e2.sat_data(:,1);
th2 = mod(e2.sat_data(:,2),2*pi);

plot(ax1,time0,r0,'Color',1/255*[0,104,87])
plot(ax1,time002,r002,'Color',1/255*[200,200,200])
plot(ax1,time005,r005,'Color',1/255*[10,10,10])
plot(ax1,time01,r01,'Color',1/255*[255,0,0])
plot(ax1,time02,r02,'Color',1/255*[0,255,0])
plot(ax1,time025,r025,'Color',1/255*[0,0,255])
plot(ax1,time05,r05,'Color',1/255*[0,0,100])
plot(ax1,time06,r06,'Color',1/255*[100,0,0])
plot(ax1,time075,r075,'Color',1/255*[0,100,0])
plot(ax1,time1,r1,'Color',1/255*[50,50,50])
plot(ax1,time125,r125,'Color',1/255*[0,255,87])
plot(ax1,time15,r15,'Color',1/255*[255,104,87])
plot(ax1,time16,r16,'Color',1/255*[44,66,77])
plot(ax1,time2,r2,'Color',1/255*[22,66,0])
title(ax1,'Non controlled propulsion with LEO drag')
ylabel(ax1, 'r [m]')
plot(ax2,time0,th0,'Color',1/255*[0,104,87])
plot(ax2,time002,th002,'Color',1/255*[200,200,200])
plot(ax2,time005,th005,'Color',1/255*[10,10,10])
plot(ax2,time01,th01,'Color',1/255*[255,0,0])
plot(ax2,time02,th02,'Color',1/255*[0,255,0])
plot(ax2,time025,th025,'Color',1/255*[0,0,255])
plot(ax2,time05,th05,'Color',1/255*[0,0,100])
plot(ax2,time06,th06,'Color',1/255*[100,0,0])
plot(ax2,time075,th075,'Color',1/255*[0,100,0])
plot(ax2,time1,th1,'Color',1/255*[50,50,50])
plot(ax2,time125,th125,'Color',1/255*[0,255,87])
plot(ax2,time15,th15,'Color',1/255*[255,104,87])
plot(ax2,time16,th16,'Color',1/255*[44,66,77])
plot(ax2,time2,th2,'Color',1/255*[22,66,0])
ylabel(ax2, 'theta [rad]')

xlabel(ax2, 'time [s]')
legend('e=0°',...
    'e=0.02°',...
    'e=0.05°',...
    'e=0.1°',...
    'e=0.2°',...
    'e=0.25°',...
    'e=0.5°',...
    'e=0.6°',...
    'e=0.75°',...
    'e=1°',...
    'e=1.25°',...
    'e=1.5°',...
    'e=1.6°',...
    'e=2°')
hold (ax1, 'off')
hold (ax2, 'off')