tiempo = out.tout;
Fp = out.data.data(:,1);
dtheta = out.data.data(:,8);
dr = out.data.data(:,9);%10

figure
ax1 = subplot(3,1,1);
plot(ax1, tiempo, Fp)
ylabel(ax1, 'Thrust [N]')
title(ax1, 'Controlled nonlinear cubesat model with linear observer')

ax2 = subplot(3,1,2);
plot(ax2, tiempo, dr, tiempo, zeros(length(tiempo),1),'--')
ylabel(ax2, 'radial deviation [m]')

ax3 = subplot(3,1,3);
plot(ax3, tiempo, dtheta, tiempo, zeros(length(tiempo),1),'--')
ylabel(ax3, 'transverse deviation [rad]')

xlabel('time [seconds]')