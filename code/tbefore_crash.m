%% INPUT DATA
radio = e002.sat_data(:,1);
thetas = e002.sat_data(:,2);
H = 409000;
R0 = 6.3781*10^6 + H;
theta0 = e002.sat_data(:,6);

tiempo = e002.tout;

angulo = 2*pi/180;
% dy = rs - rp*cos(ths - thp);

%% LOOP
t_choque = 0;
for i=1:length(tiempo)
    
    dx_cm = R0*sin(thetas(i) - theta0(i));
    dy_cm = radio(i) - R0*cos(thetas(i) - theta0(i));
    
    if abs(dx_cm) >= (0.01 + dy_cm*sin(angulo))
        t_choque = tiempo(i);
        fprintf('Choque en x\n')
        break;
    end
    if abs(dy_cm) >= (0.0125 + dx_cm*sin(angulo))
        t_choque = tiempo(i);
        fprintf('Choque en y\n')
        fprintf('%f',t_choque)
        break;
    end
    
end
