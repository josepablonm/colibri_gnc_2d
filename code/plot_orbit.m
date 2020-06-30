%% PLOT_ORBIT
function [movieVector] = plot_orbit(out, filename);
%PLOT_ORBIT function to plot CubeSat trajectory and orientation
% 

    %% we get data from output structure

    % CubeSats PM position vector
    r = out.sat_data(:,1);
    theta = out.sat_data(:,2);

    % Cubesats oritentation
    theta_sat = out.sat_data(:,3);

    % time
    t = out.tout;

    %% Constants

    G=6.67408*10^-11; %Gravity constant m3 kg-1 s-1
    R_earth=6.3781*10^6; %Radius earth meters
    M_earth=5.972*10^24; %mass earth kg
    H=409000; %altitude 409 km
    r0=R_earth+H;
    w0=sqrt(G*M_earth/r0^3);

    %% We reduce the amount of data
    DT = length(t) % time vector length
    step = 500;
    x = zeros(fix(DT/(step)),1);
    y = zeros(fix(DT/(step)),1);
    t_k = zeros(fix(DT/(step)),1);
    th_k = zeros(fix(DT/(step)),1);
    th_s = zeros(fix(DT/(step)),1);
    i=1;
    j=1;
    while i< length(r)
        x(j) = r(i)*cos(theta(i));
        y(j) = r(i)*sin(theta(i));
        t_k(j) = t(i);
        th_k(j) = theta(i);
        th_s(j) = theta_sat(i);
        i=i+step;
        j=j+1;
    end
    % to plot the Earth
    th = 0:pi/50:2*pi;
    xunit = R_earth*cos(th);
    yunit = R_earth*sin(th);
    %% We plot the data
    fig = figure;
    axis equal
    for i=1:DT/(step)
        clf
        axis equal
        axis([x(i)-2*10^6 x(i)+2*10^6 y(i)-2*10^6 y(i)+2*10^6])
        hold on
        % earth
        plot(xunit, yunit, 'blue','Markersize',6,'HandleVisibility','off');
        % earth centered rf
        quiver(0,0,1,0,10^6,'linewidth',1,'maxheadsize',1,'color','blue');
        quiver(0,0,0,1,10^6,'linewidth',1,'maxheadsize',1,'color','blue','HandleVisibility','off');
        % orbit trajectory
        plot(x,y,'r.');
        %plot(x(i),y(i),'m','Marker','o','Markersize',6,'HandleVisibility','off');
        % sat
        x_1 = x(i) - 0.158114*cos(th_s(i) + atan(1/3))*3*10^6;
        x_2 = x(i) + 0.158114*cos(th_s(i) - atan(1/3))*3*10^6;
        x_3 = x(i) + 0.158114*cos(th_s(i) + atan(1/3))*3*10^6;
        x_4 = x(i) - 0.158114*cos(th_s(i) - atan(1/3))*3*10^6;
        sat_x = [x_1 x_2 x_3 x_4];
        y_1 = y(i) - 0.158114*sin(th_s(i) + atan(1/3))*3*10^6;
        y_2 = y(i) + 0.158114*sin(th_s(i) - atan(1/3))*3*10^6;
        y_3 = y(i) + 0.158114*sin(th_s(i) + atan(1/3))*3*10^6;
        y_4 = y(i) - 0.158114*sin(th_s(i) - atan(1/3))*3*10^6;
        sat_y = [y_1 y_2 y_3 y_4];
        patch(sat_x,sat_y,'m')
        % orbital rf
        x_x = 1*cos(th_k(i)+0.5*pi);
        x_y = 1*sin(th_k(i)+0.5*pi);
        quiver(x(i),y(i),x_x,x_y,10^6,'linewidth',1,'maxheadsize',1,'color','green');
        y_x = 1*cos(th_k(i)+pi);
        y_y = 1*sin(th_k(i)+pi);
        quiver(x(i),y(i),y_x,y_y,10^6,'linewidth',1,'maxheadsize',1,'color','green','HandleVisibility','off');
        %sat reference frame
        quiver(x(i),y(i),cos(th_s(i)),sin(th_s(i)),10^6,'linewidth',1,'maxheadsize',1,'color','k');
        quiver(x(i),y(i),cos(th_s(i)+0.5*pi),sin(th_s(i)+0.5*pi),10^6,'linewidth',1,'maxheadsize',1,'color','k','HandleVisibility','off');
        grid on
        xlabel('x')
        ylabel('y')
        h = legend('Earth','Satellite Trajectory','Pakal','Orbit RF', 'Satellite RF')
        set(h,'Location','northeast')
        title(['t = ',num2str(t_k(i)),' seconds'])
        movieVector(i) = getframe(fig);
        %pause(0.0001)  % wait if you want to see the output in 'real time'
    end
    %% We make a video

    myWriter = VideoWriter(join(['../outputs/',filename]),'MPEG-4'); %create an .mp4 file
    myWriter.FrameRate = 20;
    %Open the VideoWriter object, write the movie, and close the file
    open(myWriter);
    writeVideo(myWriter, movieVector);
    close(myWriter);
    hold off
    %% We clear some variables from the workspace
    clear movieVector;
    clear myWriter;
    clear fig;

end