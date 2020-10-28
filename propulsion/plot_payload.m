%% PLOT_PAYLOAD
function [movieVector] = plot_payload(dr, dtheta, t, R, step, filename);
%PLOT_ORBIT function to plot CubeSat trajectory and orientation
% 
    %% We reduce the amount of data
    DT = length(t); % time vector length
    t_k = zeros(fix(DT/(step)),1);
    y = zeros(fix(DT/(step)),1);
    x = zeros(fix(DT/(step)),1);
    i=1;
    j=1;
    while i< length(dr)
        
        y(j) = -1000*R*sin(dtheta(i));
        x(j) = 1000*(R*(cos(dtheta(i))-1) - dr(i));
        t_k(j) = t(i);
        i=i+step;
        j=j+1;
    end
    %% We plot the data
    fig = figure;
    axis equal
    for i=1:DT/(step)
        clf
        axis equal
        axis([-25 25 -25 25])
        hold on
        % payload
        patch([-20,20,20,-20],[22.5,22.5,-22.5,-22.5],'g')
        % test mass
        h = rectangle('Position',[y(i)-10 x(i)-10 20 20],'Curvature',[1,1],'FaceColor',[0.3 0.3 0.3]);
        
        % center
        h = rectangle('Position',[-10 -10 20 20],'Curvature',[1,1],'EdgeColor','r');
        
        grid on
        xlabel('y [mm]')
        ylabel('x [mm]')
        title(['t = ',num2str(t_k(i)),' seconds'])
        movieVector(i) = getframe(fig);
        pause(0.0001)  % wait if you want to see the output in 'real time'
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