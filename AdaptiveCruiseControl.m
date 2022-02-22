%% Adaptive Cruise Control System
% Define the sample time, |Ts|, and simulation duration, |T|, in seconds 
close all;clear;
Ts = 0.05;
T = 50;
%Define human driver's freq & amplitude of acccel

HA=2;
HF=0.4;     %Radians

%%
% Specify the linear model for ego car.
G_ego = tf(1,[0.5,1,0]);

%%
vref=21;L_RDis=7;L_Vel=26.8;R_RDis=20;R_Vel=30;
% Specify the initial position and velocity for the two vehicles.
x0_lead = 80;   % initial position for lead car (m)
v0_lead = 30;   % initial velocity for lead car (m/s)

x0_ego = 20;   % initial position for ego car (m)
v0_ego = 30;   % initial velocity for ego car (m/s)

x0_rear = 14;   % initial position for rear car (m)
v0_rear = 30;   % initial velocity for rear car (m/s)

%%
% Specify the driver-set velocity in m/s.
v_set = 100;

%% Simulation Analysis
% Run the simulation.
sim('ACCsystem.slx')

%%
figure(1);subplot(211);
yyaxis left
plot(Lv);hold on
plot(Ev);
plot(Rv);

grid on;

title(['Vehicle Velocities: Vref= ',num2str(v_set) ])
xlabel('Time')
ylabel('Velocity (m/s)')

yyaxis right
plot(Ea); hold off
ylabel('Ego Acceleration')
legend('Lead','Ego','Rear','Ego-Accerlation')

%% Plot Relative Distances
subplot(212)
plot(Ep.Time,Lp.Data-Ep.Data);hold on
yline(0);
plot(Ep.Time,Rp.Data-Ep.Data);
hold off
title('Relative Lead/Rear Distances from Ego')
xlabel('Time(s)')
ylabel('Relative distance (m)')
legend('Lead Vehicle Offset','Ego Vehicle','Rear Vehicle offset')
%% Circular Plot
%X,V,A
X=[Lp.Data';Ep.Data';Rp.Data'];
V=[Lv.Data';Ev.Data';Rv.Data'];
A=[La.Data';Ea.Data';Ra.Data'];

N=3;        %Number of Players
Nh = 2;     %Number of Humans
iH = [1,3]; %Position of Humans
R = 1000;      %Radius of Circle
eV=0.05;
K=T/Ts;

%Video initialization

myVideo = VideoWriter('ACC_Demo'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

dW = N*pi/(eV*N);
XC = @( x ) R*sin( x/dW );
YC = @( x ) R*cos( x/dW );

COLORS = { [0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.9290 0.6940 0.1250],...
    [0.4940 0.1840 0.5560], [0.4660 0.6740 0.1880], [0.3010 0.7450 0.9330],...
    [0.6350 0.0780 0.1840] };

NAMES = {'Trajectories', 'Accelerations', 'Distance between Agents', 'Velocities'};
XLAB = {'','Time(s)','Time(s)','Time(s)'};
YLAB = {'','Acceleration (m/s^2)','Distance (m)','Velocity (m/s)'};

figure('Name','Animation','NumberTitle','off','WindowState','maximized')
for k = 1:4
    ax(k) = subplot(2,2,k);
    title( NAMES{k} );
    xlabel( XLAB{k} );
    ylabel( YLAB{k} );
    hold on
    grid on
    grid minor
end

for i = 1:N
    idx = mod(i,7);
    if idx == 0, idx = 7; end
    cTemp = COLORS{ idx };

    if sum( iH == i ) > 0
        SC(i) = scatter( XC( X(i,1) ), YC( X(i,1) ), 100, cTemp, ...
            'filled', 'd', 'Parent', ax(1) );
    else
        SC(i) = scatter( XC( X(i,1) ), YC( X(i,1) ), 100, cTemp, ...
            'filled', 'Parent', ax(1) );
    end

    H(i) = animatedline( ax(1), XC( X(i,1) ), YC( X(i,1) ),'Color', ...
        cTemp ,'LineWidth',3, 'MaximumNumPoints',30, 'Parent', ax(1) );

    Hx(i) = animatedline( ax(3), 0, X(i,1) - mean( X(:,1) ),'Color', ...
        cTemp ,'LineWidth',3, 'Parent', ax(3) );

    Hv(i) = animatedline( ax(4), 0, V(i,1),'Color', ...
        cTemp ,'LineWidth',1, 'Parent', ax(4) );

    Ha(i) = animatedline( ax(2), 0, A(i,1),'Color', ...
        cTemp ,'LineWidth',1, 'Parent', ax(2) );
end

subplot( ax(1) )
xlim( [-R-1,R+1] )
ylim( [-R-1,R+1] )
axis square
xticks([])
yticks([])

subplot( ax(2) ), xlim( [0, K*Ts] )
subplot( ax(3) ), xlim( [0, K*Ts] )
subplot( ax(4) ), xlim( [0, K*Ts] )



for k = 2:3:K
    for i = 1:N
        addpoints( H(i), XC( X(i,k) ), YC( X(i,k) ) );

        addpoints( Hx(i), (k-1)*Ts, X(i,k) - mean( X(:,k) ) );
        addpoints( Hv(i), (k-1)*Ts, V(i,k) );
        addpoints( Ha(i), (k-1)*Ts, A(i,k) );

        SC(i).XData = XC( X(i,k) );
        SC(i).YData = YC( X(i,k) );
    end
    drawnow
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)