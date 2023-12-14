% PlotDroneData
close all
clear
clc
savetime = clock;
Data = load('data.csv');
fig_filename = 'ExperimentPlots/StationaryTest__';
fig_filetime = ['__',num2str(savetime(1)),'_',num2str(savetime(2)),'_',num2str(savetime(3)),'_',num2str(savetime(4)),'_',num2str(savetime(5))];

% Separate data
t      = Data(:,1);
x      = Data(:,2);
y      = Data(:,3);
z      = Data(:,4);
roll   = Data(:,5);
pitch  = Data(:,6);
yaw    = Data(:,7);
dx     = Data(:,8);
dy     = Data(:,9);
dz     = Data(:,10);
droll  = Data(:,11);
dpitch = Data(:,12);
dyaw   = Data(:,13);
u1     = Data(:,14);
u2     = Data(:,15);
u3     = Data(:,16);
u4     = Data(:,17);

% Calculate bandwidth
Hz = ((t(end)-t(1))/length(t))^-1;

% Convert to relative time
t_rel = t-t(1);

% Plot
fig1 = figure(1);
title('Position')
subplot(2,1,1)
hold on
plot(t_rel,x)
plot(t_rel,y)
plot(t_rel,z)
legend('x','y','z')
xlabel('time (s)')
ylabel('position (m)')
set(gca,'FontSize',18)
hold off
subplot(2,1,2)
hold on
plot(t_rel,dx)
plot(t_rel,dy)
plot(t_rel,dz)
legend('dx','dy','dz')
xlabel('time (s)')
ylabel('velocity (m/s)')
set(gca,'FontSize',18)
hold off

fig2 = figure(2);
title('Attitude')
subplot(2,1,1)
hold on
plot(t_rel,roll)
plot(t_rel,pitch)
plot(t_rel,yaw)
legend('roll','pitch','yaw')
xlabel('time (s)')
ylabel('attitude (rad)')
set(gca,'FontSize',18)
hold off
subplot(2,1,2)
hold on
plot(t_rel,droll)
plot(t_rel,dpitch)
plot(t_rel,dyaw)
legend('droll','dpitch','dyaw')
xlabel('time (s)')
ylabel('angular velocity (rad/s)')
set(gca,'FontSize',18)
hold off

fig3 = figure(3);
title('Controls')
hold on
plot(t_rel,u1)
plot(t_rel,u2)
plot(t_rel,u3)
plot(t_rel,u4)
legend('u1','u2','u3','u4')
xlabel('time (s)')
ylabel('pulsewidth (micro s)')
set(gca,'FontSize',18)
hold off

saveas(fig1,[fig_filename,'fig1',fig_filetime])
saveas(fig2,[fig_filename,'fig2',fig_filetime])
saveas(fig3,[fig_filename,'fig3',fig_filetime])




