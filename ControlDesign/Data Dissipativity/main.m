close all
clear

%% Parameters
p.Ixx = 0.013022;   % moment of inertia around x axis, kg*m^2
p.Iyy = 0.012568;   % moment of inertia around y axis, kg*m^2
p.Izz = 0.021489;   % moment of inertia around z axis, kg*m^2

%{
p.g   = 9.81;       % gravitational constant, m/s^2
p.m   = 0.9445;     % mass of drone, kg
p.l   = .23;        % distance between rotor and center of mass of quadcopter
p.k   = 1.29e-7;    % lift constant, N/rpm
p.b   = 8.21e-9;    % drag constant, N/rpm
p.ke = 0.000656;   % back EMF, V/rpm
p.kT = 0.0108;     % torque constant, Nm/A
p.R    = .17;       % motor resistance, Ohms
p.RbkT = p.R*p.b/p.kT;   
p.vmax = 12.3;      % battery voltage
p.min_PW  = 1100;
p.max_PW  = 1900;
p.min_omega = 0;
p.max_omega = max(roots([p.R*p.b/p.kT, p.ke, -p.vmax]));
%}

%% Simulations
tspan = [0 5];    % time range
x0    = [.1; .1; .1; 0; 0; 0]; % initial conditions
[t,x,tau] = nlSim(x0,tspan,p); % nonlinear simulation

%% Plots
figure
subplot(2,1,1)
plot(t,x(:,1:3))
xlabel('time')
ylabel('radians')
legend('\phi','\theta','\psi')
subplot(2,1,2)
hold on
plot(t,tau.phi)
plot(t,tau.theta)
plot(t,tau.psi)
xlabel('time')
ylabel('Newtons')
legend('\tau_\phi','\tau_\theta','\tau_\psi')
hold off
