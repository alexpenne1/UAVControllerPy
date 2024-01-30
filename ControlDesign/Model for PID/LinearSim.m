function [t,x,u,pulsewidth] = LinearSim_PID(tspan,xe,x0,A,B,c,p)
dx0 = x0 - xe;

[t,dx]= ode45(@(t,dx)linearStatespace_PID(A,t,dx,B,c,p),tspan,dx0);

dx = dx';
x = zeros(12,length(t));
u = zeros(4,length(t));
pulsewidth = zeros(4,length(t));
for i = 1:length(t)
    ddx_com     = -c.K_x*x(1,i) - c.K_dx*x(7,i);
    ddy_com     = -c.K_y*x(2,i) - c.K_dy*x(8,i);
    roll_com    = (ddx_com*sin(c.YawSet) - ddy_com*cos(c.YawSet))/p.g;
    pitch_com   = (ddx_com*cos(c.YawSet) + ddy_com*sin(c.YawSet))/p.g;
    Delta_roll  = x(4,i) - roll_com;  % this should work only if roll setpoint is 0 in initialization (or rather, roll bias)
    Delta_pitch = x(5,i) - pitch_com; % this should work only if pitch setpoint is 0 in initialization (or rather, pitch bias)
    lift = p.m*p.g - c.K_z*x(3,i) - c.K_dz*x(9,i); % might be a discrepancy here with the linearized system
    tau1 = c.K_roll*Delta_roll + c.K_droll*x(10,i);
    tau2 = c.K_pitch*Delta_pitch + c.K_dpitch*x(11,i);
    tau3 = c.K_yaw*x(6,i) + c.K_dyaw*x(12,i);
    Tau  = [lift; tau1; tau2; tau3];
    GT   = c.Gamma*Tau;
    GT(GT<0) = 0;
    u(:,i)    = sqrt(GT);
    x(:,i) = xe + dx(:,i);
    pulsewidth(:,i) = 800/p.vmax.*((p.RbkT*u(:,i).^2 + p.ke*u(:,i))) + 1100;
end

end