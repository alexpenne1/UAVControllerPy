function [t,x,u,pulsewidth] = nlSim(x0,tspan,xe,c,p)

[t,x] = ode45(@(t, x) NonlinearStatespace(t,x,xe,c,p), tspan, x0);

u    = zeros(4,length(t));
pulsewidth = zeros(4,length(t));
for i = 1:length(t)
    dx = x(i,:)'-xe';
    ddx_com     = -c.K_x*dx(1) - c.K_dx*dx(7);
    ddy_com     = -c.K_y*dx(2) - c.K_dy*dx(8);
    roll_com    = (ddx_com*sin(c.YawSet) - ddy_com*cos(c.YawSet))/p.g;
    pitch_com   = (ddx_com*cos(c.YawSet) + ddy_com*sin(c.YawSet))/p.g;
    Delta_roll  = dx(4) - roll_com;  % this should work only if roll setpoint is 0 in initialization (or rather, roll bias)
    Delta_pitch = dx(5) - pitch_com; % this should work only if pitch setpoint is 0 in initialization (or rather, pitch bias)
    lift = p.m*p.g - c.K_z*dx(3) - c.K_dz*dx(9); % might be a discrepancy here with the linearized system
    tau1 = c.K_roll*Delta_roll + c.K_droll*dx(10);
    tau2 = c.K_pitch*Delta_pitch + c.K_dpitch*dx(11);
    tau3 = c.K_yaw*dx(6) + c.K_dyaw*dx(12);
    Tau  = [lift; tau1; tau2; tau3];
    GT   = c.Gamma*Tau;
    GT(GT<0) = 0;
    u(:,i)   = sqrt(GT);
    u(u>p.max_omega) = p.max_omega;
    u(u<p.min_omega) = p.min_omega;
    pulsewidth(:,i) = 800/p.vmax.*((p.RbkT*u(:,i).^2 + p.ke*u(:,i))) + 1100;
end

end