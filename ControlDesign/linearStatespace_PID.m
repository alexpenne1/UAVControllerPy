function [dxdt,K] = linearStatespace_PID(A,t,x,B,c,p)
ddx_com     = -c.K_x*x(1) - c.K_dx*x(7);
ddy_com     = -c.K_y*x(2) - c.K_dy*x(8);
roll_com    = (ddx_com*sin(c.YawSet) - ddy_com*cos(c.YawSet))/p.g;
pitch_com   = (ddx_com*cos(c.YawSet) + ddy_com*sin(c.YawSet))/p.g;
Delta_roll  = x(4) - roll_com;  % this should work only if roll setpoint is 0 in initialization (or rather, roll bias)
Delta_pitch = x(5) - pitch_com; % this should work only if pitch setpoint is 0 in initialization (or rather, pitch bias)
lift = - c.K_z*x(3) - c.K_dz*x(9);
tau1 = c.K_roll*Delta_roll + c.K_droll*x(10);
tau2 = c.K_pitch*Delta_pitch + c.K_dpitch*x(11);
tau3 = c.K_yaw*x(6) + c.K_dyaw*x(12);
Tau  = [lift; tau1; tau2; tau3];
GT   = c.Gamma*Tau;
GT(GT<0) = 0;
u    = sqrt(GT);
dxdt = A*x + B*u;
end
