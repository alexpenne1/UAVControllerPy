function [t,x,u,pulsewidth] = LinearSim(tspan,xe,x0,A,B,K,p)
dx0 = x0 - xe; 

[t,dx]= ode45(@(t,dx)linearStatespace(A,t,dx,B,K),tspan,dx0);

dx = dx';
x = zeros(12,length(t));
u = zeros(4,length(t));
duty = zeros(4,length(t));
for i = 1:length(t)
    u(:,i) = -K*dx(:,i);
    x(:,i) = xe + dx(:,i);
    pulsewidth(:,i) = 800/p.vmax.*((p.RbkT*u(:,i).^2 + p.ke*u(:,i))) + 1100;
end

end