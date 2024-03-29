function [t,x,u,pulsewidth] = nlSim(x0,tspan,K,ue,xe,p)
tic
[t,x] = ode45(@(t, x) NonlinearStatespace(t,x,K,ue,xe,p), tspan, x0);
myT = toc
u    = zeros(4,length(t));
pulsewidth = zeros(4,length(t));
for i = 1:length(t)
    u(:,i)    = ue'-K*(x(i,:)'-xe');
    u(u>p.max_omega) = p.max_omega;
    u(u<p.min_omega) = p.min_omega;
    pulsewidth(:,i) = 800/p.vmax.*((p.RbkT*u(:,i).^2 + p.ke*u(:,i))) + 1100;

end

end