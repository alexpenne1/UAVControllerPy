function [t,x,tau] = nlSim(x0,tspan,p)

[t,x] = ode45(@(t, x) NonlinearStatespace(t,x,p), tspan, x0);
tau   = tau_func(x');
end