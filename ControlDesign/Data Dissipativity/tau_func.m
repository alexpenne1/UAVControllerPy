function tau = tau_func(x)

phi    = x(1,:);
theta  = x(2,:);
psi    = x(3,:);
dphi   = x(4,:);
dtheta = x(5,:);
dpsi   = x(6,:);

K.phi    = 1;
K.dphi   = .1;
K.theta  = 1;
K.dtheta = .1;
K.psi    = 1;
K.dpsi   = .1;

tau.phi   = - K.phi.*phi   - K.dphi.*dphi;
tau.theta = - K.theta.*theta - K.dtheta.*dtheta;
tau.psi   = - K.psi.*psi   - K.dpsi.*dpsi;

end