function xdot = NonlinearStatespace(t,x,p)

my_tau    = tau_func(x);
tau_phi   = my_tau.phi;
tau_theta = my_tau.theta;
tau_psi   = my_tau.psi;
tau_B     = [tau_phi; tau_theta; tau_psi];

Ixx = p.Ixx;
Iyy = p.Iyy;
Izz = p.Izz;

phi    = x(1);
theta  = x(2);
dphi   = x(4);
dtheta = x(5);
dpsi   = x(6);

C11 = 0;
C12 = (Iyy - Izz)*(dtheta*cos(phi)*sin(phi) + dpsi*sin(phi)^2*cos(theta)) + (Izz - Iyy)*(dpsi*cos(phi)^2*cos(theta)) - Ixx*dpsi*cos(theta);
C13 = (Izz - Iyy)*dpsi*cos(phi)*sin(phi)*cos(theta)^2;
C21 = (Izz - Iyy)*(dtheta*cos(phi)*sin(phi) + dpsi*sin(phi)*cos(theta)) + (Iyy - Izz)*(dpsi*cos(phi)^2*cos(theta)) + Ixx*dpsi*cos(theta);
C22 = (Izz - Iyy)*dphi*cos(phi)*sin(phi);
C23 = -Ixx*dpsi*sin(theta)*cos(theta) + Iyy*dpsi*sin(phi)^2*sin(theta)*cos(theta) + Izz*dpsi*cos(phi)^2*sin(theta)*cos(theta);
C31 = (Iyy - Izz)*dpsi*cos(theta)^2*sin(phi)*cos(phi) - Ixx*dtheta*cos(theta);
C32 = (Izz - Iyy)*(dtheta*cos(phi)*sin(phi)*sin(theta) + dphi*sin(phi)^2*cos(theta)) + (Iyy - Izz)*(dphi*cos(phi)^2*cos(theta)) + Ixx*dpsi*sin(theta)*cos(theta) - Iyy*dpsi*sin(phi)^2*sin(theta)*cos(theta) - Izz*dpsi*cos(phi)^2*sin(theta)*cos(theta);
C33 = (Iyy - Izz)*dphi*cos(phi)*sin(phi)*cos(theta)^2 - Iyy*dtheta*sin(phi)^2*cos(theta)*sin(theta) - Izz*dtheta*cos(phi)^2*cos(theta)*sin(theta) + Ixx*dtheta*cos(theta)*sin(theta);
C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];

J11 = Ixx;
J12 = 0;
J13 = -Ixx*sin(theta);
J21 = 0; 
J22 = Iyy*cos(phi)^2 + Izz*sin(phi)^2;
J23 = (Iyy - Izz)*cos(phi)*sin(phi)*cos(theta);
J31 = J13;
J32 = J23;
J33 = Ixx*sin(theta)^2 + Iyy*sin(phi)^2*cos(theta)^2 + Izz*cos(phi)^2*cos(theta)^2;
J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];

ndot = [dphi; dtheta; dpsi];

ndotdot = J\(tau_B - (C*ndot));

xdot = [dphi; dtheta; dpsi; ndotdot];

end