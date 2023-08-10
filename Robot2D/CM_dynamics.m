function dx = CM_dynamics(t,x)
global u

dx = zeros(4,1);
dx(1) = x(4)*cos(x(3));
dx(2) = x(4)*sin(x(3));
dx(3) = u(1);
dx(4) = u(2);