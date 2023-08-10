function dx = acc(t,x)
global u;
global v0;
m = 1650;
f0 = 0.1;
f1 = 5;
f2 = 0.25;
dx = zeros(3,1);
dx(1) = x(2);
dx(2) = (1/m)*u(1); %Fr = 0
%dx(2) = (1/m)*(u(1) - f0 - f1*x(2) - f2*x(2).^2);
dx(3) = v0 - x(2);