clc;
x0 = [0 20 100]; % ego vehicle state, x, v, distance with respect to the preceding vehicle
g = 9.81;
m = 1650;
f0 = 0.1;
f1 = 5;
f2 = 0.25;
vd = 30; %24
eps = 10;
gamma = 1;
ca = 0.4;
cd = 0.4;
psc = 1;
result3 = zeros(300,5);
data1 = zeros(3000,8);
c = 10;
set(0,'DefaultTextInterpreter','latex')
warning('off');
global u;
global v0;
v0 = 13.89;  % the speed of the preceding vehicle (constant)


mode = 1;  % control model: 0 - without minimum braking distance, 1-with minimum braking distance.

k = 1;
for i = 1:300
    i
Fr = f0 + f1*x0(2) + f2*x0(2)^2; Fr = 0;
phi0 = -2*(x0(2) - vd)*Fr/m + eps*(x0(2) - vd)^2;
phi1 = 2*(x0(2) - vd)/m;

if mode == 0  % CBF without minimum braking distance
    LfB =v0 - x0(2) +1.8*Fr/m + x0(3) - 1.8*x0(2);   % 1st degree CBF
    LgB = 1.8/m;
else  %CBF with minimum braking distance
    LfB = v0 - x0(2) - ((v0 - x0(2))/(cd*g) - 1.8)*Fr/m + x0(3) - 1.8*x0(2) - 0.5*(v0 - x0(2))^2/(cd*g);
    LgB = -((v0 - x0(2))/(cd*g) - 1.8)/m;
end


A = [phi1 -1;LgB 0;1 0];
b = [-phi0;LfB;ca*m*g];
H = [2/(m^2) 0; 0 2*psc];
F = [-2*Fr/(m^2); 0];
options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
[u,fval,exitflag,output,lambda] = ...
   quadprog(H,F,A,b,[],[],[],[],[],options);
t=[0 0.1];
data1(i,7:8) = [u(1)/m, u(2)];
[tt,xx]=ode45('acc',t,x0);
x0 = [xx(end, 1) xx(end, 2) xx(end, 3)];
result1(i,:) = [0.1*i xx(end,2) u(1) x0(3)-1.8*x0(2)];
end

figure(1)
subplot(3, 1, 1)
plot(result1(:, 1),result1(:, 2),'b-',[0,30],[vd,vd], 'k--',[0,30],[13.89,13.89], 'k--')
xlabel('$t/s$','fontsize',15)
ylabel('speed', 'fontsize',15)
grid on
subplot(3, 1, 2)
plot(result1(:, 1),result1(:, 3)/(m*g), 'b-',[0,30],[ca,ca], 'k--',[0,30],[-cd,-cd], 'k--')
axis([0 30 -0.75 0.75]); 
xlabel('$t/s$','fontsize',15)
ylabel('control', 'fontsize',15)
grid on
subplot(3, 1, 3)
plot(result1(:, 1),result1(:, 4), 'b-',[0,30],[0,0], 'k--')
axis([0 30 -5 75]); 
grid on
xlabel('$t/s$','fontsize',15)
ylabel('safety', 'fontsize',15)


 