function [rt, dist_dst] = CM_exe(state, dst, unsafe,unsafeB,unsafeC, sense, coe) 
global u once b_c set1 set2 theta_d pobsA pobsB pobsC
x = state(1); y = state(2); theta = state(3); speed = state(4);
px = dst(1); py = dst(2);
radius = 7;
p1 = coe(1);q1 = coe(2);p2 = coe(3);q2 = coe(4);
u_max = 0.2; u_min = -0.2;
a_max = 0.5; a_min = -0.5;
v_max = 2; v_min = 0;
    dist_dst = sqrt((x - px)^2 + (y - py)^2);
    eps = 10;
    psc = 1;
    if(theta < -pi)
        theta = pi;
        state(3) = pi;
    end
    if(theta > pi)
        theta = -pi;
        state(3) = -pi;
    end
    theta_d = atan2(py - y, px - x);
%     if(theta < 0 && theta > -pi && theta_d > 0)
%         theta_d = -1.5*pi;
%     end
    if(theta < 0 && theta > -pi && theta_d >= pi + theta && theta_d <= pi)
        theta_d = -1.5*pi;
    end
    if(theta > 0 && theta < pi && theta_d <= -pi + theta && theta_d >= -pi)
        theta_d = 1.5*pi;
    end
    
    V = (theta - theta_d)^2;
    LfV = 0;
    LgV = 2*(theta - theta_d);
    b_V = -LfV - eps*V;
    
    nx = unsafe(1); ny = unsafe(2);  %%%%%%%%%%%%%%Obs A
    dist = sqrt((x - nx)^2 + (y - ny)^2);
    if(sensing(sense,[nx,ny]))
        pobsA.Color = 'red'; 
        b = dist - radius;
        b_dot = ((x - nx)*speed*cos(theta) + (y - ny)*speed*sin(theta))/dist;
        LgLfb = ((y - ny)*speed*cos(theta) - (x - nx)*speed*sin(theta))/dist;
        LgLfb2 = ((x - nx)*cos(theta) + (y - ny)*sin(theta))/dist;
        Lf2b  = (speed^2*dist^2 - ((x - nx)*speed*cos(theta) + (y - ny)*speed*sin(theta))^2)/dist^3;
        A_safe = -LgLfb;
        A_safeu2 = -LgLfb2;
        A_safe = [A_safe A_safeu2 0 0];
        psi_1 = b_dot + p1*b^q1;
        b_safe = Lf2b + p1*q1*b^(q1 - 1)*b_dot + p2*psi_1^q2; 
        set1 = b; set2 = psi_1;
    else
        pobsA.Color = 'cyan';
        A_safe = [];
        b_safe = [];
    end
    
    
    nx = unsafeB(1); ny = unsafeB(2);  %%%%%%%%%%%%%%Obs B
    dist = sqrt((x - nx)^2 + (y - ny)^2);
    if(sensing(sense,[nx,ny]))
        pobsB.Color = 'red';
        radius = 6;
        b = dist - radius;
        b_dot = ((x - nx)*speed*cos(theta) + (y - ny)*speed*sin(theta))/dist;
        LgLfb = ((y - ny)*speed*cos(theta) - (x - nx)*speed*sin(theta))/dist;
        LgLfb2 = ((x - nx)*cos(theta) + (y - ny)*sin(theta))/dist;
        Lf2b  = (speed^2*dist^2 - ((x - nx)*speed*cos(theta) + (y - ny)*speed*sin(theta))^2)/dist^3;
        A_safeB = -LgLfb;
        A_safeu2B = -LgLfb2;
        A_safeB = [A_safeB A_safeu2B 0 0];
        psi_1 = b_dot + p1*b^q1;
        b_safeB = Lf2b + p1*q1*b^(q1 - 1)*b_dot + p2*psi_1^q2; 
    else
        pobsB.Color = 'cyan';
        A_safeB = [];
        b_safeB = [];
    end
    
    nx = unsafeC(1); ny = unsafeC(2);  %%%%%%%%%%%%%%Obs C
    dist = sqrt((x - nx)^2 + (y - ny)^2);
    if(sensing(sense,[nx,ny]))
        radius = 7;
        pobsC.Color = 'red';
        b = dist - radius;
        b_dot = ((x - nx)*speed*cos(theta) + (y - ny)*speed*sin(theta))/dist;
        LgLfb = ((y - ny)*speed*cos(theta) - (x - nx)*speed*sin(theta))/dist;
        LgLfb2 = ((x - nx)*cos(theta) + (y - ny)*sin(theta))/dist;
        Lf2b  = (speed^2*dist^2 - ((x - nx)*speed*cos(theta) + (y - ny)*speed*sin(theta))^2)/dist^3;
        A_safeC = -LgLfb;
        A_safeu2C = -LgLfb2;
        A_safeC = [A_safeC A_safeu2C 0 0];
        psi_1 = b_dot + p1*b^q1;
        b_safeC = Lf2b + p1*q1*b^(q1 - 1)*b_dot + p2*psi_1^q2; 
    else
        pobsC.Color = 'cyan';
        A_safeC = [];
        b_safeC = [];
    end
    
    vd = dist_dst*v_max/10;
    if(dist_dst < 1)
        vd = 0;
    end
    V_speed = (speed - vd)^2;
    LfV_speed = 0;
    LgV_speed = 2*(speed - vd);
    
    b_vmax = v_max - speed;
    Lgb_vmax = -1;
    A_vmax = -Lgb_vmax;
    b_vmin = speed - v_min;
    Lgb_vmin = 1;
    A_vmin = -Lgb_vmin;
    %angle_control, acc, relax for theta, relax for speed   %0 -1 0 0;
    % -a_min;
    A = [LgV 0 -1 0;A_safe;A_safeB;A_safeC; 1 0 0 0; -1 0 0 0; 0 1 0 0; 0 A_vmax 0 0; 0 A_vmin 0 0;0 LgV_speed 0 -1];
    b = [b_V;b_safe; b_safeB;b_safeC; u_max; -u_min;  a_max; b_vmax; b_vmin;-LfV_speed - eps*V_speed];
    H = [2 0 0 0;0 2 0 0;0 0 2*psc 0;0 0 0 2*psc];
    F = [0; 0; 0; 0];
    options = optimoptions('quadprog',...
        'Algorithm','interior-point-convex','Display','off');
    [u,fval,~,~,~] = ...
       quadprog(H,F,A,b,[],[],[],[],[],options);
    t=[0 0.1];
    if(u(3) > 0.1 && once == 1)
        b_c  = dist - radius;
        once = 0;
    end
    [~,xx]=ode45('CM_dynamics',t,state);
    rt = [xx(end, 1), xx(end, 2), xx(end, 3), xx(end, 4)];
end