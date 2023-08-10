 clc; clear
set(0,'DefaultTextInterpreter', 'latex')
mode = '2D'; global va vb first1 first2 firstB firstC firstD %pa pb pc pd pA pB pC pD
global u once b_c set1 set2
first1 = 1; first2 = 1;firstB = 1; firstC = 1; firstD = 1; va = 15; vb = 40;
sense.x = 0; sense.y = 0; set1 = 0; set2 = 0;

once = 1; b_c = 0;
te = 2500; 
ax = 38; ay = 40; bx = 39; by = 35; cx = 30; cy = 15;  dx = 20; dy = 28; %target location 10 20
Ax = 32; Ay = 25; Bx = 28; By = 35; Cx = 30; Cy = 40;%Obstacle location
x1 = [12 0 atan2(35,bx - 12) 1]; %

[rt, ps, ptheta] = map(mode, ax, ay, bx, by, cx, cy, dx, dy, Ax, Ay, Bx, By, Cx, Cy);
pl = rt(1); pt = rt(2); 

size = 2500; j = 1; %700
px_history = zeros(1,size); py_history = zeros(1,size); 
%   pause(15);
%   start = 1
%   pause(2);
u_history = zeros(te,5);


coe = [2.5, 0, 0, 0.28, 0]; %0, 0, 0.3, 0, 0.2 %or 2.5, 0.1, 0, 0.28, 0
coe = [0.7426    1.9148    1.9745    0.7024];
coe = [0.7535    1.0046    0.6664    1.0267];
for i = 1:te
    if(i < 870)
        x1 = CM_exe(x1,[bx,by],[Ax,Ay],[Bx,By],[Cx,Cy],sense,coe);
    else
        if(i < 1200)
            x1 = CM_exe(x1,[cx,cy],[Ax,Ay],[Bx,By],[Cx,Cy],sense,coe);
        else
            if(i < 1865) %1880
                x1 = CM_exe(x1,[ax,ay],[Ax,Ay],[Bx,By],[Cx,Cy],sense,coe);
            else
                x1 = CM_exe(x1,[dx,dy],[Ax,Ay],[Bx,By],[Cx,Cy],sense,coe);
            end
        end
    end
    u_history(i,:) = [0.1*i, u(1), u(2) set1 set2];
    pos = [x1(1) x1(2)];
    px_history(j) = pos(1); py_history(j) = pos(2);
    
    [j, sense] = CM_draw(mode, pl, pt,ps, ptheta, pos, x1(3), px_history, py_history, i, j, size);
    if(b_c ~= 0)
        b_c
        b_c = 0;
    end
end
figure(2)
plot(u_history(:,1),u_history(:,2))
figure(3)
plot(u_history(:,1),u_history(:,3))
figure(4)
plot(u_history(:,1),u_history(:,4))
figure(5)
plot(u_history(:,1),u_history(:,5))


% for i = 1:te
%     if(i < 700)
%         x1 = CM_exe(x1,[bx,by],[Ax,Ay]);
%     else
%         if(i < 1200)
%             x1 = CM_exe(x1,[dx,dy],[Ax,Ay]);
%         else
%             if(i < 1600)
%                 x1 = CM_exe(x1,[ax,ay],[Ax,Ay]);
%             else
%                 x1 = CM_exe(x1,[cx,cy],[Ax,Ay]);
%             end
%         end
%     end
%     u_history(i,:) = [0.1*i, u(1), u(2)];
%     pos = [x1(1) x1(2)];
%     px_history(j) = pos(1); py_history(j) = pos(2);
%     
%     j = CM_draw(mode, pl, pt, pos, px_history, py_history, i, j, size);
% end


