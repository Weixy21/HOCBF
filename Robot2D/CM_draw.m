function [j, sense] = CM_draw(mode, pl, pt, ps, ptheta, pos, heading, px_history, py_history, i, j, size) 
global va vb theta_d
j = j + 1; 
if(j > size)
    j = 1;
end
if i <= size
    p_history = [px_history(1:i); py_history(1:i)];
else
    if j < 2
        p_history = [px_history; py_history];
    else
        p_history = [[px_history(j:end),px_history(1:j-1)];[py_history(j:end),py_history(1:j-1)]];
    end
end
if (strcmp(mode,'3D'))
        pz = 0;
        px = pos(1); 
        py = pos(2);
        lll = length(p_history(1,:));
        ppp = zeros(1,lll);
        set(pl,'XData',px,'YData',py,'ZData',pz)
        set(pt,'XData',p_history(1,:),'YData',p_history(2,:),'ZData',ppp)
        if(i >= 100 && i < 199)
            va = -75/99*i + 15 + 100*75/99;
            view(va, vb);
        end
        if(i >= 299 && i < 399)
            va =0.9*i -60 - 0.9*299;
            view(va, vb);
        end
        if(i >= 749 && i < 799)
            va =1.2*i + 30 - 1.2*749;
            view(va, vb);
        end
        if(i >= 899 && i < 959)
            vb =0.8*i + 40 - 0.8*899;
            view(va, vb);
        end
        drawnow
else
        px = pos(1); 
        py = pos(2);
        set(pl,'XData',px,'YData',py)
        set(pt,'XData',p_history(1,:),'YData',p_history(2,:))
        r=7; theta=heading - pi/3:pi/20:heading + pi/3;
        len = length(theta);
        noi = rand(1,len)-0.5;
        x=r*cos(theta) + noi; y=r*sin(theta) + noi;
        set(ps,'XData',[px, px + x, px],'YData',[py, py + y, py])
        sense.x = px + x; sense.y = py + y;
        dd = 2;
        xd = cos(theta_d)*dd; yd = sin(theta_d)*dd;
        xarrow = px + 0.6*xd; yarrow = py + 0.6*yd; 
        theta1 = theta_d + pi/2; theta2 = theta_d - pi/2;
        xa1 = xarrow + 0.4*cos(theta1); ya1 = yarrow + 0.4*sin(theta1);
        xa2 = xarrow + 0.4*cos(theta2); ya2 = yarrow + 0.4*sin(theta2);
        set(ptheta,'XData',[px, px + xd, xa1, px + xd, xa2],'YData',[py, py + yd, ya1, py + yd, ya2])
        drawnow    
end
end