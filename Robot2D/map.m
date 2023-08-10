function  [rt, ps, ptheta] = map(mode, ax, ay, bx, by, cx, cy, dx, dy, Ax, Ay, Bx, By, Cx, Cy) 
global pa pb pc pd pA pB pC pD pobsA pobsB pobsC
if(strcmp(mode,'3D'))  
    figure(1)
    hold on; axis equal
    warning('off','Matlab:hg:EraseModeIgnored');
    R = 6; h = 6; m = 400;
    [x,y,z] = cylinder(R,m,'b');
    x = x + Ax; y = y + Ay; z = h*z;
    mesh(x,y,z)
    r=7; theta=0:pi/100:2*pi;
    x=r*cos(theta); y=r*sin(theta);
    plot3(x+Ax,y+Ay,zeros(1,length(x)),'--')
    x=R*cos(theta); y=R*sin(theta);
    fill3(x+Ax,y+Ay,ones(1,length(x))*h,'b')
    [x,y,z] = cylinder(R,m,'b');
    
    pa = plot3(ax,ay,0,'k+','LineWidth',5);
    pA = text(ax,ay+2,0,'A','FontSize',20);
    pa.MarkerEdgeColor = 'green'; pa.MarkerFaceColor= 'green';
    pA.Color = 'green';
    pb = plot3(bx,by,0,'k+','LineWidth',5);
    pB = text(bx,by+2,0,'B','FontSize',20);
    pc = plot3(cx,cy,0,'k+','LineWidth',5);
    pC = text(cx,cy+2,0,'C','FontSize',20);
    pd = plot3(dx,dy,0,'k+','LineWidth',5);
    pD = text(dx,dy+2,0,'D','FontSize',20);
    
    t=0; y1 = 0;
    pl = plot3(t,y1,3,'go',...                    %vehicle trunk
        'EraseMode','background','LineWidth',10);
    pl.MarkerEdgeColor = 'red';
    pt =  plot3(t,y1,3,'r-',...                  %vehicle trajectory
        'EraseMode','background','LineWidth',1);
    axis([0 50 0 50]);
    set(gcf,'unit','centimeters','position',[8 3 20 15]);
    txt_legend = text(0,0,0,'');
    grid on
    view(15, 40);
else    
    figure(1)    
    hold on;axis equal
    
    r=6; theta=0:pi/100:2*pi;
    x=r*cos(theta); y=r*sin(theta);
    rho=r*sin(theta);
    plot(x+Ax,y+Ay,'k-');
    fill(x+Ax,y+Ay,'c')
    r=7; theta=0:pi/100:2*pi;
    x=r*cos(theta); y=r*sin(theta);
    rho=r*sin(theta);
    pobsA = text(Ax-6,Ay,'Detected','FontSize',20);
    pobsA.Color = 'cyan';
    plot(x+Ax,y+Ay,'b--')
    
    r=5; theta=0:pi/100:2*pi;
    x=r*cos(theta); y=r*sin(theta);
    rho=r*sin(theta);
    plot(x+Bx,y+By,'r-');
    fill(x+Bx,y+By,'c')
    r=6; theta=0:pi/100:2*pi;
    x=r*cos(theta); y=r*sin(theta);
    rho=r*sin(theta);
    pobsB = text(Bx-6,By,'Detected','FontSize',20);
    pobsB.Color = 'cyan';
    plot(x+Bx,y+By,'b--')
    
    r=6; theta=0:pi/100:2*pi;
    x=r*cos(theta); y=r*sin(theta);
    rho=r*sin(theta);
    plot(x+Cx,y+Cy,'r-');
    fill(x+Cx,y+Cy,'c')
    r=7; theta=0:pi/100:2*pi;
    x=r*cos(theta); y=r*sin(theta);
    rho=r*sin(theta);
    pobsC = text(Cx-6,Cy,'Detected','FontSize',20);
    pobsC.Color = 'cyan';
    plot(x+Cx,y+Cy,'b--')
    
    pa = plot(ax,ay,'k+','LineWidth',5);
    pA = text(ax,ay+2,'A','FontSize',20);
    pa.MarkerEdgeColor = 'green'; pa.MarkerFaceColor= 'green';
    pA.Color = 'green';
    pb = plot(bx,by,'k+','LineWidth',5);
    pB = text(bx,by+2,'B','FontSize',20);
    pb.MarkerEdgeColor = 'green'; pb.MarkerFaceColor= 'green';
    pB.Color = 'green';
    pc = plot(cx,cy,'k+','LineWidth',5);
    pC = text(cx,cy+2,'C','FontSize',20);
    pc.MarkerEdgeColor = 'green'; pc.MarkerFaceColor= 'green';
    pC.Color = 'green';
    pd = plot(dx,dy,'k+','LineWidth',5);
    pD = text(dx,dy+2,'D','FontSize',20);
    pd.MarkerEdgeColor = 'green'; pd.MarkerFaceColor= 'green';
    pD.Color = 'green';
    warning('off','Matlab:hg:EraseModeIgnored');
    t=0; y1 = 0; 
    pl = plot(t,y1,'go',...                     %vehicle trunk
        'EraseMode','background','LineWidth',10);
    pl.MarkerEdgeColor = 'red';
    pt =  plot(t,y1,'r-',...                   %vehicle trajectory
        'EraseMode','background','LineWidth',1);
    ps =  plot(t,y1,'r:',...                   %vehicle sense
        'EraseMode','background','LineWidth',2);
    ptheta = plot(t,y1,'k',...                   %vehicle sense
        'EraseMode','background','LineWidth',2);
    
    axis([0 50 0 50]);
    set(gcf,'unit','centimeters','position',[8 3 20 15]);
    txt_legend = text(0,0,'');
end
rt = [pl,pt];
end