clf

% Define transfer function variable
s = tf('s');

% Constants
ke = 0.71;
rl = 0.05;
bl = 20;
j = 0.2;
ml = 3.89;
Ml = ml+((4*j)/rl^2);

G = (2*ke*Ml^(-1))/(rl*(s^2+s*Ml^(-1)*bl))

figure(1)
%rlocus(G*((1+(1/2)*s)))
rlocus(G)
%zgrid(0.71,1)
%xlim([-2.5 0.1])
%ylim([-1.2 1.2])
%xlim([-4.5 0.2])
%ylim([-2.5 2.5])
%x = [0.54,0.54];
%y = [0.6,0.5];
%x1 = [0.55,0.55];
%y1 = [0.4,0.49];
%a = annotation('textarrow',x,y,'String','Gain = 150 ');
%b = annotation('textarrow',x1,y1,'String','Gain = inf ');
axis equal
%text(-0.99,0.755,'\zeta =')
%text(-0.99,-0.755,'\zeta =')
%text(-1.24,0.03,'\omega_n =')

%text(-0.25,0.3,'45.23^{\circ}', 'FontSize', 8)

x0=0;
   y0=0;
   plotwidth=600;
   height=400;
   set(gcf,'position',[x0,y0,plotwidth,height])
   
exportgraphics(gcf,'plant_rlocus.pdf','ContentType','vector')

figure(2)
rlocus(G*((1+(1/2)*s)))

x = [0.54,0.54];
y = [0.6,0.5];
x1 = [0.55,0.55];
y1 = [0.4,0.49];

xlim([-4.5 0.1])
axis equal
hold on
p = plot(-3.32,1.46,'^',-3.32,-1.46,'^')
p(1).Color='k';
p(2).Color='k';
grid on


x0=0;
   y0=0;
   plotwidth=600;
   height=400;
   set(gcf,'position',[x0,y0,plotwidth,height])
   
exportgraphics(gcf,'plant_rlocus_zero_22.pdf','ContentType','vector')

