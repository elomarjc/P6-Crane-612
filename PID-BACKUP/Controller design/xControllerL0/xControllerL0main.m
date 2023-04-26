g = 9.82;
k_ex = 0.609;
J_x = 0.13;
m_t = 0.729;
m_l = 0.951;
r_x = 75*10^(-3);
B_x = 12;
sFrictionX=20;
cFrictionX=18;

m_1 = m_t + m_l + (J_x/(r_x^2));

s = tf('s');

G = (g*k_ex)/(s^2*m_1*g*r_x+B_x*r_x*g);
G2 = k_ex/(s*r_x*(m_1*s+B_x));

figure(1);
rlocus(G)
grid on

D = 50*(1+1/2*s);

figure(2);
rlocus(G2*D)
grid on

%figure(3);
%rlocus(G2);

%figure(4);
%rlocus(G2*D);

%figure(5);
Q = feedback(G2*D,1);
%step(Q)

P = 100;
I = 0;
D = 25;

closedLoop1=sim("xControllerL0sim1.slx");

P = 100;
I = 0;
D = 25;

closedLoop2=sim("xControllerL0sim2.slx");
%  plot(closedLoop.tout, closedLoop.X)

data1 = csvread('P100D25F004.csv');
data2 = csvread('OP100D25F004.csv');

t1 = data1(:,1)*10^-3;
t2 = data2(:,1)*10^-3;

x1 = data1(:,2);
x2 = data2(:,2);

I1 = data1(:,3);
I2 = data2(:,3);

x0=0;
y0=0;
plotwidth=450;
height=350;
yLineVal1 = 0.49;
yLineVal2 = 0.51;
%% 

figure(6)
plot(closedLoop1.tout, closedLoop1.X,t1-t1(1)+1,x1);
grid on
xlim([0.9,2.5])
ylim([-0.1,0.6])
legend('Simulated','Measured');
xlabel('Time [s]');
ylabel('x-position [m]');
yline(yLineVal1);
yline(yLineVal2);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'xl0TrolleyQTS.pdf','ContentType','vector')
%% 

figure(7)
plot(closedLoop2.tout, closedLoop2.X+4,t2-t2(1)+1,x2);
grid on
xlim([0.9,2.5])
ylim([3.4,4.1])
legend('Simulated','Measured');
xlabel('Time [s]');
ylabel('x-position [m]');
yline(3.49);
yline(3.51);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'xl0TrolleySTQ.pdf','ContentType','vector')

P = 50;
I = 0;
D = 25;

closedLoop3=sim("xControllerL0sim1.slx");

%% 

figure(8)
q = plot(closedLoop3.tout, closedLoop3.X,1.2,0.05,'o',2.1,0.45,'o');
q(2).LineWidth = 5;
q(2).Color='#EDB120';
q(3).LineWidth = 5;
q(3).Color='#EDB120';
grid on
xlim([0.9,3.5])
ylim([-0.1,0.6])
%legend('Simulated','Location','southeast');
xlabel('Time [s]');
ylabel('Trolley position [m]');
yline(0.5+0.5*0.02);
yline(0.5-0.5*0.02);
legend('Simulated','','','Location','southeast');
text(1.25,0.04,'10% t = 1.2 s','HorizontalAlignment','left','VerticalAlignment','top')
text(2.15,0.43,'90% t = 2.1 s','HorizontalAlignment','left','VerticalAlignment','top')
text(1,0.55,'Error band \pm 2%','HorizontalAlignment','left','VerticalAlignment','top')
text(2.5,0.3-0.05,'Rise time = 0.9 s','HorizontalAlignment','left','VerticalAlignment','top')
text(2.5,0.25-0.05,'Settling time = 1.3 s','HorizontalAlignment','left','VerticalAlignment','top')
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'xl0TrolleySim1.pdf','ContentType','vector')

%% 

P = 100;
I = 0;
D = 25;

closedLoop4 = sim("xControllerL0sim1.slx");

figure(9)
hold on
q = plot(closedLoop4.tout, closedLoop4.X,t1-t1(1)+1,x1,1.2,0.05,'o',1.7,0.45,'o');
q(3).LineWidth = 5;
q(3).Color='#EDB120';
q(4).LineWidth = 5;
q(4).Color='#EDB120';
grid on
xlim([0.9,3.5])
ylim([-0.1,0.6])
%legend('Simulated','Location','southeast');
xlabel('Time [s]');
ylabel('Trolley position [m]');
yline(0.5+0.5*0.02);
yline(0.5-0.5*0.02);
legend('Simulated','Measured','','','','','Location','southeast');
text(1.25,0.04,'10% t = 1.2 s','HorizontalAlignment','left','VerticalAlignment','top')
text(1.8,0.43,'90% t = 1.7 s','HorizontalAlignment','left','VerticalAlignment','top')
text(1,0.55,'Error band \pm 2%','HorizontalAlignment','left','VerticalAlignment','top')
text(2.5,0.3-0.05,'Rise time = 0.5 s','HorizontalAlignment','left','VerticalAlignment','top')
text(2.5,0.25-0.05,'Settling time = 1 s','HorizontalAlignment','left','VerticalAlignment','top')
hold off

set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'xl0TrolleySimAdjusted.pdf','ContentType','vector')

%% 



figure(10)
q = plot(closedLoop4.tout, closedLoop4.X+4,1.2,3.95,'o',2.1,3.55,'o');
q(2).LineWidth = 5;
q(2).Color='#EDB120';
q(3).LineWidth = 5;
q(3).Color='#EDB120';
grid on
xlim([0.9,3.5])
ylim([3.4,4.1])
legend('Simulated');
xlabel('Time [s]');
ylabel('x-position [m]');
yline(3.485);
yline(3.515);
text(1.25,4,'10% t = 1.2 s','HorizontalAlignment','left','VerticalAlignment','top')
text(2.15,4-0.39,'90% t = 2.1 s','HorizontalAlignment','left','VerticalAlignment','top')
text(1,4-0.475,'Error band \pm 2%','HorizontalAlignment','left','VerticalAlignment','top')
text(2.5,4-0.3+0.05,'Rise time = 0.9 s','HorizontalAlignment','left','VerticalAlignment','top')
text(2.5,4-0.25+0.05,'Settling time = 1.3 s','HorizontalAlignment','left','VerticalAlignment','top')
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'xl0TrolleySim1.pdf','ContentType','vector')


  