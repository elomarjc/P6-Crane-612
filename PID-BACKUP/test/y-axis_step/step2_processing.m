%Read CSV files
neg1 = readtable('stepv2/data-1.csv');
neg2 = readtable('stepv2/data-2.csv');
neg3 = readtable('stepv2/data-3.csv');
neg7 = readtable('stepv2/data-7.csv');

pos1 = readtable('stepv2/data1.csv');
pos2 = readtable('stepv2/data2.csv');
pos3 = readtable('stepv2/data3.csv');
pos7 = readtable('stepv2/data7.csv');

%% 

mn = 20;

%Calculate velocity
v_neg1 = diff(movmean(neg1.Var2,mn))./diff(neg1.Var1)*1e6;
v_neg2 = diff(movmean(neg2.Var2,mn))./diff(neg2.Var1)*1e6;
v_neg3 = diff(movmean(neg3.Var2,mn))./diff(neg3.Var1)*1e6;
v_neg7 = diff(movmean(neg7.Var2,mn))./diff(neg7.Var1)*1e6;
v_pos1 = diff(movmean(pos1.Var2,mn))./diff(pos1.Var1)*1e6;
v_pos2 = diff(movmean(pos2.Var2,mn))./diff(pos2.Var1)*1e6;
v_pos3 = diff(movmean(pos3.Var2,mn))./diff(pos3.Var1)*1e6;
v_pos7 = diff(movmean(pos7.Var2,mn))./diff(pos7.Var1)*1e6;

%Makes time vector for velodity
tv_neg1 = neg1.Var1(1:length(neg1.Var1)-1,1);
tv_neg2 = neg2.Var1(1:length(neg2.Var1)-1,1);
tv_neg3 = neg3.Var1(1:length(neg3.Var1)-1,1);
tv_neg7 = neg7.Var1(1:length(neg7.Var1)-1,1);
tv_pos1 = pos1.Var1(1:length(pos1.Var1)-1,1);
tv_pos2 = pos2.Var1(1:length(pos2.Var1)-1,1);
tv_pos3 = pos3.Var1(1:length(pos3.Var1)-1,1);
tv_pos7 = pos7.Var1(1:length(pos7.Var1)-1,1);

%%

%plot model with data
syms s
ke = 2.96e-2;
r = 48e-3;
m = 0.951;
J = 0.5; % Tweaked from 0.14
b = 0.16; % Tweaked from ???

h = (ke*24)/(s*((J)/(2*r)+(m*r)/(2))+(b)/(2*r));

fnc = ilaplace(h*(1/s));
ht = matlabFunction(fnc);

x = linspace(0,10,200);

m_neg1 = ht(x)*-1;
m_neg2 = ht(x)*-2;
m_neg3 = ht(x)*-3;
m_neg7 = ht(x)*-7;

m_pos1 = ht(x)*1;
m_pos2 = ht(x)*2;
m_pos3 = ht(x)*3;
m_pos7 = ht(x)*7;

clf(gcf)
mn = 100;

figure(2)
tiledlayout(2,4);
nexttile
plot(tv_neg1/1e6-43, movmean(v_neg1,mn),x,m_neg1)
xlabel('Time') 
ylabel('Velocity')
title('neg 1')
grid on

nexttile
plot(tv_neg2/1e6-134, movmean(v_neg2,mn),x,m_neg2)
xlabel('Time') 
ylabel('Velocity')
title('neg 2')
grid on

nexttile
plot(tv_neg3/1e6-220.5, movmean(v_neg3,mn),x,m_neg3)
xlabel('Time') 
ylabel('Velocity')
title('neg 3')
grid on

nexttile
plot(tv_neg7/1e6-321.3, movmean(v_neg7,mn),x,m_neg7)
xlabel('Time') 
ylabel('Velocity')
title('neg 7')
grid on

nexttile
plot(tv_pos1/1e6-381, movmean(v_pos1,mn),x,m_pos1)
xlabel('Time') 
ylabel('Velocity')
title('pos 1')
grid on

nexttile
plot(tv_pos2/1e6-439, movmean(v_pos2,mn),x,m_pos2)
xlabel('Time') 
ylabel('Velocity')
title('pos 2')
grid on

nexttile
plot(tv_pos3/1e6-516.3, movmean(v_pos3,mn),x,m_pos3)
xlabel('Time') 
ylabel('Velocity')
title('pos 3')
grid on

nexttile
plot(tv_pos7/1e6-576.95, movmean(v_pos7,mn),x,m_pos7)
xlabel('Time') 
ylabel('Velocity')
title('pos 7')
grid on

% x0=0;
%    y0=0;
%    plotwidth=600;
%    height=300;
%    set(gcf,'position',[x0,y0,plotwidth,height])
%    
% exportgraphics(gcf,'normal1.pdf','ContentType','vector')
%%
s=tf('s');

l = 1;
B = 14;
m = ((0.2)/((0.075)^2))+0.951+0.792+2.943;
b = 2.943/m;
g = 9.82;

G = (s)/(s^3*(1-b*l)+s^2*((B*l)/(m))+s*g+((B*g)/(m)))
figure(1)
margin(G)
figure(2)
step(G)