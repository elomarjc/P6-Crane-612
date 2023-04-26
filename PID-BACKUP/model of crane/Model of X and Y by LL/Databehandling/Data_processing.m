%Load measurements without container
Down_3 = readtable('3amp2sek.csv');
Down_5 = readtable('5amp2sek.csv');
Down_7 = readtable('7amp2sek.csv');

Up_3 = readtable('-3amp2sek.csv');
Up_5 = readtable('-5amp2sek.csv');
Up_7 = readtable('-7amp2sek.csv');

%Load measurements with container
Down_3_container = readtable('3amp2sekCon.csv');
Down_5_container = readtable('5amp2sekCon.csv');
Down_7_container = readtable('5amp2sekCon.csv'); %Husk at den her ikke passer!

Up_4_container = readtable('-4amp2sekCon.csv');
Up_5_container = readtable('-5amp2sekCon.csv');
Up_7_container = readtable('-7amp2sekCon.csv');

%%
%Calculate velocity
mn1 = 1;

%Down
v_down_3 = diff(movmean(Down_3.y,mn1))./diff(Down_3.t);
v_down_5 = diff(movmean(Down_5.y,mn1))./diff(Down_5.t);
v_down_7 = diff(movmean(Down_7.y,mn1))./diff(Down_7.t);

%Up
v_up_3 = diff(movmean(Up_3.y,mn1))./diff(Up_3.t);
v_up_5 = diff(movmean(Up_5.y,mn1))./diff(Up_5.t);
v_up_7 = diff(movmean(Up_7.y,mn1))./diff(Up_7.t);

%Container down
v_down_3_container = diff(movmean(Down_3_container.y,mn1))./diff(Down_3_container.t);
v_down_5_container = diff(movmean(Down_5_container.y,mn1))./diff(Down_5_container.t);
v_down_7_container = diff(movmean(Down_7_container.y,mn1))./diff(Down_7_container.t);

%Container up
v_up_4_container = diff(movmean(Up_4_container.y,mn1))./diff(Up_4_container.t);
v_up_5_container = diff(movmean(Up_5_container.y,mn1))./diff(Up_5_container.t);
v_up_7_container = diff(movmean(Up_7_container.y,mn1))./diff(Up_7_container.t);

%Makes time vector for velodity
tv_down_3 = Down_3.t(1:length(Down_3.t)-1,1);
tv_down_5 = Down_5.t(1:length(Down_5.t)-1,1);
tv_down_7 = Down_7.t(1:length(Down_7.t)-1,1);

tv_up_3 = Up_3.t(1:length(Up_3.t)-1,1);
tv_up_5 = Up_5.t(1:length(Up_5.t)-1,1);
tv_up_7 = Up_7.t(1:length(Up_7.t)-1,1);

tv_down_3_container = Down_3_container.t(1:length(Down_3_container.t)-1,1);
tv_down_5_container = Down_5_container.t(1:length(Down_5_container.t)-1,1);
tv_down_7_container = Down_7_container.t(1:length(Down_7_container.t)-1,1);

tv_up_4_container = Up_4_container.t(1:length(Up_4_container.t)-1,1);
tv_up_5_container = Up_5_container.t(1:length(Up_5_container.t)-1,1);
tv_up_7_container = Up_7_container.t(1:length(Up_7_container.t)-1,1);

%%
%Plot raw data position
figure(1)
tiledlayout(4,3);
nexttile
plot(Down_3.t, Down_3.y)
xlabel('Time') 
ylabel('Position')
title('Down 3')
nexttile
plot(Down_5.t, Down_5.y)
xlabel('Time') 
ylabel('Position')
title('Down 5')
nexttile
plot(Down_7.t, Down_7.y)
xlabel('Time') 
ylabel('Position')
title('Down 7')

nexttile
plot(Down_3_container.t, Down_3_container.y)
xlabel('Time') 
ylabel('Position')
title('Down container 3')
nexttile
plot(Down_5_container.t, Down_5_container.y)
xlabel('Time') 
ylabel('Position')
title('Down container 5')
nexttile
plot(Down_7_container.t, Down_7_container.y)
xlabel('Time') 
ylabel('Position')
title('Down container 7')

nexttile
plot(Up_3.t, Up_3.y)
xlabel('Time') 
ylabel('Position')
title('Up 3')
nexttile
plot(Up_5.t, Up_5.y)
xlabel('Time') 
ylabel('Position')
title('Up 5')
nexttile
plot(Up_7.t, Up_7.y)
xlabel('Time') 
ylabel('Position')
title('Up 7')

nexttile
plot(Up_4_container.t, Up_4_container.y)
xlabel('Time') 
ylabel('Position')
title('Up container 4')
nexttile
plot(Up_5_container.t, Up_5_container.y)
xlabel('Time') 
ylabel('Position')
title('Up container 5')
nexttile
plot(Up_7_container.t, Up_7_container.y)
xlabel('Time') 
ylabel('Position')
title('Up container 7')
%%
%Plot raw data velocity
figure(2)
tiledlayout(4,3);
nexttile
plot(tv_down_3, v_down_3)
xlabel('Time') 
ylabel('Velocity')
title('Down 3')
nexttile
plot(tv_down_5, v_down_5)
xlabel('Time') 
ylabel('Velocity')
title('Down 5')
nexttile
plot(tv_down_7, v_down_7)
xlabel('Time') 
ylabel('Velocity')
title('Down 7')

nexttile
plot(tv_down_3_container, v_down_3_container)
xlabel('Time') 
ylabel('Velocity')
title('Down container 3')
nexttile
plot(tv_down_5_container, v_down_5_container)
xlabel('Time') 
ylabel('Velocity')
title('Down container 5')
nexttile
plot(tv_down_7_container, v_down_7_container)
xlabel('Time') 
ylabel('Position')
title('Down container 7')

nexttile
plot(tv_up_3, v_up_3)
xlabel('Time') 
ylabel('Velocity')
title('Up 3')
nexttile
plot(tv_up_3, v_up_3)
xlabel('Time') 
ylabel('Velocity')
title('Up 5')
nexttile
plot(tv_up_3, v_up_3)
xlabel('Time') 
ylabel('Velocity')
title('Up 7')

nexttile
plot(tv_up_4_container, v_up_4_container)
xlabel('Time') 
ylabel('Velocity')
title('Up container 4')
nexttile
plot(tv_up_5_container, v_up_5_container)
xlabel('Time') 
ylabel('Velocity')
title('Up container 5')
nexttile
plot(tv_up_7_container, v_up_7_container)
xlabel('Time') 
ylabel('Velocity')
title('Up container 7')


%%
%Plot with model. Ikke rigtig model, linear peter/jakob model
syms s t

ke = 2.96e-2;
r = 48e-3;
m = 0.951;
i3 = 0.14;
b = 7;
h = (ke*24)/(s*(i3/(r*2)+(m*r)/2)+(b)/(r*2));

r = ilaplace(h*(1/s));
 
ht = matlabFunction(r);

x = linspace(0,10,200);
y_3 = ht(x)*3;
y_5 = ht(x)*5;
y_7 = ht(x)*7;
y_3_neg = ht(x)*-3;
y_5_neg = ht(x)*-5;
y_7_neg = ht(x)*-7;
