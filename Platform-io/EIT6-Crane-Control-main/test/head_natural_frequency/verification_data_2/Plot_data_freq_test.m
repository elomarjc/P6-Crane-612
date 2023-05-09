%% Read data files %% 
% Read data with container
data1_container = readtable("container_10cm.csv");
data2_container = readtable("container_30cm.csv");
data3_container = readtable("container_50cm.csv");
data4_container = readtable("container_70cm.csv");
data5_container = readtable("container_90cm.csv");
data6_container = readtable("container_110cm.csv");

% Read data without container
data1 = readtable("noContainer_10cm.csv");
data2 = readtable("noContainer_30cm.csv");
data3 = readtable("noContainer_50cm.csv");
data4 = readtable("noContainer_70cm.csv");
data5 = readtable("noContainer_90cm.csv");
data6 = readtable("noContainer_110cm.csv");

t_fix = 1000000;

%% Make plots %%
% Plot with container
figure(1)
subplot(3,2,1)
plot(data1_container.Var1/t_fix,data1_container.Var2,data1_container.Var1/t_fix,data1_container.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('With container 10 cm')

subplot(3,2,2)
plot(data2_container.Var1/t_fix,data2_container.Var2,data2_container.Var1/t_fix,data2_container.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('With container 30 cm')

subplot(3,2,3)
plot(data3_container.Var1/t_fix,data3_container.Var2,data3_container.Var1/t_fix,data3_container.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('With container 50 cm')

subplot(3,2,4)
plot(data4_container.Var1/t_fix,data4_container.Var2,data4_container.Var1/t_fix,data4_container.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('With container 70 cm')

subplot(3,2,5)
plot(data5_container.Var1/t_fix,data5_container.Var2,data5_container.Var1/t_fix,data5_container.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('With container 90 cm')

subplot(3,2,6)
plot(data6_container.Var1/t_fix,data6_container.Var2,data6_container.Var1/t_fix,data6_container.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('With container 110 cm')


x0=0;
   y0=0;
   plotwidth=5000;
   height=3000;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'angle_with_container.pdf','ContentType','vector')

% Plot without container
figure(2)
subplot(3,2,1)
plot(data1.Var1/t_fix,data1.Var2,data1.Var1/t_fix,data1.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('Without container 10 cm')

subplot(3,2,2)
plot(data2.Var1/t_fix,data2.Var2,data2.Var1/t_fix,data2.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('Without container 30 cm')

subplot(3,2,3)
plot(data3.Var1/t_fix,data3.Var2,data3.Var1/t_fix,data3.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('Without container 50 cm')

subplot(3,2,4)
plot(data4.Var1/t_fix,data4.Var2,data4.Var1/t_fix,data4.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('Without container 70 cm')

subplot(3,2,5)
plot(data5.Var1/t_fix,data5.Var2,data5.Var1/t_fix,data5.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('Without container 90 cm')

subplot(3,2,6)
plot(data6.Var1/t_fix,data6.Var2,data6.Var1/t_fix,data6.Var3)
%xlim([1 5])
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('unfiltered','filtered')
title('Without container 110 cm')

x0=0;
   y0=0;
   plotwidth=5000;
   height=3000;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'angle_without_container.pdf','ContentType','vector')