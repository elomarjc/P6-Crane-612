%% Read data files %% 
% Read data before notch
before1 = readtable("before_notch/noContainer_10cm.csv");
before2 = readtable("before_notch/noContainer_30cm.csv");
before3 = readtable("before_notch/noContainer_50cm.csv");
before4 = readtable("before_notch/noContainer_70cm.csv");
before5 = readtable("before_notch/noContainer_90cm.csv");
before6 = readtable("before_notch/noContainer_110cm.csv");

% Read data after notch
after1 = readtable("after_notch/noContainer_10cm.csv");
after2 = readtable("after_notch/noContainer_30cm.csv");
after3 = readtable("after_notch/noContainer_50cm.csv");
after4 = readtable("after_notch/noContainer_70cm.csv");
after5 = readtable("after_notch/noContainer_90cm.csv");
after6 = readtable("after_notch/noContainer_110cm.csv");

t_fix = 1000000;

%% Make plots %%
% Plot without container
figure(2)
subplot(3,2,1)
plot((before1.Var1/t_fix)-871,before1.Var2,((after1.Var1/t_fix)+302)-871,after1.Var2)
xlim([0 13])
xlabel('Time [s]')
ylabel('Angle [deg]')
%legend('Raw angle','Notch filter')
title('10 cm')

subplot(3,2,2)
plot((before2.Var1/t_fix)-805,before2.Var2,((after2.Var1/t_fix)+453.8)-805,after2.Var2)
xlim([0 13])
xlabel('Time [s]')
ylabel('Angle [deg]')
%legend('Raw angle','Notch filter')
title('30 cm')

subplot(3,2,3)
plot((before3.Var1/t_fix)-750,before3.Var2,((after3.Var1/t_fix)+456)-750,after3.Var2)
xlim([0 13])
xlabel('Time [s]')
ylabel('Angle [deg]')
%legend('Raw angle','Notch filter')
title('50 cm')

subplot(3,2,4)
plot((before4.Var1/t_fix)-678,before4.Var2,((after4.Var1/t_fix)+502.5)-678,after4.Var2)
xlim([0 13])
xlabel('Time [s]')
ylabel('Angle [deg]')
%legend('Raw angle','Notch filter')
title('70 cm')

subplot(3,2,5)
plot((before5.Var1/t_fix)-594,before5.Var2,((after5.Var1/t_fix)+469)-594,after5.Var2)
xlim([0 13])
xlabel('Time [s]')
ylabel('Angle [deg]')
%legend('Raw angle','Notch filter')
title('90 cm')

subplot(3,2,6)
plot((before6.Var1/t_fix)-534,before6.Var2,((after6.Var1/t_fix)+489)-534,after6.Var2)
xlim([0 13])
xlabel('Time [s]')
ylabel('Angle [deg]')
%legend('Raw angle','Notch filter')
title('110 cm')

x0=0;
   y0=0;
   plotwidth=600;
   height=500;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'comparason_notch_filter.pdf','ContentType','vector')