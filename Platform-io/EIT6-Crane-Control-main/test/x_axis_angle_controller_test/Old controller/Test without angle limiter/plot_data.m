%No movement of L direction
step_1 = readtable("step_1.csv");
step_2 = readtable("step_2.csv");
step_3 = readtable("step_3.csv");
step_4 = readtable("step_4.csv");

subplot(2,2,1)
plot(step_1.Var1/1000,step_1.Var4)
xlabel('Time [s]') 
ylabel('Angle [deg]')
title('Step 1')
grid on
%legend('Measured','Location','southeast')

subplot(2,2,2)
plot(step_2.Var1/1000,step_2.Var4)
xlabel('Time [s]') 
ylabel('Angle [deg]')
title('Step 2')
grid on
%legend('Measured','Location','southeast')

subplot(2,2,3)
plot(step_3.Var1/1000,step_3.Var4)
xlabel('Time [s]') 
ylabel('Angle [deg]')
title('Step 3')
grid on
%legend('Measured','Location','southeast')

subplot(2,2,4)
plot(step_4.Var1/1000,step_4.Var4)
xlabel('Time [s]') 
ylabel('Angle [deg]')
title('Step 4')
grid on
%legend('Measured','Location','southeast')


 
x0=0;
   y0=0;
   plotwidth=600;
   height=400;
   set(gcf,'position',[x0,y0,plotwidth,height])
   
exportgraphics(gcf,'plot.pdf','ContentType','vector')