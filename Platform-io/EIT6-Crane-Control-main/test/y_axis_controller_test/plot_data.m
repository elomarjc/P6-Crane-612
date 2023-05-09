%No movement of L direction
down_container = readtable("container_down.csv");
down_no_container = readtable("down.csv");
up_container = readtable("container_up.csv");
up_no_container = readtable("up.csv");

subplot(2,2,1)
plot(down_container.Var1/1000,down_container.Var2)
xlabel('Time [s]') 
ylabel('Position [m]')
title('Down with container (stepsize = 0.46)')
grid on
%legend('Measured','Location','southeast')

subplot(2,2,2)
plot(up_container.Var1/1000,up_container.Var2)
xlabel('Time [s]') 
ylabel('Position [m]')
title('Up with container (stepsize = 0.46)')
grid on
%legend('Measured','Location','southeast')

subplot(2,2,3)
plot(down_no_container.Var1/1000,down_no_container.Var2)
xlabel('Time [s]') 
ylabel('Position [m]')
title('Down without container (stepsize = 1.2)')
grid on
%legend('Measured','Location','southeast')

subplot(2,2,4)
plot(up_no_container.Var1/1000,up_no_container.Var2)
xlabel('Time [s]') 
ylabel('Position [m]')
title('Up without container (stepsize = 1.2)')
grid on
%legend('Measured','Location','southeast')


 
x0=0;
   y0=0;
   plotwidth=600;
   height=400;
   set(gcf,'position',[x0,y0,plotwidth,height])
   
exportgraphics(gcf,'plot.pdf','ContentType','vector')