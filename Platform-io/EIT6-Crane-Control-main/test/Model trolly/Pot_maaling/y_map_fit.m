in = readtable("head pos.csv"); 

mean = (in.HF+in.FH)./2

[p1,S1] = polyfit(mean,in.Pos,1)

x1 = linspace(min(mean),max(mean),5000);
y1 = polyval(p1,x1);

plot(mean,in.Pos,'o',x1,y1)
grid on

text(10,1.3,'f(x)=0.0015x-0.0025')
title('Mean ADC-reading as function of wire length')
set(gcf,'position',[0.4,-0.5,500,300])

xlabel('Mean ADC-reading [.]')
ylabel('Wire length [m]')
legend('Mean ADC-reading','Linear regression','Location','southeast')
%% 


x0=0;
   y0=0;
   plotwidth=500;
   height=300;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'y_map_fit.pdf','ContentType','vector')