data1QTS = csvread('1QTS.csv');
data8QTS = csvread('8QTS.csv');
data10QTS = csvread('10QTS.csv');
data13QTS = csvread('13QTS.csv');

data10STQ = csvread('STQ10.csv');
data11STQ = csvread('STQ11.csv');
data12STQ = csvread('STQ12.csv');
data14STQ = csvread('STQ14.csv');

t1QTS = data1QTS(:,1);
t8QTS = data8QTS(:,1);
t10QTS = data10QTS(:,1);
t13QTS = data13QTS(:,1);

t10STQ = data10STQ(:,1);
t11STQ = data11STQ(:,1);
t12STQ = data12STQ(:,1);
t14STQ = data14STQ(:,1);

x1QTS1 = data1QTS(:,2);
x8QTS1 = data8QTS(:,2);
x10QTS1 = data10QTS(:,2);
x13QTS1 = data13QTS(:,2);

x10STQ1 = data10STQ(:,2);
x11STQ1 = data11STQ(:,2);
x12STQ1 = data12STQ(:,2);
x14STQ1 = data14STQ(:,2);

y1QTS1 = data1QTS(:,3);
y8QTS1 = data8QTS(:,3);
y10QTS1 = data10QTS(:,3);
y13QTS1 = data13QTS(:,3);

y10STQ1 = data10STQ(:,3);
y11STQ1 = data11STQ(:,3);
y12STQ1 = data12STQ(:,3);
y14STQ1 = data14STQ(:,3);

x1QTS2 = data1QTS(:,4);
x8QTS2 = data8QTS(:,4);
x10QTS2 = data10QTS(:,4);
x13QTS2 = data13QTS(:,4);

x10STQ2 = data10STQ(:,4);
x11STQ2 = data11STQ(:,4);
x12STQ2 = data12STQ(:,4);
x14STQ2 = data14STQ(:,4);

y1QTS2 = data1QTS(:,5);
y8QTS2 = data8QTS(:,5);
y10QTS2 = data10QTS(:,5);
y13QTS2 = data13QTS(:,5);

y10STQ2 = data10STQ(:,5);
y11STQ2 = data11STQ(:,5);
y12STQ2 = data12STQ(:,5);
y14STQ2 = data14STQ(:,5);

angle1QTS = atan((x1QTS2-x1QTS1)./(y1QTS2-y1QTS1))*180/pi;
angle8QTS = atan((x8QTS2-x8QTS1)./(y8QTS2-y8QTS1))*180/pi;
angle10QTS = atan((x10QTS2-x10QTS1)./(y10QTS2-y10QTS1))*180/pi;
angle13QTS = atan((x13QTS2-x13QTS1)./(y13QTS2-y13QTS1))*180/pi;

angle10STQ = atan((x10STQ2-x10STQ1)./(y10STQ2-y10STQ1))*180/pi;
angle11STQ = atan((x11STQ2-x11STQ1)./(y11STQ2-y11STQ1))*180/pi;
angle12STQ = atan((x12STQ2-x12STQ1)./(y12STQ2-y12STQ1))*180/pi;
angle14STQ = atan((x14STQ2-x14STQ1)./(y14STQ2-y14STQ1))*180/pi;

x0=0;
y0=0;
plotwidth=500;
height=200;
xlim1 = 0;
xlim2 = 12;
lText = 'Measured angle';
lLoc = 'Location';
lPlacement = 'southwest';
yLabel = 'Theta [deg]';
xLabel = 'Time [s]';

figure(1)
plot(t1QTS(5:end),angle1QTS(5:end));
grid on
xlim([xlim1,xlim2]);
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle1QTS.pdf','ContentType','vector')

figure(2)
plot(t8QTS(5:end),angle8QTS(5:end));
grid on
xlim([xlim1,xlim2])
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle8QTS.pdf','ContentType','vector')

figure(3)
plot(t10QTS(5:end),angle10QTS(5:end));
grid on
xlim([xlim1,xlim2])
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle10QTS.pdf','ContentType','vector')

figure(4)
plot(t13QTS(5:end),angle13QTS(5:end));
grid on
xlim([xlim1,xlim2])
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle13QTS.pdf','ContentType','vector')

figure(5)
plot(t10STQ(5:end),angle10STQ(5:end));
grid on
xlim([xlim1,xlim2])
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle10STQ.pdf','ContentType','vector')

figure(6)
plot(t11STQ(5:end),angle11STQ(5:end));
grid on
xlim([xlim1,xlim2])
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle11STQ.pdf','ContentType','vector')

figure(7)
plot(t12STQ(5:end),angle12STQ(5:end));
grid on
xlim([xlim1,xlim2])
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle12STQ.pdf','ContentType','vector')

figure(8)
plot(t14STQ(5:end),angle14STQ(5:end));
grid on
xlim([xlim1,xlim2])
%legend(lText,lLoc,lPlacement);
xlabel(xLabel);
ylabel(yLabel);
set(gcf,'position',[x0,y0,plotwidth,height])
exportgraphics(gcf,'angle14STQ.pdf','ContentType','vector')