clear
close all
clc


plotTrolley("PIDtest13.2.txt")
plotAngle("PIDtest13.2.txt")
plotContainer("PIDtest13.2.txt")

% figure(1)
% hold on
% plotTrolley("PIDtest13.txt")
% %export_fig("TunedTrolleyPositionX246cmL075cm.pdf")
% figure(2)
% hold on
% plotAngle("PIDtest13.txt")
% %export_fig("TunedAngleX246cmL075cm.pdf")
% figure(3)
% hold on
% plotContainer("PIDtest13.txt")
% %export_fig("TunedContainerPositionX246cmL075cm.pdf")
% hold off
% 
% figure(1)
% hold on
% plotTrolley("step3m-2.txt")
% %export_fig("TunedTrolleyPositionX246cmL075cm.pdf")
% figure(2)
% hold on
% plotAngle("step3m-2.txt")
% %export_fig("TunedAngleX246cmL075cm.pdf")
% figure(3)
% hold on
% plotContainer("step3m-2.txt")
% %export_fig("TunedContainerPositionX246cmL075cm.pdf")
% hold off
% 
% figure(1)
% legend("Test 1","Test 2","Test 3", "Location","southeast")
% figure(2)
% legend("Test 1","Test 2","Test 3", "Location","southeast")
% figure(3)
% legend("Test 1","Test 2","Test 3", "Location","southeast")
% 
figure(1)
ylim([0 3.5])
% 
% 
figure(1)
grid on
export_fig("TrolleyPosition3MstepTuned.pdf")

figure(2)
grid on
export_fig("Angle3MstepTuned.pdf")

figure(3)
grid on
export_fig("ContainerPosition3MstepTuned.pdf")



function plotContainer(test1)
    data1 = readmatrix(test1);
    data1(:,1) = (data1(:,1)- data1(1,1))/1000;
    xstart = data1(1,3);
    data1(:,3) = data1(:,3)-xstart;
    data1(:,6) = data1(:,6)-xstart;

    figure(3)
    plot(data1(:,1), data1(:,3))
    hold on
    yline(data1(1,6)+.05,':')
    yline(data1(1,6)-.05,':')
    yline(data1(1,6))
    hold off
    xlabel("time [s]")
    ylabel("Container position [m]")
    xlim([0, 15])    
end


function plotAngle(test1)
    data1 = readmatrix(test1);
    data1(:,1) = (data1(:,1)- data1(1,1))/1000;
    
    figure(2)
    plot(data1(:,1), data1(:,5))
    xlabel("time [s]")
    ylabel("Angle [deg]")
    xlim([0, 15])
end


function plotTrolley(test1)
    data1 = readmatrix(test1);
    data1(:,1) = (data1(:,1)- data1(1,1))/1000;
    xstart = data1(1,2);
    data1(:,2) = data1(:,2)-xstart;
    data1(:,6) = data1(:,6)-xstart;

    figure(1)
    plot(data1(:,1), data1(:,2))
    hold on
    yline(data1(1,6)+.05,':')
    yline(data1(1,6)-.05,':')
    yline(data1(1,6))
    hold off
    xlabel("time [s]")
    ylabel("Trolley position [m]")
    xlim([0, 15])
end


function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650/1.5;
    plotHeight=400/1.5;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end