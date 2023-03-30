clear
close all
clc


plotTrolley("Untuned2.txt")
grid on
export_fig("UntunedTrolleyPosition.pdf")

plotAngle("Untuned2.txt")
grid on
export_fig("UntunedAngle.pdf")

plotContainer("Untuned2.txt")
grid on
export_fig("UntunedContanerPosition.pdf")

% 
% plotTrolley("nonTunedConL115mX246m.csv", "nonTunedConL115mX246mv2.csv")
% export_fig("nonTunedTrolleyPositionX246cmL115cm.pdf")
% plotAngle("nonTunedConL115mX246m.csv", "nonTunedConL115mX246mv2.csv")
% export_fig("nonTunedAngleX246cmL115cm.pdf")
% plotContainer("nonTunedConL115mX246m.csv", "nonTunedConL115mX246mv2.csv")
% export_fig("nonTunedContainerPositionX246cmL115cm.pdf")
% 
% 
% plotTrolley("nonTunedConL115mX0785m.csv", "nonTunedConL115mX0785mv2.csv")
% export_fig("nonTunedTrolleyPositionX0785cmL115cm.pdf")
% plotAngle("nonTunedConL115mX0785m.csv", "nonTunedConL115mX0785mv2.csv")
% export_fig("nonTunedAngleX0785cmL115cm.pdf")
% plotContainer("nonTunedConL115mX0785m.csv", "nonTunedConL115mX0785mv2.csv")
% export_fig("nonTunedContainerPositionX0785cmL115cm.pdf")
% 
% 
% plotTrolley("nonTunedConL075mX0785m.csv", "nonTunedConL075mX0785mv2.csv")
% export_fig("nonTunedTrolleyPositionX0785cmL075cm.pdf")
% plotAngle("nonTunedConL075mX0785m.csv", "nonTunedConL075mX0785mv2.csv")
% export_fig("nonTunedAngleX0785cmL075cm.pdf")
% plotContainer("nonTunedConL075mX0785m.csv", "nonTunedConL075mX0785mv2.csv")
% export_fig("nonTunedContainerPositionX0785cmL075cm.pdf")
disp("Job done")




function plotContainer(test1)
    data1 = readmatrix(test1);
    data1(:,1) = (data1(:,1)- data1(1,1))/1000;
    
  
    figure
    plot(data1(:,1), -data1(:,3)+3.5)
    hold on
    yline(3.05)
    yline(2.95)
    hold off
    xlabel("time [s]")
    ylabel("Container position[m]")
    xlim([0, 15])    
end


function plotAngle(test1)
    data1 = readmatrix(test1);
    data1(:,1) = (data1(:,1)- data1(1,1))/1000;


    figure
    plot(data1(:,1), -data1(:,5))
    hold on
  
    hold off
    xlabel("time [s]")
    ylabel("Angle [deg]")
    xlim([0, 15])
end


function plotTrolley(test1, test2)
    data1 = readmatrix(test1);
    data1(:,1) = (data1(:,1)- data1(1,1))/1000;
    
  
    figure
    plot(data1(:,1), -data1(:,2)+3.5)
    hold on
    yline(3.05)
    yline(2.95)
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