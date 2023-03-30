clear
close all
clc


testData = readmatrix('testOfControllers\4-5-2022\PIDtest1.txt', 'CommentStyle', '//');


plotData(testData, "test 1")

%plotData(readmatrix('testOfControllers\29-4-2022\gain = 7.5\1 meter\test2.csv', 'CommentStyle', '//'), "test 2")

%plotData(readmatrix('testOfControllers\29-4-2022\gain = 7.5\2 cm\test1.csv', 'CommentStyle', '//'), "test 3")
%plotData(readmatrix('testOfControllers\29-4-2022\gain = 7.5\2 cm\test2.csv', 'CommentStyle', '//'), "test 4")

%plotPrevX(readmatrix('testOfControllers\28-4-2022\xControllerTest\rapportGains\test1.txt', 'CommentStyle', '//'), "rapport gains")
%plotPrevX(readmatrix('testOfControllers\28-4-2022\xControllerTest\rapportGains\test2.txt', 'CommentStyle', '//'), "rapport gains2")

function plotData(input, test)
    testTime = (input(:,1)-input(1,1))/1000000;
    testX = input(:,2)-input(1,2);
    testY = input(:,3);
    testAngle = input(:,4);
    testXRef = input(:,5);
    
    
    Xref = testXRef(1)-input(1,2);
    stepSize = abs(Xref-testX(1));
    

    figure
    plot(testTime, testX)
    xlabel("Time [s]")
    ylabel("Position [m]")
    title(test)


    figure
    plot(testTime, testX + sin(testAngle*pi/180).*testY)
    errorBand = .064;
    errorBandValue=stepSize*errorBand;
    yline(Xref + .05)
    yline(Xref - .05)
    
    xlabel("Time [s]")
    ylabel("Position [m]")
    title(test)
   % export_fig(strcat("XcontrollerTestFinalGain", test, ".pdf"))

    figure
    plot(testTime, testAngle)
    xlabel("Time [s]")
    ylabel("Angle [deg]")
    title(test)
end
function plotPrevX(input, test)
    testTime = (input(:,1)-input(1,1))/1000000;
    testX = input(:,2)-input(1,2);
    testY = input(:,3);
    testAngle = input(:,4);
    testXRef = input(:,7);
    
    
    Xref = testXRef(1)-input(1,2);
    stepSize = abs(Xref-testX(1));
    

    figure
    plot(testTime, testX)
    xlabel("Time [s]")
    ylabel("Position [m]")
    title(test)
    grid on

    figure
    plot(testTime, testX + sin(testAngle*pi/180).*testY)
    errorBand = .064;
    errorBandValue=stepSize*errorBand;
    yline(Xref + errorBandValue)
    yline(Xref - errorBandValue)
    
    xlabel("Time [s]")
    ylabel("Position [m]")
    %title(strcat("Container position, ", test))
    export_fig(strcat("ContainerPosition ", test, ".pdf"))
    

    figure
    plot(testTime, testAngle)
    xlabel("Time [s]")
    ylabel("Angle [deg]")
    title(test)
end


function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650/1.5;
    plotHeight=400/1.5;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end