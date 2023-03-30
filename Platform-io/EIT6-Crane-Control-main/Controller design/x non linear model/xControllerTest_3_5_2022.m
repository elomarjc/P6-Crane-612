clear
close all
clc


testData = readmatrix('testOfControllers\4-5-2022\crossStep2.txt', 'CommentStyle', '//');
testData1 = readmatrix('testOfControllers\4-5-2022\PIDtest13.txt', 'CommentStyle', '//');
%testData2 = readmatrix("testOfControllers\3-5-2022\SingleStep6.txt");

[testTime0, testContainer0, conout0] = plotData(testData, "Cross step");
[testTime1, testContainer1, conout1] = plotData(testData1, "PID test 13");
%[testTime2, testContainer2, conout2] = plotData(testData2, "Single step 6");



% 
% close all
% plot(testTime1, testContainer1)
% hold on 
% plot(testTime2, testContainer2)
% hold off
% legend("Kryds step", "Single step");
%    yline(3 + .04)
%     yline(3 - .04)
%     
%     xlabel("Time [s]")
%     ylabel("Position [m]")
% 

function[testTime, testContainerX, conOutput] =  plotData(input, test)
    %Serial.println(String(millis())+ ", " + String(in.posTrolley.x)+ ", "
    %+ String(in.posTrolley.y) + "," +String(in.angle) +  ","+ String(ref.x) + ", " + String(ref.y) + ", " + String(xInnerConOut ) + ", " + String(xOuterConOut));

    testTime = (input(:,1)-input(1,1))/1000;
    testX = input(:,2);
    testY = input(:,4);
    testAngle = input(:,5);
    testXRef = input(:,6);
    testYRef = input(:,7);
    testAngleconOut = input(:,8);
    testXconOut = input(:,9);
    testContainerX = input(:,3);
    
    conOutput = [testXconOut testAngleconOut];

    Xref = testXRef(1);
    stepSize = abs(Xref-testX(1));
    

    figure
    plot(testTime, testX)
    xlabel("Time [s]")
    ylabel("Position [m]")
    title(strcat(test,", Trolley x"))

    figure
    plot(testTime, testY)
    xlabel("Time [s]")
    ylabel("Position [m]")
    title(strcat(test,", Trolley y"))
    

    figure
    plot(testTime, testX + sin(testAngle*pi/180).*testY)
    hold on
    plot(testTime, testContainerX)
    hold off
    errorBand = .064;
    errorBandValue=stepSize*errorBand;
    Xref = 3.5;
    yline(Xref + .05)
    yline(Xref - .05)
    xlim([0 14]);
    
    xlabel("Time [s]")
    ylabel("Position [m]")
    title(strcat(test,", Container x"))
    legend("Ml container position", "con container position", "Location","southeast")
    %export_fig(strcat("XcontrollerTestFinalGain", test, ".pdf"))

    figure
    plot(testTime, testAngle)
    xlabel("Time [s]")
    ylabel("Angle [deg]")
    title(strcat(test,", angle"))

    figure
    plot(testTime, testXconOut)
    hold on
    plot(testTime, testAngleconOut)
    hold off
    title(strcat(test,", Controller value"))
    legend("X controller", "Angle controller")
    grid on
    xlabel("Time [s]")
    ylabel("Current [Ampere]")
    
end

function plotData2(input, test)
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


function export_fig(name)
    x0=0;
    y0=0;
    plotwidth=650/1.5;
    plotHeight=400/1.5;
    set(gcf,'position',[x0,y0,plotwidth,plotHeight])
    
    
    exportgraphics(gcf,name,'ContentType','vector')
end