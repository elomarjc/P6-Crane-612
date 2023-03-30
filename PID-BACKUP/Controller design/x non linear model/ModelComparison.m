clear
close all
clc
%run("variables.m");
%simXGain = 5;

testData = readmatrix('testOfControllers\DCoffsetTest2.txt', 'CommentStyle', '//');
testData = readmatrix('testOfControllers\4-5-2022\PIDtest13.txt', 'CommentStyle', '//');
testTime = (testData(:,1)-testData(1,1))/1000;
testX = testData(:,2);
testY = testData(:,4);
testAngle = testData(:,5);
testXRef = testData(:,6);
testXcon = testData(:,9);
testAnglecon = testData(:,8);

Xref = testXRef(1);
stepSize = abs(Xref-testX(1));
%InitXPos = testX(1);

%simulation = sim("CraneModel.slx");

figure(23597)
plot(testTime, testXRef)
ylim([0 4])

figure(2359899)
plot(testTime, testY)

figure(2358)
plot(testTime, testAngle)

figure(346)
plot(testTime, testX)

figure(32857)
plot(testTime, testXcon+testAnglecon)
ylim([-10 10])

%%
figure(1)
title("X-position")
plot(testTime, testX)
%hold on
%plot(simulation.tout, simulation.X1)
%hold off
grid on

figure(2)
title("angle")
plot(testTime, testAngle)
grid on

figure(3)
title("Container position")
plot(testTime, testX+sin(testAngle*pi/180).*testY)
grid on
errorBand = .064;
errorBandValue=stepSize*errorBand;
yline(Xref + errorBandValue)
yline(Xref - errorBandValue)
xlabel("Time [s]")
ylabel("X-position [m]")
title("Container position")

%%
figure(4)
title("STFT angle")
stft(testAngle, 100)
