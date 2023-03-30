clear
clc

run("variables.m");

testData = readmatrix('testOfControllers\ControllerImpulseRespons3.txt');
testTime = (testData(:,1)-testData(1,1))/1000 + 1;
testCurrent = testData(:,2);

simulation = sim("CraneModel.slx");
%%

figure(1)
title("Controller step respons")
stairs(testTime, testCurrent);
hold on
stairs(simulation.tout, simulation.ControllerCurrent)
hold off
xlabel("time [s]")
ylabel("Controller output [A]")
legend("Imp. controller", "Model controller")



%%
figure(2)
plot(diff(testTime))


