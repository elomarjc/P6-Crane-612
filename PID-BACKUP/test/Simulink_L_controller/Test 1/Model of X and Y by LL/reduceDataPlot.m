
function [time, data]=reduceDataPlot(inputTime,inputData)
    time= transpose( inputTime(1): inputTime(length(inputTime))/500 :inputTime(length(inputTime)));
    data=interp1(inputTime, inputData, time);
    plot(time,data)
end


