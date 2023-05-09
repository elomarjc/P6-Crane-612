% OBS, dette er en Octave fil, udkommenter pkg load signal hvis det skal bruges i matlab

pkg load signal

clear all

data1 = csvread("angleData.csv");
data2 = csvread("angleDataHand.csv");

time1 = data1(:,1)./1e6;
angle1 = data1(:,2);

time2 = data2(:,1)./1e6;
angle2 = data2(:,2);

function [f, A] = fftFrequency (time,signal,freqinterp)
    x1 = linspace(time(1),time(length(time)),(time(length(time))-time(1))*freqinterp);
    y1 = interp1(time, signal,x1,'linear');
    A = (abs(fft(y1))./freqinterp)';
    f = (linspace(0,freqinterp,length(A)))';
endfunction

[f1,A1] = fftFrequency(time1,angle1,20);
[f2,A2] = fftFrequency(time2,angle2,20);

figure(1)
plot(f1, A1,f2,A2)
xlim([0 10])
grid on

figure(2)
specgram(ifft(A1),length(ifft(A1))-1)

figure(3)
specgram(ifft(A2),length(ifft(A2))-1)
