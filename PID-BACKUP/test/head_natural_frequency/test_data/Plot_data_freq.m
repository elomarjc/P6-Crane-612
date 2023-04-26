%% Read data files %% 
% Read data with container
data1_container = readtable("container_10cm.csv");
data2_container = readtable("container_50cm.csv");
data3_container = readtable("container_110cm.csv");

% Read data without container
data1 = readtable("noContainer_10cm.csv");
data2 = readtable("noContainer_50cm.csv");
data3 = readtable("noContainer_110cm.csv");

%% Make Short-time Fourier transform %%
% With container
[f1,A1] = fftFrequency(data1_container.Var1/1000,data1_container.Var2,20);
[f2,A2] = fftFrequency(data2_container.Var1/1000,data2_container.Var2,20);
[f3,A3] = fftFrequency(data3_container.Var1/1000,data3_container.Var2,20);

% Without container
[f4,A4] = fftFrequency(data1.Var1/1000,data1.Var2,20);
[f5,A5] = fftFrequency(data2.Var1/1000,data2.Var2,20);
[f6,A6] = fftFrequency(data3.Var1/1000,data3.Var2,20);


%% Make plots %%
% Plot with container
figure(1)
plot(f1, A1,f2,A2,f3,A3)
xlim([0 4])
xlabel('Frequency [Hz]')
ylabel('Magnitude ')
legend('10 cm','50 cm','110 cm')
title('With container')
grid on

x0=0;
   y0=0;
   plotwidth=500;
   height=300;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'fft_with_container.pdf','ContentType','vector')

% Plot without container
figure(2)
plot(f4, A4,f5,A5,f6,A6)
xlim([0 4])
xlabel('Frequency [Hz]')
ylabel('Magnitude ')
legend('10 cm','50 cm','110 cm')
title('Without container')
grid on

x0=0;
   y0=0;
   plotwidth=500;
   height=300;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'fft_without_container.pdf','ContentType','vector')

%% Note peaks of fft %%
% With container
freq_peak_with_container = [2.45614, 2.58373, 3.26923];
length_peak_with_container = [1.10, 0.5, 0.1];


% Without container
freq_peak_without_container = [2.2093, 2.14765, 2.69231];
length_peak_without_container = [1.10, 0.5, 0.1];

%% Make polyfit and plot %%
% With container
p1 = polyfit(length_peak_with_container,freq_peak_with_container,1)

x1 = linspace(0,1.4);
y1 = polyval(p1,x1);

tan_val1 = 3.85-atan(6.5*x1)

figure(3)
plot(length_peak_with_container,freq_peak_with_container,'o',x1,y1,x1,tan_val1)
xlabel('Wire length [m]')
ylabel('Frequency [Hz]')
legend('Measurements','Lin-reg','Atan-fit')
title('With container')
text(0.1,2.1,'f(x) = -0.7657 \cdot x + 3.2036')
text(0.1,2.25,'f(x) = 3.85 - atan(6.5 \cdot x)')
grid on

x0=0;
   y0=0;
   plotwidth=500;
   height=300;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'length_to_freq_with_container.pdf','ContentType','vector')

% Without container
p2 = polyfit(length_peak_without_container,freq_peak_without_container,1)

x2 = linspace(0,1.4);
y2 = polyval(p2,x1);

%s = 2.12
%h = 0.8
%c = 5

%tan_val2 = atan(-x1*c)*2*h/pi + s + h
tan_val2 = 2.92-0.51*atan(5*x2)

figure(4)
plot(length_peak_without_container,freq_peak_without_container,'o',x2,y2,x2,tan_val2)
xlabel('Wire length [m]')
ylabel('Frequency [Hz]')
legend('Measurements','Lin-reg','Atan-fit')
title('Without container')
text(0.1,1.87,'f(x) = -0.4368 \cdot x + 2.5973')
text(0.1,1.955,'f(x) = 2.92 - 0.51 \cdot atan(5 \cdot x)')
grid on

x0=0;
   y0=0;
   plotwidth=500;
   height=300;
   set(gcf,'position',[x0,y0,plotwidth,height])

exportgraphics(gcf,'length_to_freq_without_container.pdf','ContentType','vector')


%% Function used to create frequincy and magnitude axis %%
function [f, A] = fftFrequency (time,signal,freqinterp)
    x1 = linspace(time(1),time(length(time)),(time(length(time))-time(1))*freqinterp);
    y1 = interp1(time, signal,x1,'linear');
    A = (abs(fft(y1))./freqinterp)';
    f = (linspace(0,freqinterp,length(A)))';
end