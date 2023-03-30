%% Read data files %% 
% Read data with container
data1_container = readtable("container_10cm.csv");
data2_container = readtable("container_30cm.csv");
data3_container = readtable("container_50cm.csv");
data4_container = readtable("container_70cm.csv");
data5_container = readtable("container_90cm.csv");
data6_container = readtable("container_110cm.csv");

% Read data without container
data1 = readtable("noContainer_10cm.csv");
data2 = readtable("noContainer_30cm.csv");
data3 = readtable("noContainer_50cm.csv");
data4 = readtable("noContainer_70cm.csv");
data5 = readtable("noContainer_90cm.csv");
data6 = readtable("noContainer_110cm.csv");


%% Make Short-time Fourier transform %%
% With container
[c_f1,c_A1] = fftFrequency(data1_container.Var1/1000000,data1_container.Var2,20);
[c_f2,c_A2] = fftFrequency(data2_container.Var1/1000000,data2_container.Var2,20);
[c_f3,c_A3] = fftFrequency(data3_container.Var1/1000000,data3_container.Var2,20);
[c_f4,c_A4] = fftFrequency(data4_container.Var1/1000000,data4_container.Var2,20);
[c_f5,c_A5] = fftFrequency(data5_container.Var1/1000000,data5_container.Var2,20);
[c_f6,c_A6] = fftFrequency(data6_container.Var1/1000000,data6_container.Var2,20);

% Without container
[f1,A1] = fftFrequency(data1.Var1/1000000,data1.Var2,20);
[f2,A2] = fftFrequency(data2.Var1/1000000,data2.Var2,20);
[f3,A3] = fftFrequency(data3.Var1/1000000,data3.Var2,20);
[f4,A4] = fftFrequency(data4.Var1/1000000,data4.Var2,20);
[f5,A5] = fftFrequency(data5.Var1/1000000,data5.Var2,20);
[f6,A6] = fftFrequency(data6.Var1/1000000,data6.Var2,20);


%% Make plots %%
% Plot with container
figure(1)
plot(c_f1, c_A1, c_f2, c_A2, c_f3, c_A3, c_f4, c_A4, c_f5, c_A5, c_f6, c_A6)
xlim([0 4])
xlabel('Frequency [Hz]')
ylabel('Magnitude ')
legend('10 cm','30 cm','50 cm','70 cm','90 cm','110 cm')
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
plot(f1, A1, f2, A2, f3, A3, f4, A4, f5, A5, f6, A6)
xlim([0 4])
xlabel('Frequency [Hz]')
ylabel('Magnitude ')
legend('10 cm','30 cm','50 cm','70 cm','90 cm','110 cm')
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
freq_peak_with_container = [2.33184, 2.39669, 2.44275, 2.61307, 2.7234, 3.42105];
length_peak_with_container = [1.10, 0.9, 0.7, 0.5, 0.3, 0.1];


% Without container
freq_peak_without_container = [2.15827, 2.25166, 2.06186, 2.22222, 2.32258, 2.58621];
length_peak_without_container = [1.10, 0.9, 0.7, 0.5, 0.3, 0.1];

%% Make polyfit and plot %%
% With container
p1 = polyfit(length_peak_with_container,freq_peak_with_container,1)

x1 = linspace(0,1.4);
y1 = polyval(p1,x1);

tan_val1 = 3.85-atan(6.5*x1)

figure(3)
plot(length_peak_with_container,freq_peak_with_container,'o',x1,tan_val1)
ylim([1.8 4])
xlabel('Wire length [m]')
ylabel('Frequency [Hz]')
legend('Measurements','Atan-fit')
title('With container')
%text(0.1,2.1-0.45,'f(x) = -0.7657 \cdot x + 3.2036')
text(0.1,2.4-0.45,'f(x) = 3.85 - atan(6.5 \cdot x)')
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

%s = 2.05
%h = 0.8
%c = 5

%tan_val2 = atan(-x1*c)*2*h/pi + s + h
tan_val2 = 2.85-0.51*atan(5*x2)

figure(4)
plot(length_peak_without_container,freq_peak_without_container,'o',x2,tan_val2)
ylim([1.8 4])
xlabel('Wire length [m]')
ylabel('Frequency [Hz]')
legend('Measurements','Atan-fit')
title('Without container')
%text(0.1,1.87,'f(x) = -0.4368 \cdot x + 2.5973')
text(0.1,1.955,'f(x) = 2.85 - 0.51 \cdot atan(5 \cdot x)')
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