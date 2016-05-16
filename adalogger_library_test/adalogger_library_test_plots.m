% ECHO | Plotting SD card Write Times
clc; clf

data = xlsread('SDechoTest_LOG00_again.csv'); % copy/pasted from Serial Monitor
lastSample = 1e4; % just for plotting a fixed number of samples
data = data(1:lastSample) -data(1); % so the timing starts at 0
data = data*1e-6; % from micros to seconds

subplot(211); %plot((data -data(1))*1e-6)
    plot(data, 'b');    hold on
    plot([1, length(data)], [0, lastSample/48e3], 'g', ...
        [1, length(data)], [0, lastSample/96e3], 'r')
    hold off
    legend('051516 result','48 kHz','96 kHz')
    xlabel('Number of writes')
    ylabel('Time elapsed (seconds)')

deltaT = data(2:end) -data(1:end-1); % time between writes
subplot(212)
    plot(1:length(deltaT), deltaT, 'ob')
    yLimit = max(abs(deltaT)); % used to set axis limits
    axis([1 lastSample -yLimit yLimit])
    xlabel('Number of writes')
    ylabel('Time between writes (seconds)')

suptitle('Writing micros()''s output to SD card using SD/File.println() on Adafruit Adalogger')
