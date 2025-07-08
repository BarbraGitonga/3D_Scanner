data = csvread("LoggedData.csv");     % Load CSV data
roll = data(:, 1);
gx = data(:, 2);
pitch = data(:, 3);

samples = (1:length(roll));  % X-axis

figure;
plot(samples, roll, 'r', samples, gx, 'g', samples, pitch, 'b');
legend("roll", "gx", "pitch");
xlabel("Sample");
ylabel("rad/s");
title("Rolla nd pitch Orientation");
grid on;


