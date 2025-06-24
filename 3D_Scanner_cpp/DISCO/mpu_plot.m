data = csvread("mpu_data.csv");     % Load CSV data
ax = data(:, 1);
gx = data(:, 2);
roll = data(:, 3);
ay =  data(:, 4);
gy =  data(:, 5);
pitch =  data(:, 6);
samples = (1:length(ax));  % X-axis

figure;
subplot(2,1,1);
plot(samples, ax, 'r', samples, gx, 'g', samples, roll, 'b');
legend("ax", "gx", "roll");
xlabel("Sample");
ylabel("Angle (degrees)");
title("Roll Orientation");
grid on;

subplot(2,1,2);
plot(samples, ay, 'r', samples, gy, 'g', samples, pitch, 'b');
legend('ay', 'gy', 'pitch');
xlabel('Sample');
ylabel('Value');
title('Pitch Axis (Y) Data');
grid on;
