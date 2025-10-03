% Data format: [roll pitch yaw distance]
data = csvread("magsensor\PCL.csv");

roll  = data(:,1);
pitch = data(:,2);
yaw   = data(:,3);
dist  = data(:,4);

% Filter distance range 0–300
mask = (dist >= 0 & dist <= 300);
roll  = roll(mask);
pitch = pitch(mask);
yaw   = yaw(mask);
dist  = dist(mask);

% Convert degrees to radians
pitch = deg2rad(pitch);
yaw   = deg2rad(yaw);

% Convert to Cartesian coordinates
x = dist .* cos(pitch) .* cos(yaw);
y = dist .* cos(pitch) .* sin(yaw);
z = dist .* sin(pitch);

% Plot point cloud
figure;
scatter3(x, y, z, 50, 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Point Cloud from Roll, Pitch, Yaw, Distance');
grid on; axis equal;

% Optional: connect points
hold on;
plot3(x, y, z, 'r--');
