% Load data
sensorReadings = readtable("LoggedData.csv");
ax = sensorReadings{:, 2}; % ax
ay = sensorReadings{:, 1}; % ay
az = sensorReadings{:, 3}; % az
gx = sensorReadings{:, 5}; % gx
gy = sensorReadings{:, 4}; % gy
gz = sensorReadings{:, 6}; % gz

n = height(sensorReadings);

% Filter initialization
T = 0.01;                   % Sampling period
g = 9.81;                   % Gravity
x = [0; 0];                 % Initial state [phi; theta]
P = 0.1 * eye(2);           % Covariance
Q = 0.003 * eye(2);         % Process noise
R = 0.1 * eye(3);           % Measurement noise

% Preallocate storage
phi_est = zeros(n, 1);
theta_est = zeros(n, 1);

% Step 1: Symbolic Setup (outside the loop)
syms phi theta p q r real
x_sym = [phi; theta];

% Nonlinear state update f(x, u)
f_sym = [
    phi + T * (p + tan(theta)*(q*sin(phi) + r*cos(phi)));
    theta + T * (q*cos(phi) - r*sin(phi))
];

A_sym = jacobian(f_sym, x_sym);

% Measurement model h(x)
h_sym = 9.81 * [
    sin(theta);
    -cos(theta)*sin(phi);
    -cos(theta)*cos(phi)
];
C_sym = jacobian(h_sym, x_sym);

% Convert symbolic expressions to MATLAB functions
f_func = matlabFunction(f_sym, 'Vars', {x_sym, [p; q; r]});
A_func = matlabFunction(A_sym, 'Vars', {x_sym, [p; q; r]});
h_func = matlabFunction(h_sym, 'Vars', {x_sym});
C_func = matlabFunction(C_sym, 'Vars', {x_sym});

for i = 1:n
    % Inputs
    p_val = gx(i);
    q_val = gy(i);
    r_val = gz(i);
    acc = [ax(i); ay(i); az(i)];

    % Prediction
    f = f_func(x, [p_val; q_val; r_val]);
    A = A_func(x, [p_val; q_val; r_val]);
    P = P + T * (A * P + P * A' + Q);

    % Measurement
    h = h_func(f);
    C = C_func(f);
    y = acc - h;

    % Kalman gain
    S = C * P * C' + R;
    K = P * C' / S;

    % Update
    x = f + K * y;
    P = (eye(2) - K * C) * P;

    phi_est(i) = x(1);
    theta_est(i) = x(2);
end

% --- PLOT RESULTS ---
t = (0:n-1) * T;
figure;

% --- Subplot 1: EKF estimated angles ---
subplot(2,1,1);
plot(t, phi_est, 'r', 'DisplayName', '\phi (roll)');
hold on;
plot(t, theta_est, 'b', 'DisplayName', '\theta (pitch)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend;
grid on;
title('EKF Estimated Roll and Pitch');

% --- Subplot 2: Gyroscope inputs ---
subplot(2,1,2);
plot(t, gx, 'm', 'DisplayName', 'gx (roll rate)');
hold on;
plot(t, gy, 'c', 'DisplayName', 'gy (pitch rate)');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
legend;
grid on;
title('Gyroscope Readings (gx and gy)');
