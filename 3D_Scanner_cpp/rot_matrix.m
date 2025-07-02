pkg load symbolic

% Step 1: Define symbolic variables
syms phi theta psi dphi dtheta dpsi real

% Step 2: Unit vectors
i = sym([1; 0; 0]);
j = sym([0; 1; 0]);
k = sym([0; 0; 1]);

% Step 3: Rotation matrices
RX = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)];

RY = [cos(theta) 0 sin(theta);
      0 1 0;
     -sin(theta) 0 cos(theta)];

% Step 4: Rotated vectors
RXj = RX * j;
RXYk = RX * RY * k;

% Step 5: Construct ω
omega = i * dphi + RXj * dtheta + RXYk * dpsi;

% Step 6: Display result
disp("ω =")
disp(simplify(omega))

