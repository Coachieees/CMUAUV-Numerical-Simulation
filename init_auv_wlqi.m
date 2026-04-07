clear; clc;

%% 1. Physical Parameters
m = 10.21; % Mass in air (kg)
W = m * 9.81; % Weight (N)

Vol = 0.011214; % Mass in water (kg)
B = 1000 * Vol * 9.81; % Buoyancy (N)

% Center of Mass (CoM) offset from Body-fixed frame origin
rg = [0; 0; 0.0620]; % [xg; yg; zg] in meters

% Center of Buoyancy (CoB) offset 
rb = [0; 0; 0.0277];         

% Moment of Inertia matrix (kg*m^2)
% calculated about the IMU origin (I0).
I0 = [ 0.120750,       0, 0.002566; 
            0,  0.165852,       0; 
      0.002566,       0,  0.189496];

% Rigid Body Mass Matrix (Mrb)
Mrb = [ m*eye(3),       -m*Smtrx(rg);
        m*Smtrx(rg),     I0 ];

%% 2. Hydrodynamic Parameters
% Added Mass Matrix (Ma)
Ma = diag([6.36, 7.12, 18.68, 0.189, 0.135, 0.222]);

% Mass Matrix (M)
M = Mrb + Ma;
M_inv = inv(M);        
disp('Inertia Matrix (M):');
disp(M);

% Linear Damping Matrix (Dl)
Dl = diag([13.7, 0, 33, 0, 0.8, 0]);

% Quadratic Damping Matrix (Dq)
Dq = diag([141.0, 217.0, 190.0, 1.19, 0.47, 1.5]);

%% 3. Thruster Configuration Matrix (TCM)
% Thruster positions relative to CM
l_com = [ 150,  110, 44;   % T1
          150, -110, 44;   % T2
         -150,  110, 44;   % T3
         -150, -110, 44;   % T4
           50,  120,  0;   % T5
           50, -120,  0;   % T6
          -50,  120,  0;   % T7
          -50, -120,  0] / 1000; % T8

% Define unit direction vectors for each thruster (v_i)
v = zeros(8, 3);

% T1 to T4 (Surge, Sway, Yaw) 
% Angles are relative to the +y axis. 
alpha = deg2rad([45, 135, -45, -135]); 
for i = 1:4
    v(i, :) = [sin(alpha(i)), cos(alpha(i)), 0];
end

% T5 to T8 (Heave, Roll, Pitch)
for i = 5:8
    v(i, :) = [0, 0, 1]; 
end

% Construct the 6x8 TCM (T) matrix
T = zeros(6, 8);
for i = 1:8
    force_dir = v(i, :)';
    pos = l_com(i, :)'; % Using l_com directly to match the spreadsheet moment arms
    moment = cross(pos, force_dir);
    T(:, i) = [force_dir; moment];
end

% Display the TCM in the command window to verify
disp('Thruster Configuration Matrix (T):');
disp(T);


%% Helper Function
% skew-symmetric matrix of a 3x1 vector
function S = Smtrx(a)
    S = [     0, -a(3),  a(2);
           a(3),     0, -a(1);
          -a(2),  a(1),     0 ];
end

%% 4. Linearization around Hover
% Linearized restoring forces matrix (G) around phi=0, theta=0
G_lin = zeros(6,6);
G_lin(1,5) = W - B;       % Pitch effect on surge
G_lin(2,4) = -(W - B);    % Roll effect on sway
G_lin(4,4) = rg(3) * W;   % Roll restoring moment
G_lin(5,5) = rg(3) * W;   % Pitch restoring moment

% Construct 12x12 A matrix
A11 = -M_inv * Dl;
A12 = -M_inv * G_lin;
A21 = eye(6);             % J(eta) is Identity matrix at small Euler angles
A22 = zeros(6,6);
A_sys = [A11, A12; A21, A22];

% Construct 12x8 B matrix
B1 = M_inv * T;
B2 = zeros(6,8);
B_sys = [B1; B2];

%% 5. LQI Controller Design (Depth Tracking)
% Original state weighting matrix Q (12x12)
Q_vel = diag([50, 1, 1, 1, 1, 50]);          % [u, v, w, p, q, r]
Q_pos = diag([100, 100, 100, 50, 50, 200]); % [x, y, z, phi, theta, psi]
Q_x = blkdiag(Q_vel, Q_pos);

% Define the C matrix to extract ONLY the depth state (z is the 9th state)
Cz = zeros(1, 12);
Cz(9) = 1; 

% Augment the A and B matrices for the integral state
% A_aug = [A_sys, 0; Cz, 0]
A_aug = [A_sys,       zeros(12, 1);
         Cz,          0];

% B_aug = [B_sys; 0]
B_aug = [B_sys;
         zeros(1, 8)];

% Add a tuning weight for the new integral depth state
Qi = 100; % Integral weight for depth

% Augmented Q matrix (13x13)
Q_aug = blkdiag(Q_x, Qi);

% Control effort weighting matrix R (8x8)
R = 0.1 * eye(8); 

% Calculate the Augmented LQI Gain Matrix (13x8)
K_aug = lqr(A_aug, B_aug, Q_aug, R);

% Split the gains for Simulink implementation
K_x = K_aug(:, 1:12); % Proportional LQR gains for the 12 main states
K_i = K_aug(:, 13);   % Integral gain for the depth error

disp('LQI Proportional Gain (K_x):');
disp(K_x);
disp('LQI Integral Gain (K_i):');
disp(K_i);

%% 6. Buoyancy Feedforward (Trim)
tau_trim = [0; 0; B - W; 0; 0; 0]; 

% Use pseudo-inverse of TCM to find required thruster outputs for trim
u_trim = pinv(T) * tau_trim;

%% 7. Observer Design (Kalman Filter / LQE)
% the Measurement Matrix (C)
C = [zeros(9, 3), eye(9)];

% Feedforward Matrix (D)
D = zeros(9, 8);

% Process Noise Covariance (Qn)
Qn = eye(12) * 0.01; 

% Measurement Noise Covariance (Rn)
Rn = eye(9) * 0.02; 

% Calculate the optimal Observer Gain Matrix (L)
[L, ~, ~] = lqe(A_sys, eye(12), C, Qn, Rn);

disp('Observer Gain Matrix (L):');
disp(L)
