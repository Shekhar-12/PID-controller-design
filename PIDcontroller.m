%% PID Controller Design for DC Motor Speed Control
% -----------------------------------------------

clc;
clear;
close all;

%% DC Motor Parameters
J = 0.01;     % Moment of inertia (Kg.m^2)
B = 0.1;      % Damping coefficient (N.m.s)
K = 0.01;     % Motor torque constant (N.m/A or V.s/rad)
R = 1;        % Armature resistance (Ohm)
L = 0.5;      % Armature inductance (H)

%% Transfer Function of DC Motor
% G(s) = K / [(L*J)s^2 + (L*B + R*J)s + (R*B + K^2)]
num = [K];
den = [L*J, (L*B + R*J), (R*B + K*K)];
plant = tf(num, den);

disp('Open-loop Transfer Function of DC Motor:');
plant

%% Step Response of Open Loop System
figure;
step(plant);
title('Open-Loop Step Response of DC Motor');
xlabel('Time');
ylabel('Angular Velocity (rad/s)');
grid on;

%% PID Controller Design
% Automatically tune PID controller using MATLAB's pidtune
C = pidtune(plant, 'PID');
disp('Tuned PID Controller Parameters:');
C

Kp = C.Kp;
Ki = C.Ki;
Kd = C.Kd;

fprintf('\nKp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp, Ki, Kd);

%% Closed-Loop System with PID Controller
T = feedback(C * plant, 1);   % Unity feedback

%% Step Response of Closed Loop System
figure;
step(T);
title('Closed-Loop Response with PID Controller');
xlabel('Time');
ylabel('Angular Velocity (rad/s)');
grid on;

%% Compare Open-Loop vs Closed-Loop Responses
figure;
step(plant, 'r', T, 'b');
legend('Open Loop', 'PID Controlled');
title('Comparison: Open-Loop vs PID Closed-Loop Response');
xlabel('Time');
ylabel('Angular Velocity (rad/s)');
grid on;

%% Performance Analysis
S_open = stepinfo(plant);
S_closed = stepinfo(T);

disp('---------------------------------------------');
disp('Performance Comparison:');
fprintf('Rise Time (Open Loop): %.4f s\n', S_open.RiseTime);
fprintf('Rise Time (Closed Loop): %.4f s\n', S_closed.RiseTime);
fprintf('Settling Time (Open Loop): %.4f s\n', S_open.SettlingTime);
fprintf('Settling Time (Closed Loop): %.4f s\n', S_closed.SettlingTime);
fprintf('Overshoot (Open Loop): %.2f %%\n', S_open.Overshoot);
fprintf('Overshoot (Closed Loop): %.2f %%\n', S_closed.Overshoot);

%% Step Response with Disturbance (Optional)
% To simulate a load disturbance, add a step input at t = 2s.
t = 0:0.01:5;
disturbance = ones(size(t));
[y, t_out] = step(T, t);

% Add disturbance of -0.2 (for example)
disturbed_output = y - 0.2 * disturbance';

figure;
plot(t_out, disturbed_output, 'b', 'LineWidth', 1.5);
title('Closed-Loop Response with Load Disturbance');
xlabel('Time (seconds)');
ylabel('Angular Velocity (rad/s)');
grid on;

disp('Simulation completed successfully.');
disp('Observe the plots for open-loop, closed-loop, and disturbed responses.');
