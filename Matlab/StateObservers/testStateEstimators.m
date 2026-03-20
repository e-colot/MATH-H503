clear; close all; clc;

Ts = 1e-2;

Ac = [-22 0;
        1 0];
Bc = [2.2;
        0];
Cc = [1 0;
      0 1];
Dc = 0;

sysC = ss(Ac, Bc, Cc, Dc);
sysD = c2d(sysC, Ts, 'zoh');

Ad = sysD.A;
Bd = sysD.B;
Cd = sysD.C;
Dd = sysD.D;

x0 = [10; -5];
% simulate
N = 100;
x_true = repmat(x0, 1, N+1);
y = zeros(size(Cc, 1), N);
u = 0.8 * randn(size(Bd, 2), N);

noiseVar = 0.1;
processNoise = noiseVar * randn(size(x_true));

for k = 1:N
    x_true(:,k+1) = Ad*x_true(:,k) + Bd*u(:,k) + processNoise(:,k);
    y(:,k) = Cd*x_true(:,k) + Dd*u(:,k);
end

x_est = Luenberger(Ad, Bd, Cd, Dd, y, u, [0.2 + 0.1j; 0.2 - 0.1j], x0 + 2 * randn(size(x0)));

figure;
subplot(211);
plot(x_true(1, :), LineWidth=2, Color='g');
hold on
plot(x_est(1,:), LineWidth=1.5, Color='r');
title('State 1');
legend('True state', 'Estimated state (Luenberger)');
xlabel('Sample');
ylabel('Amplitude');

subplot(212);
plot(x_true(2, :), LineWidth=2, Color='g');
hold on
plot(x_est(2,:), LineWidth=1.5, Color='r');
title('State 2');
legend('True state', 'Estimated state (Luenberger)');
xlabel('Sample');
ylabel('Amplitude');
