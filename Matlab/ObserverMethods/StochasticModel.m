clear; close all; clc;

% -------- System Creation ----------

Ts = 1e-3;
load("noisyData\triphase_snr25db_healthy_0008.mat");
y1 = signals;
load("noisyData\triphase_snr25db_faulty_0034.mat");
y2 = signals;

N = size(y1, 2);
w_exc = 2*pi/(50*Ts);

step = 0:N-1;

A_continuous = [0,  w_exc;
                -w_exc, 0];
B_continuous = zeros(2, 1);
E_continuous = zeros(2, 3);

C_continuous = [      1,              0;
                cos(-2*pi/3),   sin(-2*pi/3);
                 cos(2*pi/3),    sin(2*pi/3)];
D_continuous = zeros(3, 1);
G_continuous = eye(3);

sys_continuous = ss(A_continuous, [B_continuous, E_continuous], C_continuous, [D_continuous, G_continuous]);
sys_discrete = c2d(sys_continuous, Ts, 'zoh');

A = sys_discrete.A;
B_ = sys_discrete.B;
C = sys_discrete.C;
D_ = sys_discrete.D;
B = B_(:, 1);
E = B_(:, 2:end);
D = D_(:, 1);
G = D_(:, 2:end);

clear B_ D_;

% Noise estimation:
    % load non noisy signal
    load("noiselessData\triphase_healthy_0008.mat");
    y0 = signals_nonoise;
    err = y1 - y0;
    variances = var(err, 0, 2);

%% Healthy signal

y = y1;

% Sensor 1

    C_mod = C(1,:);
    y_mod = y(1,:);
    R_mod = diag([variances(1)]);

    x0 = randn(2, 1);
    sigm_0 = eye(size(A, 1)) * 100; % high state uncertainty in the beginning

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    res = zeros(1, N);
    sigm = zeros(size(A, 1), size(A, 2), N+1);
    sigm(:,:,1) = sigm_0;

    for k = 1:N
        K = A * sigm(:,:,k) * C_mod.' * inv(C_mod*sigm(:,:,k)*C_mod.' + R_mod);
        x_hat(:,k+1) = A*x_hat(:,k) + K*(y_mod(:,k) - C_mod*x_hat(:,k));
        sigm(:,:,k+1) = A*sigm(:,:,k)*A.' - K*C_mod*sigm(:,:,k)*A.';
        res(k) = y_mod(:,k) - C_mod*x_hat(:,k);
    end

    r1 = res;

% Sensor 2

    C_mod = C(2,:);
    y_mod = y(2,:);
    R_mod = diag([variances(2)]);

    x0 = randn(2, 1);
    sigm_0 = eye(size(A, 1)) * 100; % high state uncertainty in the beginning

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    res = zeros(1, N);
    sigm = zeros(size(A, 1), size(A, 2), N+1);
    sigm(:,:,1) = sigm_0;

    for k = 1:N
        K = A * sigm(:,:,k) * C_mod.' * inv(C_mod*sigm(:,:,k)*C_mod.' + R_mod);
        x_hat(:,k+1) = A*x_hat(:,k) + K*(y_mod(:,k) - C_mod*x_hat(:,k));
        sigm(:,:,k+1) = A*sigm(:,:,k)*A.' - K*C_mod*sigm(:,:,k)*A.';
        res(k) = y_mod(:,k) - C_mod*x_hat(:,k);
    end

    r2 = res;

% Sensor 3

    C_mod = C(3,:);
    y_mod = y(3,:);
    R_mod = diag([variances(3)]);

    x0 = randn(2, 1);
    sigm_0 = eye(size(A, 1)) * 100; % high state uncertainty in the beginning

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    res = zeros(1, N);
    sigm = zeros(size(A, 1), size(A, 2), N+1);
    sigm(:,:,1) = sigm_0;

    for k = 1:N
        K = A * sigm(:,:,k) * C_mod.' * inv(C_mod*sigm(:,:,k)*C_mod.' + R_mod);
        x_hat(:,k+1) = A*x_hat(:,k) + K*(y_mod(:,k) - C_mod*x_hat(:,k));
        sigm(:,:,k+1) = A*sigm(:,:,k)*A.' - K*C_mod*sigm(:,:,k)*A.';
        res(k) = y_mod(:,k) - C_mod*x_hat(:,k);
    end

    r3 = res;

% plots

    figure('Name', 'Faultless');
    colors = get(gca, 'ColorOrder');

    hold on;
    plot(step, r1, Color=colors(1,:), LineWidth=2, LineStyle="-", DisplayName='r_1[k]');
    plot(step, r2, Color=colors(2,:), LineWidth=2, LineStyle="-", DisplayName='r_2[k]');
    plot(step, r3, Color=colors(3,:), LineWidth=2, LineStyle="-", DisplayName='r_3[k]');

    legend();
    grid on;
    xlabel('Time sample[k]');
    ylabel('Amplitude');

%% Faulty signal

y = y2;

% Sensor 1

    C_mod = C(1,:);
    y_mod = y(1,:);
    R_mod = diag([variances(1)]);

    x0 = randn(2, 1);
    sigm_0 = eye(size(A, 1)) * 100; % high state uncertainty in the beginning

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    res = zeros(1, N);
    sigm = zeros(size(A, 1), size(A, 2), N+1);
    sigm(:,:,1) = sigm_0;

    for k = 1:N
        K = A * sigm(:,:,k) * C_mod.' * inv(C_mod*sigm(:,:,k)*C_mod.' + R_mod);
        x_hat(:,k+1) = A*x_hat(:,k) + K*(y_mod(:,k) - C_mod*x_hat(:,k));
        sigm(:,:,k+1) = A*sigm(:,:,k)*A.' - K*C_mod*sigm(:,:,k)*A.';
        res(k) = y_mod(:,k) - C_mod*x_hat(:,k);
    end

    r1 = res;

% Sensor 2

    C_mod = C(2,:);
    y_mod = y(2,:);
    R_mod = diag([variances(2)]);

    x0 = randn(2, 1);
    sigm_0 = eye(size(A, 1)) * 100; % high state uncertainty in the beginning

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    res = zeros(1, N);
    sigm = zeros(size(A, 1), size(A, 2), N+1);
    sigm(:,:,1) = sigm_0;

    for k = 1:N
        K = A * sigm(:,:,k) * C_mod.' * inv(C_mod*sigm(:,:,k)*C_mod.' + R_mod);
        x_hat(:,k+1) = A*x_hat(:,k) + K*(y_mod(:,k) - C_mod*x_hat(:,k));
        sigm(:,:,k+1) = A*sigm(:,:,k)*A.' - K*C_mod*sigm(:,:,k)*A.';
        res(k) = y_mod(:,k) - C_mod*x_hat(:,k);
    end

    r2 = res;

% Sensor 3

    C_mod = C(3,:);
    y_mod = y(3,:);
    R_mod = diag([variances(3)]);

    x0 = randn(2, 1);
    sigm_0 = eye(size(A, 1)) * 100; % high state uncertainty in the beginning

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    res = zeros(1, N);
    sigm = zeros(size(A, 1), size(A, 2), N+1);
    sigm(:,:,1) = sigm_0;

    for k = 1:N
        K = A * sigm(:,:,k) * C_mod.' * inv(C_mod*sigm(:,:,k)*C_mod.' + R_mod);
        x_hat(:,k+1) = A*x_hat(:,k) + K*(y_mod(:,k) - C_mod*x_hat(:,k));
        sigm(:,:,k+1) = A*sigm(:,:,k)*A.' - K*C_mod*sigm(:,:,k)*A.';
        res(k) = y_mod(:,k) - C_mod*x_hat(:,k);
    end

    r3 = res;

% plots

    figure('Name', 'Fault present');
    colors = get(gca, 'ColorOrder');

    hold on;
    plot(step, r1, Color=colors(1,:), LineWidth=2, LineStyle="-", DisplayName='r_1[k]');
    plot(step, r2, Color=colors(2,:), LineWidth=2, LineStyle="-", DisplayName='r_2[k]');
    plot(step, r3, Color=colors(3,:), LineWidth=2, LineStyle="-", DisplayName='r_3[k]');

    legend();
    grid on;
    xlabel('Time sample[k]');
    ylabel('Amplitude');