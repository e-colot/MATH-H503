clear; close all; clc;

% -------- System Creation ----------

Ts = 1e-3;
load("noiselessData\triphase_healthy_0008.mat");
y1 = signals_nonoise;
load("noiselessData\triphase_faulty_0034.mat");
y2 = signals_nonoise;

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

    figure('Name', 'Data');
    colors = get(gca, 'ColorOrder');

    hold on;
    plot(step, y1(1,:), Color=colors(1,:), LineWidth=2, LineStyle="-", DisplayName='y_1[k]');
    plot(step, y1(2,:), Color=colors(2,:), LineWidth=2, LineStyle="-", DisplayName='y_2[k]');
    plot(step, y1(3,:), Color=colors(3,:), LineWidth=2, LineStyle="-", DisplayName='y_3[k]');

    legend();
    grid on;
    xlabel('Time sample[k]');
    ylabel('Amplitude');


%% Fault present

y = y2; % faulty

% sensor 1
    C1 = C(1,:);
    x0 = 0.05 * randn(2, 1);
    
    L = place(A', C1', [0.2 0.3])';

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    r1 = zeros(1, N);

    for k = 1:N
        r1(k) = y(1,k) - C1*x_hat(:, k);
        x_hat(:, k+1) = A*x_hat(:, k) + L*r1(k);
    end

% sensor 2
    C2 = C(2,:);
    x0 = 0.05 * randn(2, 1);
    
    L = place(A', C2', [0.2 0.3])';

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    r2 = zeros(1, N);

    for k = 1:N
        r2(k) = y(2,k) - C2*x_hat(:, k);
        x_hat(:, k+1) = A*x_hat(:, k) + L*r2(k);
    end

% sensor 3
    C3 = C(3,:);
    x0 = 0.05 * randn(2, 1);
    
    L = place(A', C3', [0.2 0.3])';

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    r3 = zeros(1, N);

    for k = 1:N
        r3(k) = y(3,k) - C3*x_hat(:, k);
        x_hat(:, k+1) = A*x_hat(:, k) + L*r3(k);
    end

    figure('Name', 'Fault present');
    colors = get(gca, 'ColorOrder');

    subplot(211);    
    hold on;
    plot(step, y2(1,:), Color=colors(1,:), LineWidth=2, LineStyle="-", DisplayName='y_1[k]');
    plot(step, y2(2,:), Color=colors(2,:), LineWidth=2, LineStyle="-", DisplayName='y_2[k]');
    plot(step, y2(3,:), Color=colors(3,:), LineWidth=2, LineStyle="-", DisplayName='y_3[k]');

    legend();
    grid on;
    xlabel('Time sample[k]');
    ylabel('Amplitude');

    subplot(212);
    hold on;
    plot(step, r1, Color=colors(1,:), LineWidth=2, LineStyle="-", DisplayName='r_1[k]');
    plot(step, r2, Color=colors(2,:), LineWidth=2, LineStyle="-.", DisplayName='r_2[k]');
    plot(step, r3, Color=colors(3,:), LineWidth=2, LineStyle="--", DisplayName='r_3[k]');

    legend();
    grid on;
    xlabel('Time sample[k]');
    ylabel('Amplitude');
    ylim([-0.01, 0.05]);

%% different residuals

y = y2; % faulty

% sensor 1
    C1 = [C(2,:); C(3,:)];
    x0 = 0.05 * randn(2, 1);
    
    L = place(A', C1', [0.2 0.3])';

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    r1 = zeros(2, N);

    for k = 1:N
        r1(:, k) = [y(2,k);y(3,k)] - C1*x_hat(:, k);
        x_hat(:, k+1) = A*x_hat(:, k) + L*r1(:, k);
    end

% sensor 2
    C2 = [C(1,:); C(3,:)];
    x0 = 0.05 * randn(2, 1);
    
    L = place(A', C2', [0.2 0.3])';

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    r2 = zeros(2, N);

    for k = 1:N
        r2(:, k) = [y(1,k);y(3,k)] - C2*x_hat(:, k);
        x_hat(:, k+1) = A*x_hat(:, k) + L*r2(:, k);
    end

% sensor 3
    C3 = [C(1,:); C(2,:)];
    x0 = 0.05 * randn(2, 1);
    
    L = place(A', C3', [0.2 0.3])';

    x_hat = zeros(size(A, 1), N+1);
    x_hat(:, 1) = x0;
    r3 = zeros(2, N);

    for k = 1:N
        r3(:, k) = [y(1,k);y(2,k)] - C3*x_hat(:, k);
        x_hat(:, k+1) = A*x_hat(:, k) + L*r3(:, k);
    end

    figure('Name', 'Fault present alt');
    colors = get(gca, 'ColorOrder');

    hold on;
    plot(step, sum(r1'*r1, 1), Color=colors(1,:), LineWidth=2, LineStyle="-", DisplayName='r_1[k]');
    plot(step, sum(r2'*r2, 1), Color=colors(2,:), LineWidth=2, LineStyle="-.", DisplayName='r_2[k]');
    plot(step, sum(r3'*r3, 1), Color=colors(3,:), LineWidth=2, LineStyle="--", DisplayName='r_3[k]');

    legend();
    grid on;
    xlabel('Time sample[k]');
    ylabel('Amplitude');
    ylim([-2, 10]);