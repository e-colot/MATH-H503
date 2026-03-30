clear; close all; clc;

% Uses d[k] = r_1[k]² + r_2[k]² as fault criterion

%% System + signals generation

    N = 1e4;
    n_sensors = 3;
    C = ones(n_sensors, 1);
    steadyState = 2;
    
    x = steadyState * ones(1, N);
    R = diag([0.1 0.1 0.2]);
    
    noise = R.^(0.5) * randn(3, N);
    
    f = zeros(1, N);
    f(1, N/2:end) = 0.1 * steadyState;
    
    % healthy
        y_h = C * x + noise;
    % fault on sensor 1
        y_1 = y_h + [1;0;0] * f;
    % fault on sensor 2
        y_2 = y_h + [0;1;0] * f;
    % fault on sensor 3
        y_3 = y_h + [0;0;1] * f;

% check signals
%     figure;
%     hold on;
%     title('First sensor output')
%     plot(y_h(1,:), LineWidth=2, DisplayName='y_{healthy}');
%     plot(y_1(1,:), LineWidth=2, DisplayName='y_1');
%     plot(y_2(1,:), LineWidth=2, DisplayName='y_2');
%     legend()


%% Compute residuals
N_window_list = [1 10 100 500];

figure('Name', 'Windowed stochastic residual');
colors = get(gca, 'colororder');
hold on;

for N_window = N_window_list
    omega = null(C')';
    F = inv(chol(omega*R*omega.')) * omega;
    F_normalized = F./vecnorm(F);
    
    % sensor 1 fault
    y = y_1;
    r = F * y;
    
    
    % chi-square test
    alpha = 0.01;
    gamma = chi2inv(1 - alpha, N_window*2);
    
    r1_squared = r(1,:).^2;
    r2_squared = r(2,:).^2;

    r_squared = r1_squared + r2_squared;

    d = r_squared(N_window:end);
    samples = N_window:N_window-1+length(d);
    for k = 1:N_window-1
        d = d + r_squared(N_window-k:end-k);
    end
        
    plot(samples, d/gamma, LineWidth=1.5, DisplayName=['$N_w = $' num2str(N_window)]);
    above_gamma = sum(d > gamma);
    above_pct = 100 * above_gamma / N;
    fprintf('%.2f%% above threshold\n', above_pct);
end

plot([N_window N], [1, 1], LineStyle="--", LineWidth=2, Color='r', DisplayName='Threshold')
xlabel('time step k');
ylabel('d[k]/thr');
legend('Interpreter','latex');