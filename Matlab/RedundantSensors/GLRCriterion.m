clear; close all; clc;

% Uses the generalized log-likelihood ratio as fault criterion

%% System + signals generation

    N = 1e4;
    n_sensors = 3;
    M = ones(n_sensors, 1);
    steadyState = 2;
    
    eta = steadyState * ones(1, N);
    S = diag([0.1 0.1 0.2]);
    
    noise = S.^(0.5) * randn(3, N);
    
    f = 0.1*steadyState * ones(1, N);
    
    % healthy
        y_h = M * eta + noise;
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

%% Healthy signal
disp('Healthy signal:')

Z = y_h;

N_window_list = [1 10 100 500];
figure('Name', 'GLR residual');
colors = get(gca, 'colororder');

% Constant value used for ln lambda computation
insideTerm = inv(S) - inv(S) * M * inv(M.'*inv(S)*M) * M.' * inv(S);

hold on;

for N_window = N_window_list

    lnLambda = diag(Z.' * insideTerm * Z);

    % Remove the first elements (window not long enough)
    d = lnLambda(N_window:end);
    samples = N_window:N_window-1+length(d);

    for k = 1:N_window-1
        d = d + lnLambda(N_window-k:end-k);
    end
    
    % chi-square test
    alpha = 0.01;
    gamma = chi2inv(1 - alpha, N_window*2);
        
    plot(samples, d/gamma, LineWidth=1.5, DisplayName=['$N_w = $' num2str(N_window)]);
    above_gamma = sum(d > gamma);
    above_pct = 100 * above_gamma / N;
    fprintf('    %.2f%% above threshold with a window size of %d \n', above_pct, N_window);
end

plot([1 N], [1, 1], LineStyle="--", LineWidth=2, Color='r', DisplayName='Threshold')
xlabel('time step k');
ylabel('ln(\lambda)/thr');
xlim([max(N_window_list) N]);
legend('Interpreter','latex');

%% Sensor 1 fault
disp('Fault on sensor 1:')
Z = y_1;

N_window_list = [1 10 100 500];
figure('Name', 'GLR residual');
colors = get(gca, 'colororder');

% Constant value used for ln lambda computation
insideTerm = inv(S) - inv(S) * M * inv(M.'*inv(S)*M) * M.' * inv(S);

hold on;

for N_window = N_window_list

    lnLambda = diag(Z.' * insideTerm * Z);

    % Remove the first elements (window not long enough)
    d = lnLambda(N_window:end);
    samples = N_window:N_window-1+length(d);

    for k = 1:N_window-1
        d = d + lnLambda(N_window-k:end-k);
    end
    
    % chi-square test
    alpha = 0.01;
    gamma = chi2inv(1 - alpha, N_window*2);
        
    plot(samples, d/gamma, LineWidth=1.5, DisplayName=['$N_w = $' num2str(N_window)]);
    above_gamma = sum(d > gamma);
    above_pct = 100 * above_gamma / N;
    fprintf('    %.2f%% above threshold with a window size of %d \n', above_pct, N_window);
end

plot([1 N], [1, 1], LineStyle="--", LineWidth=2, Color='r', DisplayName='Threshold')
xlabel('time step k');
ylabel('ln(\lambda)/thr');
xlim([max(N_window_list) N]);
legend('Interpreter','latex');
