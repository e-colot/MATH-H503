clear; close all; clc;
% Using the parity method, fault detection is applied to the ship problem,
% with 2 sensors and no physical redundancy
% The fault is applied on both sensors alternatively, with a step or a ramp
% shape

rng(42);

%% DT system construction
    Ts = 0.2;
    eta1 = -10;
    b = 2.2;

    [A, B, C, D, E_d, E_f, G_d, G_f, sys] = generateShip(Ts, eta1, b, 1);

%% PLOT 1
% Simulation

    % Parameters
    T = 50; % total duration
    t = 0:Ts:T;
    N = length(t);
    step = 0:N-1;
    x0 = [0; 0];

    % Signals
        % input
        u = 3*randn(1, N);
        % disturbance
        f_w = 0.2;
        Amplitude = 0.5;
        d = Amplitude*sin(2*pi*f_w*t);
        % fault
        f = zeros(2, N);
        %f(1, floor(N/4):end) = 2; % step fault on first sensor
    
    [x, y] = simulateSystem(u, d, f, t, x0, sys);

    
figure("Name", "System Evolution")
colors = get(gca, 'ColorOrder');

    subplot(211);
    hold on;
    plot(step, u(1,:), LineWidth=2, Color=colors(1,:), DisplayName='u[k]');
    plot(step, y(1,:), LineWidth=2, Color=colors(2,:), DisplayName='y_1[k]');
    plot(step, y(2,:), LineWidth=2, Color=colors(3,:), DisplayName='y_2[k]');
    legend();
    grid on;
    xlabel('Time sample [k]');
    ylabel('Amplitude');
    
    subplot(212);
    hold on;
    plot(step, x(1,:), LineWidth=2, Color=colors(4,:), DisplayName='x_1[k]');
    plot(step, x(2,:), LineWidth=2, Color=colors(5,:), DisplayName='x_2[k]');
    plot(step, d(1,:), LineWidth=2, Color=colors(6,:), DisplayName='d[k]');
    legend();
    grid on;
    xlabel('Time sample [k]');
    ylabel('Amplitude');

%% PLOT 2

% Simulation

    % Signals
        % fault
        f = zeros(2, N);
        f(1, floor(N/4):end) = 2; % step fault on first sensor
    
    [x, y] = simulateSystem(u, d, f, t, x0, sys);

% Fault detection

    s = 2;
    [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sys);
    
    % Check existence of a non zero w_s:
        if (s+1)*size(y,1) - rank([Theta, T_d]) > 0
            disp('A non-zero w_s exists');
        else
            error('No non-zero w_s exists, increase s');
        end
    
    omega_s = null([Theta, T_d]')';
    
    if norm(omega_s*T_f) < 1e-6
        error('Fault detection condition not fulfilled!')
    end
    
    % Residual computation
    
    r = omega_s * (Y_s - T_u*U_s);
    
    figure('Name', 'Step Fault');
    colors = get(gca, 'ColorOrder');

% Plots
        subplot(121);
        hold on;
        plot(step, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1[k]');
        plot(step, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2[k]');
        plot(step(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-', LineWidth=1.2, DisplayName='||r[k]||^2');
        legend();
        xlabel('Time sample [k]');
        ylabel('Amplitude');
        grid on;

%%   --------- ERROR ON THE SECOND STATE ----------

% Simulation

    % Signals
        % fault
        f = zeros(2, N);
        f(2, floor(N/4):end) = 2; % step fault on second sensor
    
    [x, y] = simulateSystem(u, d, f, t, x0, sys);

% Fault detection

    s = 2;
    [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sys);
    
    % Check existence of a non zero w_s:
        if (s+1)*size(y,1) - rank([Theta, T_d]) > 0
            disp('A non-zero w_s exists');
        else
            error('No non-zero w_s exists, increase s');
        end
    
    omega_s = null([Theta, T_d]')';
    
    if norm(omega_s*T_f) < 1e-6
        error('Fault detection condition not fulfilled!')
    end
    
    % Residual computation
    
    r = omega_s * (Y_s - T_u*U_s);

% Plots
    subplot(122);
        hold on;
        plot(step, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1[k]');
        plot(step, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2[k]');
        plot(step(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-', LineWidth=1.2, DisplayName='||r[k]||^2');
        legend();
        xlabel('Time sample [k]');
        ylabel('Amplitude');
        grid on;

%% --------- Ramp fault on sensor 1 ----------

% Simulation

    % Signals
        % fault
        f = zeros(2, N);
        f(1, N-floor(3*N/4):N) = (0:floor(3*N/4))/floor(3*N/4); % ramp fault on first sensor
    
    [~, y] = simulateSystem(u, d, f, t, x0, sys);

    % Fault detection
    
    s = 2;
    [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sys);
    
    % Check existence of a non zero w_s:
        if (s+1)*size(y,1) - rank([Theta, T_d]) > 0
            disp('A non-zero w_s exists');
        else
            error('No non-zero w_s exists, increase s');
        end
    
    omega_s = null([Theta, T_d]')';
    
    if norm(omega_s*T_f) < 1e-6
        error('Fault detection condition not fulfilled!')
    end
    
    % Residual computation
    
    r = omega_s * (Y_s - T_u*U_s);

% Plots
    figure('Name', 'Ramp Fault');
    % fault + residual
        subplot(121);
        hold on;
        plot(step, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1[k]');
        plot(step, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2[k]');
        plot(step(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-', LineWidth=1.2, DisplayName='||r[k]||^2');
        legend();
        xlabel('Time sample [k]');
        ylabel('Amplitude');
        grid on;

%% --------- Ramp fault on sensor 2 ----------

% Simulation

    % Signals
        % fault
        f = zeros(2, N);
        f(2, N-floor(3*N/4):N) = (0:floor(3*N/4))/floor(3*N/4); % ramp fault on first sensor
    
    [~, y] = simulateSystem(u, d, f, t, x0, sys);

% Fault detection

    s = 2;
    [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sys);
    
    % Check existence of a non zero w_s:
        if (s+1)*size(y,1) - rank([Theta, T_d]) > 0
            disp('A non-zero w_s exists');
        else
            error('No non-zero w_s exists, increase s');
        end
    
    omega_s = null([Theta, T_d]')';
    
    if norm(omega_s*T_f) < 1e-6
        error('Fault detection condition not fulfilled!')
    end
    
    % Residual computation
    
    r = omega_s * (Y_s - T_u*U_s);

% Plots
    % fault + residual
        subplot(122);
        hold on;
        plot(step, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1[k]');
        plot(step, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2[k]');
        plot(step(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-', LineWidth=1.2, DisplayName='||r[k]||^2');
        legend();
        xlabel('Time sample [k]');
        ylabel('Amplitude');
        ylim([0; 2*max(sum(r'*r, 1))])
        grid on;
    
        % subplot(224);
        % hold on;
        % plot(step(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-', LineWidth=1.2, DisplayName='||r[k]||^2');
        % legend();
        % xlabel('Time sample [k]');
        % ylabel('Amplitude');
        % grid on;
