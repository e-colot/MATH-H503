clear; close all; clc;
% Using the parity method, fault detection is applied to the ship problem,
% with 2 sensors and no physical redundancy
% The fault is applied on both sensors alternatively, with a step or a ramp
% shape

%% DT system construction
    Ts = 0.2;
    eta1 = -10;
    b = 2.2;

    [A, B, C, D, E_d, E_f, G_d, G_f, sys] = generateShip(Ts, eta1, b, 1);

%%   --------- ERROR ON THE FIRST STATE ----------

% Simulation

    % Parameters
    T = 50; % total duration
    t = 0:Ts:T;
    N = length(t);
    x0 = [0; 0];

    % Signals
        % input
        u = 1*sin(2*pi*0.13*t + 1) + 2*sin(2*pi*0.08*t-0.1);
        % disturbance
        f_w = 0.2;
        Amplitude = 2;
        d = Amplitude*sin(2*pi*f_w*t);
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
    
    figure('Name', 'Step-like fault');
    colors = get(gca, 'ColorOrder');

% Plots
    % subplot 1: input and output
        subplot(4,2,1);
        title('System signals');
        hold on;
        plot(t, u(1,:), Color=colors(1,:), LineStyle='-', LineWidth=2, DisplayName='u(t)');
        plot(t, y(1,:), Color=colors(2,:), LineStyle='-', LineWidth=2, DisplayName='y_1(t)');
        plot(t, y(2,:), Color=colors(3,:), LineStyle='-', LineWidth=2, DisplayName='y_2(t)');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
    
    % subplot 2: state
        subplot(4,2,3);
        title('System state');
        hold on;
        plot(t, x(1,:), Color=colors(4,:), LineStyle='-', LineWidth=2, DisplayName='x_1(t)');
        plot(t, x(2,:), Color=colors(5,:), LineStyle='-', LineWidth=2, DisplayName='x_2(t)');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
    
    % subplot 3: disturbance
        subplot(4,2,5);
        title('Disturbance');
        hold on;
        plot(t, d(1,:), Color=colors(6,:), LineStyle='-', LineWidth=2, DisplayName='d(t)');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
    
    % subplot 4: fault + residual
        subplot(4,2,7);
        title('Fault and Residual');
        hold on;
        plot(t, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1(t)');
        plot(t, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2(t)');
        plot(t(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-.', LineWidth=2, DisplayName='residual');
        legend();
        xlabel('Time [s]');
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
    % subplot 1: input and output
        subplot(4,2,2);
        title('System signals');
        hold on;
        plot(t, u(1,:), Color=colors(1,:), LineStyle='-', LineWidth=2, DisplayName='u(t)');
        plot(t, y(1,:), Color=colors(2,:), LineStyle='-', LineWidth=2, DisplayName='y_1(t)');
        plot(t, y(2,:), Color=colors(3,:), LineStyle='-', LineWidth=2, DisplayName='y_2(t)');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
    
    % subplot 2: state
        subplot(4,2,4);
        title('System state');
        hold on;
        plot(t, x(1,:), Color=colors(4,:), LineStyle='-', LineWidth=2, DisplayName='x_1(t)');
        plot(t, x(2,:), Color=colors(5,:), LineStyle='-', LineWidth=2, DisplayName='x_2(t)');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
    
    % subplot 3: disturbance
        subplot(4,2,6);
        title('Disturbance');
        hold on;
        plot(t, d(1,:), Color=colors(6,:), LineStyle='-', LineWidth=2, DisplayName='d(t)');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
    
    % subplot 4: fault + residual
        subplot(4,2,8);
        title('Fault and Residual');
        hold on;
        plot(t, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1(t)');
        plot(t, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2(t)');
        plot(t(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-.', LineWidth=2, DisplayName='residual');
        legend();
        xlabel('Time [s]');
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
    figure('Name', 'Ramp-like fault');
    % fault + residual
        subplot(121);
        title('Fault and Residual');
        hold on;
        plot(t, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1(t)');
        plot(t, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2(t)');
        plot(t(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-.', LineWidth=2, DisplayName='residual');
        legend();
        xlabel('Time [s]');
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
        subplot(222);
        title('Fault and Residual');
        hold on;
        plot(t, f(1,:), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='f_1(t)');
        plot(t, f(2,:), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='f_2(t)');
        plot(t(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-.', LineWidth=2, DisplayName='residual');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
    
        subplot(224);
        title('Zoomed in version of residual');
        hold on;
        plot(t(s+1:N), sum(r'*r, 1), Color='r', LineStyle='-.', LineWidth=2, DisplayName='residual');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
