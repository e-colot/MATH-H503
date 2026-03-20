clear; close all; clc;
% Using the parity method, fault detection is applied to the ship problem,
% with 3 sensors (redundancy)

%% Fault on 1st sensor

% DT system construction
    Ts = 0.2;
    eta1 = -10;
    b = 2.2;

    [A, B, C, D, E_d, E_f, G_d, G_f, sys] = generateShip(Ts, eta1, b, 2);

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
        f = zeros(3, N);
        f(1, floor(N/4):end) = 2; % step fault on first sensor
    
    [x, y] = simulateSystem(u, d, f, t, x0, sys);

s = 2;

% remove f_1 for r1:
    sysR1 = sys;
    sysR1.E_d = [sys.E_d, sys.E_f(:, 1)];
    sysR1.E_f = [sys.E_f(:, 2), sys.E_f(:, 3)];
    sysR1.G_d = [sys.G_d, sys.G_f(:, 1)];
    sysR1.G_f = [sys.G_f(:, 2), sys.G_f(:, 3)];
        % build r1
        [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sysR1);
    
        % Check existence of a non zero w_s:
            if (s+1)*size(y,1) - rank([Theta, T_d]) > 0
                disp('A non-zero w_s exists for r1');
            else
                error('No non-zero w_s exists, increase s for r1');
            end
    
        omega_s = null([Theta, T_d]')';
        if norm(omega_s*T_f) < 1e-6
            error('Fault detection condition not fulfilled!')
        end
    
        % Residual computation
    
        r1 = omega_s * (Y_s - T_u*U_s);

% remove f_2 for r2:
    sysR2 = sys;
    sysR2.E_d = [sys.E_d, sys.E_f(:, 2)];
    sysR2.E_f = [sys.E_f(:, 1), sys.E_f(:, 3)];
    sysR2.G_d = [sys.G_d, sys.G_f(:, 2)];
    sysR2.G_f = [sys.G_f(:, 1), sys.G_f(:, 3)];
        % build r2
        [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sysR2);
    
        % Check existence of a non zero w_s:
            if (s+1)*size(y,1) - rank([Theta, T_d]) > 0
                disp('A non-zero w_s exists for r2');
            else
                error('No non-zero w_s exists, increase s for r2');
            end
    
        omega_s = null([Theta, T_d]')';
        if norm(omega_s*T_f) < 1e-6
            error('Fault detection condition not fulfilled!')
        end
    
        % Residual computation
    
        r2 = omega_s * (Y_s - T_u*U_s);

% remove f_3 for r3:
    sysR3 = sys;
    sysR3.E_d = [sys.E_d, sys.E_f(:, 3)];
    sysR3.E_f = [sys.E_f(:, 1), sys.E_f(:, 2)];
    sysR3.G_d = [sys.G_d, sys.G_f(:, 3)];
    sysR3.G_f = [sys.G_f(:, 1), sys.G_f(:, 2)];
        % build r3
        [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sysR3);
    
        % Check existence of a non zero w_s:
            if (s+1)*size(y,1) - rank([Theta, T_d]) > 0
                disp('A non-zero w_s exists for r3');
            else
                error('No non-zero w_s exists, increase s for r3');
            end
    
        omega_s = null([Theta, T_d]')';
        if norm(omega_s*T_f) < 1e-6
            error('Fault detection condition not fulfilled!')
        end
    
        % Residual computation
    
        r3 = omega_s * (Y_s - T_u*U_s);

% Plots
    figure('Name', 'Step-like fault');
    colors = get(gca, 'ColorOrder');
    % fault + residual
        subplot(111);
        title('Fault and Residual');
        hold on;
        plot(t, f(1,:), Color=colors(1,:), LineStyle='-', LineWidth=1, DisplayName='f_1(t)');
        plot(t, f(2,:), Color=colors(2,:), LineStyle='-', LineWidth=1, DisplayName='f_2(t)');
        plot(t, f(3,:), Color=colors(3,:), LineStyle='-', LineWidth=1, DisplayName='f_3(t)');
        plot(t(s+1:N), sum(r1'*r1, 1), Color=colors(1,:), LineStyle='--', LineWidth=2, DisplayName='r_1(t)');
        plot(t(s+1:N), sum(r2'*r2, 1), Color=colors(2,:), LineStyle='--', LineWidth=2, DisplayName='r_2(t)');
        plot(t(s+1:N), sum(r3'*r3, 1), Color=colors(3,:), LineStyle='--', LineWidth=2, DisplayName='r_3(t)');
        legend();
        xlabel('Time [s]');
        ylabel('Amplitude');
        grid on;
