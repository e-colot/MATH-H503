clear; close all; clc;


N = 15e3;

% white gaussian noise sequence with a step-like fault
variance = 0.5;
m_0 = 0;
m_1 = 0.5;

r = sqrt(variance) * randn(1, N) + m_0;

fault = zeros(1, N);
fault(floor(N/3):floor(2*N/3)) = fault(floor(N/3):floor(2*N/3)) + (m_1 - m_0);

r = r + fault;

fprintf('True fault start in %d \n', floor(N/3));
fprintf('True fault end in %d \n\n', floor(2*N/3));


h = 20;

% Check signal generation
% figure;
% plot(r);

N_window_list = [200 500 1000];
figure('Name', 'GLR residual'); hold on;
colors = get(gca, 'colororder');

err = r-m_0;
for M = N_window_list

    g = zeros(1, N);
    nu_hat = zeros(1, N);

    % For each sample
    for k = M:N

        best_g = -inf;
        best_j = -1;

        % Suboptimal loop but improves readability
        for j = k-M+1:k

            cum_err = sum(err(j:k));

            new_g = 1/(2*variance * (k-j+1)) * cum_err.^2;

            if new_g > best_g
                best_j = j;
                best_g = new_g;
            end

        end

        g(k) = best_g;
        if g(k) > h
            nu_hat(k) = 1/(k-best_j+1) * sum(err(best_j:k));
        end

    end

    name = ['M = ', num2str(M)];
    subplot(211); hold on;
    plot(g, LineWidth=2, DisplayName=name);
    subplot(212); hold on;
    plot(nu_hat, LineWidth=2, DisplayName=name);
end

subplot(211); hold on;
%legend('Interpreter','latex');
ylabel('g[k]');
xlim([4 11]*1e3);
grid on;

subplot(212); hold on;
plot(fault, 'r--', LineWidth=2, DisplayName='True fault');
legend('Interpreter','latex');
ylabel('\nu[k]');
xlim([4 11]*1e3);
grid on;
xlabel('Time step [k]');

