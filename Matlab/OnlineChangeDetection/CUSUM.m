clear; close all; clc;


N = 15e3;

% white gaussian noise sequence with a step-like fault
variance = 0.5;
m_0 = 0;
m_1 = 0.5;

r = sqrt(variance) * randn(1, N) + m_0;

r(floor(N/3):floor(2*N/3)) = r(floor(N/3):floor(2*N/3)) + (m_1 - m_0);

fprintf('True fault start in %d \n', floor(N/3));
fprintf('True fault end in %d \n\n', floor(2*N/3));

% Check signal generation
% figure;
% plot(r);

%% Basic CUSUM

s = (m_1-m_0)/variance * (r - (m_0+m_1)/2);
g = zeros(size(r));
d = zeros(size(r));

h = 50; % threshold for g

g(1) = max(0, s(1));
d(1) = 1;

triggered = 0;

for k = 2:N
    g(k) = max(0, g(k-1) + s(k));
    if g(k-1) > 0
        d(k) = d(k-1) + 1;
    else
        d(k) = 1;
    end
    if (g(k) > h) & (triggered == 0)
        fprintf('Fault appearance estimated in %d \n', k-d(k));
        triggered = 1;
    end
end


figure('Name', 'CUSUM');
colors = get(gca, 'colororder');

subplot(321); hold on; grid on;
plot(r, LineWidth=2, color=colors(1,:));
ylabel('r[k]');
xlim([4 11]*1e3);
subplot(323); hold on; grid on;
plot(g, LineWidth=2, color=colors(3,:), DisplayName='g[k]');
plot([1 N], [h, h], LineStyle="--", LineWidth=2, Color='r', DisplayName='h');
ylabel('Amplitude');
xlim([4 11]*1e3);
legend('Interpreter','latex');
subplot(325); hold on; grid on;
plot(d, LineWidth=2, color=colors(2,:));
xlabel('Time step [k]');
ylabel('Detection delay');
xlim([4 11]*1e3);

%% CUSUM with reset

s = (m_1-m_0)/variance * (r - (m_0+m_1)/2);
g = zeros(size(r));
d = zeros(size(r));

h = 50; % threshold for g

g(1) = max(0, s(1));
d(1) = 1;
orientation = 1; % determines if checking for appearance (+1) or disappearance (-1)


for k = 2:N
    g(k) = max(0, g(k-1) + orientation * s(k));
    if g(k-1) > 0
        d(k) = d(k-1) + 1;
    else
        d(k) = 1;
    end
    if g(k) > h
        if orientation == 1
            fprintf('Fault appearance estimated in %d \n', k-d(k));
        else
            fprintf('Fault disappearance estimated in %d \n', k-d(k));
        end

        orientation = -orientation;
        g(k) = max(0, orientation * s(k));
        d(k) = 0;
    end
end

subplot(322); hold on; grid on;
plot(r, LineWidth=2, color=colors(1,:));
xlim([4 11]*1e3);
subplot(324); hold on; grid on;
plot(g, LineWidth=2, color=colors(3,:), DisplayName='Decision function');
plot([1 N], [h, h], LineStyle="--", LineWidth=2, Color='r', DisplayName='Threshold');
xlim([4 11]*1e3);
ylim([min(g) h+5]);
%legend('Interpreter','latex');
subplot(326); hold on; grid on;
plot(d, LineWidth=2, color=colors(2,:));
xlim([4 11]*1e3);
xlabel('Time step [k]');

