function [x, y] = simulateSystem(u, d, f, t, x0, sys)
% simulates the systems evolution
% written by E. Colot on March 16 2026

N = length(t);

% Time evolution
    x = zeros(size(sys.A,1), N+1);
    y = zeros(size(sys.C,1), N);
    x(:, 1) = x0;
    for k = 1:N
        x(:,k+1) = sys.A*x(:,k) + sys.B*u(:,k) + sys.E_d*d(:,k) + sys.E_f*f(:,k);
        y(:,k) = sys.C*x(:,k) + sys.D*u(:,k) + sys.G_d*d(:,k) + sys.G_f*f(:,k);
    end
    x = x(:, 1:N); % removing last state for aligned vectors

end