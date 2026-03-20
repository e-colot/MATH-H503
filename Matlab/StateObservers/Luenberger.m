function x_est = Luenberger(A, B, C, D, y, u, poles, x0)
    % Estimates the state using a Luenberger estimator
    % time is the second dimension for y, u and x_est

    N = size(u, 2);
    L = place(A, C, poles);
    x_est = repmat(x0, 1, N+1);

    for k = 1:N

        x_est(:,k+1) = A*x_est(:,k) + B*u(:,k) + L*(y(:,k) - C*x_est(:,k) - D*u(:,k));

    end

end
