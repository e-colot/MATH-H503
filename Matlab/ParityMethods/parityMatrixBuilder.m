function [Theta, T_u, T_d, T_f, Y_s, U_s] = parityMatrixBuilder(s, u, y, N, sys)
% generates the different matrices needed for parity-based fault detection
% and isolation
% Written by E. Colot on March 17 2026

% Matrix building

    % Theta
    Theta = repmat(sys.C, s+1, 1);
    for row = 1:s
        Theta(1+row*size(sys.C,1):(1+row)*size(sys.C,1),:) = sys.C*sys.A^row;
    end

    % T^u
    T_u = zeros((s+1)*size(sys.D,1), (s+1)*size(sys.D,2));
    for row = 0:s
        for col = 0:s
            if col == row
                tmp = sys.D;
            elseif row < col
                tmp = zeros(size(sys.D));
            else
                tmp = sys.C*sys.A^(row-col-1)*sys.B;
            end
            T_u(1+row*size(sys.D,1):(row+1)*size(sys.D,1), 1+col*size(sys.D,2):(col+1)*size(sys.D,2)) = tmp;
        end
    end

    % T^d
    T_d = zeros((s+1)*size(sys.G_d,1), (s+1)*size(sys.G_d,2));
    for row = 0:s
        for col = 0:s
            if col == row
                tmp = sys.G_d;
            elseif row < col
                tmp = zeros(size(sys.G_d));
            else
                tmp = sys.C*sys.A^(row-col-1)*sys.E_d;
            end
            T_d(1+row*size(sys.G_d,1):(row+1)*size(sys.G_d,1), 1+col*size(sys.G_d,2):(col+1)*size(sys.G_d,2)) = tmp;
        end
    end

    % T^f
    T_f = zeros((s+1)*size(sys.G_f,1), (s+1)*size(sys.G_f,2));
    for row = 0:s
        for col = 0:s
            if col == row
                tmp = sys.G_f;
            elseif row < col
                tmp = zeros(size(sys.G_f));
            else
                tmp = sys.C*sys.A^(row-col-1)*sys.E_f;
            end
            T_f(1+row*size(sys.G_f,1):(row+1)*size(sys.G_f,1), 1+col*size(sys.G_f,2):(col+1)*size(sys.G_f,2)) = tmp;
        end
    end

% Signal shaping

    % Y_s
    Y_s = zeros((s+1)*size(y,1), N-s); % N-s because y(k-s) is needed, meaning it doesn"t exist for the first few samples
    for row = 0:s
        for col = 1:N-s
            Y_s(1+row*size(y,1):(1+row)*size(y,1), col) = y(:, col+row);
        end
    end

    % U_s
    U_s = zeros((s+1)*size(u,1), N-s); % N-s because y(k-s) is needed, meaning it doesn"t exist for the first few samples
    for row = 0:s
        for col = 1:N-s
            U_s(1+row*size(u,1):(1+row)*size(u,1), col) = u(:, col+row);
        end
    end

end
