function [A, B, C, D, E_d, E_f, G_d, G_f, sys] = generateShip(Ts, eta1, b, n_phi)
% Generates the discrete-time system corresponding to the ship problem of
% the course.
% Ts is the sampling time for the CT -> DT conversion (using zoh)
% n_phi is the number of sensors for the state phi
%
% sys is a struct for easy sharing of the system through functions
% Written by E. Colot on March 16, 2026

%% Continuous time system construction    
    
    A = [b*eta1 0;
            1 0];
    B = [b;
         0];
    E_d = [0;
           1];
    E_f = zeros(2, n_phi+1);
    
    C = [repmat([1 0], n_phi, 1);
          0 1];
    D = [zeros(n_phi, 1);
         0];
    G_d = [ones(n_phi, 1);
           0];
    G_f = eye(n_phi+1);
    
    sysC = ss(A, [B, E_d, E_f], C, [D, G_d, G_f]);
    sysD = c2d(sysC, Ts, 'zoh');
    
    %% Discrete-time system
    
    A = sysD.A;
    B_ = sysD.B;
    C = sysD.C;
    D_ = sysD.D;
    
    B = B_(:, 1);
    E_d = B_(:, 2);
    E_f = B_(:, 3:end);
    
    D = D_(:, 1);
    G_d = D_(:, 2);
    G_f = D_(:, 3:end);

%% sys construction
    sys = struct();
    sys.A = A;
    sys.B = B;
    sys.C = C;
    sys.D = D;
    sys.E_d = E_d;
    sys.E_f = E_f;
    sys.G_d = G_d;
    sys.G_f = G_f;

end