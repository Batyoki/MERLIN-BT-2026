function u = mpc_controller(state, reference, params, A, B, C, dt)
% MPC_CONTROLLER Simple Model Predictive Controller
%
% Inputs:
%   state - Current system state
%   reference - Desired reference
%   params - [prediction_horizon, Q_weight, R_weight]
%   A, B, C - System matrices
%   dt - Sampling time
%
% Output:
%   u - Control input

    % Extract parameters
    N = round(params(1));  % Prediction horizon (steps)
    Q_weight = params(2);   % State tracking weight
    R_weight = params(3);   % Control effort weight
    
    % Discretize system
    sys_c = ss(A, B, C, 0);
    sys_d = c2d(sys_c, dt);
    Ad = sys_d.A;
    Bd = sys_d.B;
    Cd = sys_d.C;
    
    % Dimensions
    nx = size(Ad, 1);  % Number of states
    nu = size(Bd, 2);  % Number of inputs
    
    % Build MPC matrices
    % State prediction: X = Phi*x0 + Psi*U
    Phi = zeros(nx * N, nx);
    Psi = zeros(nx * N, nu * N);
    
    for k = 1:N
        % Phi block
        Phi((k-1)*nx+1:k*nx, :) = Ad^k;
        
        % Psi blocks
        for j = 1:k
            Psi((k-1)*nx+1:k*nx, (j-1)*nu+1:j*nu) = Ad^(k-j) * Bd;
        end
    end
    
    % Output prediction: Y = Gamma*x0 + Theta*U
    Gamma = zeros(N, nx);
    Theta = zeros(N, nu * N);
    for k = 1:N
        Gamma(k, :) = Cd * Ad^k;
        for j = 1:k
            Theta(k, (j-1)*nu+1:j*nu) = Cd * Ad^(k-j) * Bd;
        end
    end
    
    % Reference trajectory (constant for simplicity)
    Ref = reference * ones(N, 1);
    
    % Cost matrices
    Q_mpc = Q_weight * eye(N);
    R_mpc = R_weight * eye(nu * N);
    
    % Quadratic programming: min 0.5*U'*H*U + f'*U
    H = Theta' * Q_mpc * Theta + R_mpc;
    f = Theta' * Q_mpc * (Gamma * state - Ref);
    
    % Ensure H is positive definite
    H = (H + H') / 2 + 1e-6 * eye(size(H));
    
    % Solve QP (unconstrained for simplicity)
    try
        U_opt = -H \ f;
    catch
        % If solve fails, use zero control
        U_opt = zeros(nu * N, 1);
    end
    
    % Extract first control action
    u = U_opt(1);
end
