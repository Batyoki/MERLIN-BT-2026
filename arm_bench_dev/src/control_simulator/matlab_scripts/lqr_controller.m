function u = lqr_controller(state, reference, params, A, B, C)
% LQR_CONTROLLER Linear Quadratic Regulator
%
% Inputs:
%   state - Current system state
%   reference - Desired reference
%   params - [Q_pos, Q_vel, R] - state and control weights
%   A, B, C - System matrices
%
% Output:
%   u - Control input

    % Extract parameters
    Q_pos = params(1);
    Q_vel = params(2);
    R = params(3);
    
    % Build Q matrix (state cost)
    Q = diag([Q_pos, Q_vel]);
    
    % Compute LQR gain
    try
        [K, ~, ~] = lqr(A, B, Q, R);
    catch
        % If LQR fails, use default gain
        K = [1.0, 0.5];
    end
    
    % Compute desired state (reference with zero velocity)
    x_desired = [reference; 0];
    
    % State feedback control
    % u = -K(x - x_desired) = -K*x + K*x_desired
    u = -K * (state - x_desired);
end
