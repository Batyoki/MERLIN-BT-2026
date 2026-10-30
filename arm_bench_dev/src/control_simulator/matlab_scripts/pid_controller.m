function [u, state] = pid_controller(error, dt, params, prev_state)
% PID_CONTROLLER Standard PID controller
%
% Inputs:
%   error - Current tracking error
%   dt - Sampling time
%   params - [Kp, Ki, Kd] gains
%   prev_state - Previous controller state [integral, prev_error]
%
% Outputs:
%   u - Control input
%   state - Updated controller state

    % Extract parameters
    Kp = params(1);
    Ki = params(2);
    Kd = params(3);
    
    % Initialize state if needed
    if isempty(prev_state)
        integral = 0;
        prev_error = 0;
    else
        integral = prev_state(1);
        prev_error = prev_state(2);
    end
    
    % Proportional term
    P = Kp * error;
    
    % Integral term (with anti-windup)
    integral = integral + error * dt;
    % Anti-windup: limit integral
    integral_max = 50.0;
    integral = max(min(integral, integral_max), -integral_max);
    I = Ki * integral;
    
    % Derivative term (with filtering)
    if dt > 0
        derivative = (error - prev_error) / dt;
    else
        derivative = 0;
    end
    D = Kd * derivative;
    
    % Control output
    u = P + I + D;
    
    % Update state
    state = [integral; error];
end
