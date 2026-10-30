% PD position controller with an integral term, for the Control Simulator.
%
% Paste this into the "Control law" box (or use "Load script...").
% The plant is the simulator's mass-spring-damper:  m = 1 kg, b = 0.5 N s/m, k = 2 N/m,
% |u| <= 10 N, step 10 ms.  Available variables: e, integral, derivative, x, r, t, dt.

% @param min=0 max=50 desc="proportional gain" unit="N/m"
Kp = 8.0;

% @param min=0 max=10 desc="integral gain" unit="N/(m s)"
Ki = 0.5;

% @param min=0 max=20 desc="derivative gain" unit="N s/m"
Kd = 0.4;

u = Kp*e + Ki*integral + Kd*derivative;
