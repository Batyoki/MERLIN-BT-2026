% PD law with spring feed-forward: the plant has a spring k = 2 N/m, so holding a
% position r needs u = k*r. The feed-forward removes the steady-state error without an integral.

Kp = 6.0;   % @param min=0 max=50 desc="proportional gain"
Kd = 1.2;   # @param min=0 max=20 desc="derivative gain (the # marker is accepted too)"
% @param Kff default=2.0 min=0 max=5 desc="spring feed-forward, ideally k = 2"

u = Kp*e + Kd*derivative + Kff*r;
