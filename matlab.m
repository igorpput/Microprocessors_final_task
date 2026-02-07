Ts = 0.2;

% ---- load your logged data ----
% columns: t_ms, u, lux
D = readmatrix("log.csv");
t = D(:,1)/1000;
u = D(:,2);
y = D(:,3);

% remove initial offset
y0 = mean(y(1:10));
y = y - y0;

% estimate K
du = mean(u(end-10:end)) - mean(u(1:10));
dy = mean(y(end-10:end)) - mean(y(1:10));
K = dy/du;

% estimate tau using 63% method
y63 = 0.632*dy;
idx = find(y >= y63, 1, 'first');
tau = t(idx) - t(find(u > u(1)+0.01, 1, 'first')); % rough

% plant model
G = tf(K, [tau 1]);

% ---- PI tuning (IMC style) ----
lambda = tau;          % try tau, then 0.5*tau if you want faster
Kp = tau/(K*lambda);
Ki = 1/lambda;

C = pid(Kp, Ki);       % PI
Tcl = feedback(C*G, 1);

step(Tcl); grid on
title(sprintf("Closed-loop: Kp=%.3f Ki=%.3f", Kp, Ki));
