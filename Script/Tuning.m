T = readtable("log.csv");           % columns: t_ms, lux, target, duty
t = T.t_ms/1000;                    % sec
y = T.lux;
u = T.duty/100;                     % 0..1

% Estimate step size in u (use initial and final average)
u0 = mean(u(1:5));
u1 = mean(u(end-4:end));
du = u1 - u0;

y0 = mean(y(1:5));
y1 = mean(y(end-4:end));
dy = y1 - y0;

K = dy / du;                        % plant gain (lux per duty)

% crude FOPDT estimation using 35%-85% method
y35 = y0 + 0.35*dy;
y85 = y0 + 0.85*dy;

t35 = interp1(y, t, y35, "linear", "extrap");
t85 = interp1(y, t, y85, "linear", "extrap");

tau = 1.5*(t85 - t35);
theta = t35 - 0.29*tau;
theta = max(theta, 0);              % no negative delay

% SIMC PI tuning
lambda = max(theta, 0.1*tau);       % tuning aggressiveness
Kp = (1/K) * (tau/(theta + lambda));
Ti = min(tau, 4*(theta + lambda));
Ki = Kp / Ti;

fprintf("Estimated FOPDT: K=%.3f, tau=%.3f s, theta=%.3f s\n", K, tau, theta);
fprintf("PI gains: Kp=%.4f, Ki=%.4f (1/s)\n", Kp, Ki);

% Plot
figure; plot(t,y); grid on;
xlabel("Time (s)"); ylabel("Lux"); title("Step response (measured)");
