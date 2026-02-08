Ts = 0.2;
T_sim = 24;
N = T_sim / Ts;
t = 0:Ts:T_sim-Ts;

Kp = 0.5;
Ki = 0.32;
PWM_MAX = 1000;

current_lux = 0;
error_sum = 0;
pwm_val = 0;

history_lux = zeros(1, N);
history_pwm = zeros(1, N);
history_setpoint = zeros(1, N);

for k = 1:N
    current_time = k * Ts;
    
    if current_time < 4
        setpoint = 100;
    elseif current_time < 8
        setpoint = 200;
    elseif current_time < 12
        setpoint = 400;
    elseif current_time < 16
        setpoint = 100;
    elseif current_time < 20
        setpoint = 500;
    else
        setpoint = 50;
    end
    
    history_setpoint(k) = setpoint;

    process_gain = 2.0; 
    inertia = 0.5; 
    
    current_lux = (current_lux * inertia) + (pwm_val * process_gain * (1 - inertia));
    history_lux(k) = current_lux;
    
    error = setpoint - current_lux;
    
    P = Kp * error;
    
    error_sum = error_sum + error;
    integral_limit = PWM_MAX / Ki;
    
    if error_sum > integral_limit
        error_sum = integral_limit;
    elseif error_sum < -integral_limit
        error_sum = -integral_limit;
    end
    
    I = Ki * error_sum;
    
    output = P + I;
    
    if output > PWM_MAX
        output = PWM_MAX;
    elseif output < 0
        output = 0;
    end
    
    pwm_val = output;
    history_pwm(k) = pwm_val;
end

figure;
subplot(2,1,1);
plot(t, history_setpoint, 'r--', 'LineWidth', 1.5); hold on;
plot(t, history_lux, 'b-', 'LineWidth', 2);
title('Semi-random step response');
xlabel('Time elapsed [s]');
ylabel('Brightness [Lux]');
legend('AIM', 'ACT');
grid on;
ylim([-50 700]);

subplot(2,1,2);
plot(t, history_pwm, 'g-', 'LineWidth', 1.5);
title('PWM');
xlabel('Time elapsed [s]');
ylabel('PWM strength 0-1000');
grid on;