clear; close all; clc;

x_0 =[0;10];
P_0 =[2,0;0,3];
state_history = x_0;
A = [1,1;0,1];
G = [0;1];
H = [1,0];
Q=.1;
R=2;
step_number = 100;
%state generation
for i=1:step_number
    update = A*state_history(:,end)+G*mvnrnd(0,Q);
    disp(update)
    state_history=[state_history,update];
end
%measure generation
measure_hist  = 0;
for i=1:step_number
    update = H*state_history(:,i+1)+mvnrnd(0,R);
    measure_hist=[measure_hist,update];
end
vec_time_updates=x_0;
vec_measurement_updates=x_0;
cov_time_updates = {P_0};
cov_measurement_updates={P_0};
for i=1:step_number
    %Time update
    error_cov = A*cov_measurement_updates{end}*A'+G*Q*G';
    est = A*vec_measurement_updates(:,end);
    vec_time_updates=[vec_time_updates,est];
    cov_time_updates=[cov_time_updates,error_cov];
    
    %Measure update
    error_cov = inv(inv(cov_time_updates{end})+H'*inv(R)*H);
    est = vec_time_updates(:,end)+error_cov*H'*inv(R)*(measure_hist(i+1)-H*vec_time_updates(:,end));
    vec_measurement_updates=[vec_measurement_updates,est];
    cov_measurement_updates=[cov_measurement_updates,error_cov];
end
% Plotting 

%velocity
true_velocity = state_history(2, :); 
estimated_velocity = vec_measurement_updates(2, :);
time_steps = 0:length(true_velocity)-1;
naive_velocity = 10*ones(1,length(time_steps));
% Plotting
figure;
plot(time_steps, true_velocity, 'b-', 'LineWidth', 2); % True velocity
hold on;
plot(time_steps, estimated_velocity, 'r-', 'LineWidth', 2); % Estimated velocity
hold on;
plot(time_steps, naive_velocity, 'g-', 'LineWidth', 2);% naive velocity
xlabel('Time Step');
ylabel('Velocity');
legend('True Velocity', 'Estimated Velocity','Naive Velocity');
title('True vs. Estimated Velocity over Time');
grid on;
saveas(gcf,'velocity.png')

%position
true_pos = state_history(1, :); 
estimated_pos = vec_measurement_updates(1, :);
time_steps = 0:length(true_pos)-1;
naive_pos = 10*time_steps;
% Plotting
figure;
plot(time_steps, true_pos, 'b-', 'LineWidth', 2); % True velocity
hold on;
plot(time_steps, estimated_pos, 'r--', 'LineWidth', 2); % Estimated velocity
hold on;
plot(time_steps, naive_pos, 'g-', 'LineWidth', 2);% naive velocity
hold on;
plot(time_steps, measure_hist, 'k:', 'LineWidth', 2);% measurements
xlabel('Time Step');
ylabel('Position');
legend('True Position', 'Estimated Position','Naive Position','Measured Position','Location','northwest');
title('True vs. Estimated Position over Time');
grid on;
saveas(gcf,'position.png')

%Error graphs
measure_error = abs(measure_hist-true_pos);
est_error = abs(estimated_pos-true_pos);
naive_error = abs(naive_pos-true_pos);
% Plotting
figure;
plot(time_steps, measure_error, 'k-', 'LineWidth', 2);% measurements error
hold on;
plot(time_steps, est_error, 'r-', 'LineWidth', 2); % Estimated error
hold on;
plot(time_steps, naive_error, 'g-', 'LineWidth', 2);% naive error
xlabel('Time Step');
ylabel('Position');
legend('Measure Error','Estimation Error','Naive Error','Location','northwest');
title('Position Error over Time');
ylim([0 10])
grid on;
saveas(gcf,'position_error.png')


vel_est_error = abs(estimated_velocity-true_velocity);
vel_naive_error = abs(naive_velocity-true_velocity);
% Plotting
figure;
plot(time_steps, vel_est_error, 'r-', 'LineWidth', 2); % Estimated error
hold on;
plot(time_steps, vel_naive_error, 'g-', 'LineWidth', 2);% naive error
xlabel('Time Step');
ylabel('Position');
legend('Estimation Error','Naive Error','Location','northwest');
title('Velocity Error over Time');
grid on;
saveas(gcf,'velocity_error.png')