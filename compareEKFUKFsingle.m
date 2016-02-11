clear all
close all

ProcessNoise_SigmaSquare = 1e-4;
MeasurementNoise_Sigma = [1e-3; 2e-2];  % Range; Bearings

% Plot
figure(1);
hold on;
% axis([1500, 8500, 1500, 8500]);

% Data Generation
DataGenerate_GenerateTrajectory
DataGenerate_AddingMeasurementNoise

% Initialisation
store_ekf_errors = zeros(1, size(Observations, 2));
store_ukf_errors = zeros(1, size(Observations, 2));

[ Mean_ekf, Cov_ekf, Q, R ] = EKFinit( [1980; 2020; 0; 0], diag([10; 10; 4; 4]), ProcessNoise_SigmaSquare, MeasurementNoise_Sigma, T );
store_ekf_errors(1, 1) = norm(Mean_ekf(1:2, 1)-store_Target_real_state(1:2, 1));
[ Mean_ukf, Cov_ukf ] = UKFinit( [1980; 2020; 0; 0], diag([10; 10; 4; 4]), ProcessNoise_SigmaSquare, MeasurementNoise_Sigma, T );
store_ukf_errors(1, 1) = norm(Mean_ukf(1:2, 1)-store_Target_real_state(1:2, 1));
plot(Mean_ekf(1), Mean_ekf(2), 'r.');
plot(Mean_ukf(1), Mean_ukf(2), 'b.');
plot(store_Target_real_state(1, 1), store_Target_real_state(2, 1), 'g.');
legend('ekf', 'ukf', 'true');
% Data Processing
for i = 2:size(Observations, 2)
    [ Mean_ekf, Cov_ekf ] = EKFrun( Mean_ekf, Cov_ekf, Observations(:, i), Q, R, T );
    store_ekf_errors(1, i) = norm(Mean_ekf(1:2, 1)-store_Target_real_state(1:2, i));
    [ Mean_ukf, Cov_ukf ] = UKFrun( Mean_ukf, Cov_ukf, Observations(:, i), Q, R, T );
    store_ukf_errors(1, i) = norm(Mean_ukf(1:2, 1)-store_Target_real_state(1:2, i));
    plot(Mean_ekf(1), Mean_ekf(2), 'r.');
    plot(Mean_ukf(1), Mean_ukf(2), 'b.');
    plot(store_Target_real_state(1, i), store_Target_real_state(2, i), 'g.');
end

figure(2);
hold on;
xlabel('Timestep');
ylabel('Euclidean Error');
plot(store_ekf_errors, 'r-');
plot(store_ukf_errors, 'b-');
legend('EKF', 'UKF');
