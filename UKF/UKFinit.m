function [ Mean, Cov, Q, R ] = UKFinit( InitX, InitCov, ProcessNoise_Sigma, MeasurementNoise_Sigma, T )
%EKFINIT Summary of this function goes here
%   Detailed explanation goes here
Q = ProcessNoise_Sigma * ...
    [1/3*T^3, 0,       1/2*T^2, 0;
     0,       1/3*T^3, 0,       1/2*T^2;
     1/2*T^2, 0,       T,       0;
     0,       1/2*T^2, 0,       T];
R = diag(MeasurementNoise_Sigma) .^ 2;
Mean = InitX;
Cov = InitCov;

end

