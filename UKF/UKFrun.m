function [ Mean_Update, Cov_Update ] = UKFrun( Mean, Cov, Observation, Q, R, T )
%EFKSIMULATE Summary of this function goes here
%   Detailed explanation goes here
F = [1, 0, T, 0;
     0, 1, 0, T;
     0, 0, 1, 0;
     0, 0, 0, 1];

% Parameters for UKF
n = size(Mean, 1);
Alpha = 0.25;
K = 1;
Beta = 4;
Lambda = Alpha^2*(n+K)-n;
W_Mean = [Lambda/(n+Lambda), repmat(1/(2*(n+Lambda)), [1, 2*n])];
W_Cov = [Lambda/(n+Lambda)+(1-Alpha^2+Beta), repmat(1/(2*(n+Lambda)), [1, 2*n])];
C = sqrt(Lambda+n);
% Predict
Mean_Predict = F * Mean;
Cov_Predict = F*Cov*F' + Q;
% Update
SPs = Find_SigmaPoints(Mean_Predict, Cov_Predict, C );
NumOfSP = size(SPs, 2);
SigmaPoints_Obs = zeros(2, NumOfSP);
for i = 1:NumOfSP
    SigmaPoints_Obs(:, i) = h(SPs(:, i));
end

Weighted_x = SPs * diag(W_Mean);
Mu_x = sum(Weighted_x, 2);
Mu_x_subtract = SPs - repmat(Mu_x, [1, NumOfSP]);
P_xx = Mu_x_subtract * diag(W_Cov) * Mu_x_subtract';
Weighted_y = SigmaPoints_Obs * diag(W_Mean);
Mu_y = sum(Weighted_y, 2);
Mu_y_subtract = SigmaPoints_Obs - repmat(Mu_y, [1, NumOfSP]);
P_yy = Mu_y_subtract * diag(W_Cov) * Mu_y_subtract' + R;
P_xy = Mu_x_subtract * diag(W_Cov) * Mu_y_subtract';
K = P_xy*inv(P_yy);
Mean_Update = Mu_x + K*(Observation - h(Mu_x));
Cov_Update = P_xx-P_xy*inv(P_yy)*P_xy';

end

function [XinPolar] = h(XinCartesian)
    x = XinCartesian(1);
    y = XinCartesian(2);
    XinPolar = [sqrt(x^2+y^2); atan2(y, x)];
end

function [SPs] = Find_SigmaPoints(XinCartesian, Cov, C)
    % Using eigenvalue decomposition
NumOfDim = size(XinCartesian, 1);
NumOfPoints = NumOfDim*2;
[V, D] = eig(Cov);
pnts = zeros(NumOfDim, NumOfPoints);
for i = 1:NumOfDim
    pnts(i, i) = sqrt(D(i, i));
    pnts(i, i+NumOfDim) = -sqrt(D(i, i));
end
%scale samples such that they have the same second order stats as a standard normal
Covariance_pnts = C*pnts;
Rotated_pnts = V*Covariance_pnts;
SigmaPoints = repmat(XinCartesian, [1, NumOfPoints]) + Rotated_pnts;
SPs = [XinCartesian, SigmaPoints];
end
