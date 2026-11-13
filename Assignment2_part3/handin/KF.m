
function [x_pst,P_pst,x_prd,P_prd] = KF( ...
    x_prd,P_prd,Ad,Bd,Ed,Cd,Qd,Rd,psi_meas,delta)

    % Innovation
    innovation = ssa(psi_meas - Cd*x_prd);

    % Innovation covariance
    S = Cd*P_prd*Cd' + Rd;

    % Kalman gain
    L = P_prd*Cd'/S;

    % Measurement update
    x_pst = x_prd + L*innovation;

    % Posterior covariance (Joseph form)
    I = eye(3);
    P_pst = (I-L*Cd)*P_prd*(I-L*Cd)' + L*Rd*L';

    % Time update
    x_prd = Ad*x_pst + Bd*delta;
    P_prd = Ad*P_pst*Ad' + Qd;

end
