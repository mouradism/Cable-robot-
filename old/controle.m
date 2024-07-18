function [Tau, F] = controle(t, x)
    global l1 l2 l3 
    global Lb r La t_ E_ TT
    global lj XL

    Dt = t - t_;

    % Ensure angles are between 0 and 2*pi
    for i = 1:3
        x(i) = atan2(sin(x(i)), cos(x(i)));
        if x(i) < 0
            x(i) = x(i) + 2*pi;
        end
    end

    q = x(1:3)';
    dq = x(4:6)';

    % Get the cables parameters
    [X, Jk, dt_Jk, S_] = cables(lj, XL, q, dq);

    % Trajectory planning
    [Xd, V, A] = planning(t, lj, q, dq, X, Dt);

    % Control gains
    Kp = 750;
    Kv = 2 * sqrt(Kp);

    % Error calculation
    delt_X = Xd - X;    
    delt_X_t = V - Jk * dq;
    Xd_tt = A;

    % Dynamic parameters
    [xMc, vFc] = dyn_0(x);

    % Integral error update
    if Dt ~= 0
        E_ = E_ + delt_X;
    end

    ion = 0;

    % Desired acceleration
    X_tt = Xd_tt + Kv * delt_X_t + Kp * delt_X + ion * 0.01 * 10^4 * E_ * Dt;

    % Control torque
    tau = xMc * (pinv(Jk) * (X_tt - dt_Jk * dq)) + vFc;

    % Force calculation
    Taw = Jk' * S_;
    pinv_Taw = pinv(Taw) * tau;
    null_Taw = null(Taw);

    % Force saturation
    Tmax = 500;
    T = pinv_Taw;
    c = 1;

    while any(T < 0) && all(T < Tmax)
        T = T + c * null_Taw;
        if all(null_Taw <= 0)
            c = -1;
        else
            c = 1;
        end
    end

    % Output torque
    Tau = r * T;
    F = 0;

    % Saturation limits
    maxtau = inf + 400 * r;
    for i = 1:4
        if abs(Tau(i)) > maxtau
            Tau(i) = sign(Tau(i)) * maxtau;
        end
    end

    % Update time history
    if t > t_
        T = [t; T; X(1:2, 1); X(3:4, 1)];
        TT = [TT, T];
    end

    t_ = t;
end

function [xMc, vFc] = dyn_0(x)
    global J1 J2 J3 m1 m2 m3 l1 l2 l3 g
    q1 = x(1);
    q2 = x(2);
    q3 = x(3);
    dq1 = x(4);
    dq2 = x(5);
    dq3 = x(6);

    % Inertia matrix
    xMc = [J2 + J3 + m2 * l1 * l2 * cos(q2) + 2 * m3 * l1 * l2 * cos(q2) + m3 * l2 * l3 * cos(q3) + m3 * l1 * l3 * cos(q2 + q3) + J1 + (1/4) * m1 * l1^2 + m3 * l2^2 + (1/4) * m3 * l3^2 + m2 * l1^2 + m3 * l1^2 + (1/4) * m2 * l2^2, J2 + J3 + (1/2) * m2 * l1 * l2 * cos(q2) + m3 * l1 * l2 * cos(q2) + m3 * l2 * l3 * cos(q3) + (1/2) * m3 * l1 * l3 * cos(q2 + q3) + m3 * l2^2 + (1/4) * m3 * l3^2 + (1/4) * m2 * l2^2, J3 + (1/2) * m3 * l2 * l3 * cos(q3) + (1/2) * m3 * l1 * l3 * cos(q2 + q3) + (1/4) * m3 * l3^2;
            J2 + J3 + (1/2) * m2 * l1 * l2 * cos(q2) + m3 * l1 * l2 * cos(q2) + m3 * l2 * l3 * cos(q3) + (1/2) * m3 * l1 * l3 * cos(q2 + q3) + m3 * l2^2 + (1/4) * m3 * l3^2 + (1/4) * m2 * l2^2, m3 * l2 * l3 * cos(q3) + (1/4) * m2 * l2^2 + m3 * l2^2 + (1/4) * m3 * l3^2 + J2 + J3, (1/2) * m3 * l2 * l3 * cos(q3) + (1/4) * m3 * l3^2 + J3;
            J3 + (1/2) * m3 * l2 * l3 * cos(q3) + (1/2) * m3 * l1 * l3 * cos(q2 + q3) + (1/4) * m3 * l3^2, (1/2) * m3 * l2 * l3 * cos(q3) + (1/4) * m3 * l3^2 + J3, J3 + (1/4) * m3 * l3^2];

    % Coriolis and gravity forces
    vFc = [-(1/2) * m2 * l1 * l2 * dq2^2 * sin(q2) - m3 * l1 * l2 * dq2^2 * sin(q2) - (1/2) * m3 * l1 * l3 * dq2^2 * sin(q2 + q3) - m2 * l1 * dq1 * l2 * sin(q2) * dq2 - 2 * m3 * l1 * dq1 * l2 * sin(q2) * dq2 - m3 * l1 * dq1 * l3 * sin(q2 + q3) * dq2 - m3 * l1 * l3 * dq3 * sin(q2 + q3) * dq2 - m3 * l2 * dq2 * l3 * sin(q3) * dq3 - m3 * l1 * dq1 * l3 * sin(q2 + q3) * dq3 - m3 * l2 * dq1 * l3 * sin(q3) * dq3 - (1/2) * m3 * l1 * l3 * dq3^2 * sin(q2 + q3) - (1/2) * m3 * l2 * l3 * dq3^2 * sin(q3) + (1/2) * m3 * g * l3 * cos(q1 + q2 + q3) + (1/2) * m1 * g * l1 * cos(q1) + m2 * g * l1 * cos(q1) + m3 * g * l1 * cos(q1) + (1/2) * m2 * g * l2 * cos(q1 + q2) + m3 * g * l2 * cos(q1 + q2);
            -m3 * l2 * dq2 * l3 * sin(q3) * dq3 + (1/2) * sin(q2) * dq1^2 * l1 * l2 * m2 + sin(q2) * dq1^2 * l1 * l2 * m3 + (1/2) * dq1^2 * sin(q2 + q3) * l1 * l3 * m3 - m3 * l2 * dq1 * l3 * sin(q3) * dq3 - (1/2) * m3 * l2 * l3 * dq3^2 * sin(q3) + (1/2) * m3 * g * l3 * cos(q1 + q2 + q3) + (1/2) * m2 * g * l2 * cos(q1 + q2) + m3 * g * l2 * cos(q1 + q2);     
            (1/2) * m3 * l3 * (dq2^2 * sin(q3) * l2 + 2 * dq2 * dq1 * sin(q3) * l2 + dq1^2 * sin(q2 + q3) * l1 + dq1^2 * sin(q3) * l2 + g * cos(q1 + q2 + q3))];

    % If necessary, the code for the external forces and the friction can be added here.
end

function [Xd, Vd, Ad] = planning(t, lj, q, dq, X, Dt)
    global l1 l2 l3 ivp alpha Xi Xf

    % Calculate segment lengths for each joint
    [l11, l12, l13] = calculateSegments(lj(1), l1, l2, l3);
    [l21, l22, l23] = calculateSegments(lj(2), l1, l2, l3);
    [l31, l32, l33] = calculateSegments(lj(3), l1, l2, l3);
    [l41, l42, l43] = calculateSegments(lj(4), l1, l2, l3);

    % Trajectory planning
    Vi = [0; 0]; 
    Vf = [0; 0]; 
    tf = 1.0;

    [a, b, c, d] = computeTrajectoryCoefficients(Xi, Xf, Vi, Vf, tf);

    if t <= tf
        [Xc, Vc, Ac] = evaluateTrajectory(a, b, c, d, t);
    else
        [Xc, Vc, Ac] = evaluateTrajectory(a, b, c, d, tf);
    end 

    X3d = Xc;
    X2d = Xc + l3 * [cos(alpha); sin(alpha)];

    V3 = Vc;
    V2 = Vc;
    A3 = Ac;   
    A2 = Ac;
 
    % Inverse kinematics
    qc = invKIN(Xc + l3 * [cos(alpha); sin(alpha)], Xc);

    % Jacobians and their time derivatives
    dqc = computeJointVelocities(l1, l2, l3, qc, Vc);
    ddqc = computeJointAccelerations(l1, l2, l3, qc, dqc, Ac);

    % Compute positions, velocities, and accelerations
    [Jk, dt_Jk, Xd] = computeJacobians(l11, l12, l13, l21, l22, l23, l31, l32, l33, l41, l42, l43, qc, dqc);
    
    Vd = Jk * dqc;
    Ad = Jk * ddqc + dt_Jk * dqc;
end

function [l1_seg, l2_seg, l3_seg] = calculateSegments(lj, l1, l2, l3)
    % Calculates segment lengths based on joint positions
    if lj <= l1
        l1_seg = lj;
        l2_seg = 0;
        l3_seg = 0;
    elseif lj <= l1 + l2
        l1_seg = l1;
        l2_seg = lj - l1;
        l3_seg = 0;
    elseif lj <= l1 + l2 + l3
        l1_seg = l1;
        l2_seg = l2;
        l3_seg = lj - (l1 + l2);
    else 
        l1_seg = 0;
        l2_seg = 0;
        l3_seg = 0;
    end
end

function [a, b, c, d] = computeTrajectoryCoefficients(Xi, Xf, Vi, Vf, tf)
    % Computes trajectory coefficients for cubic polynomial
    a = -(2 / tf^3) * (Xf - Xi) + (1 / tf^2) * (Vf + Vi);
    b = (3 / tf^2) * (Xf - Xi) - (2 / tf) * Vi - (1 / tf) * Vf;
    c = Vi;
    d = Xi;
end

function [Xc, Vc, Ac] = evaluateTrajectory(a, b, c, d, t)
    % Evaluates the trajectory at time t
    Xc = a * t^3 + b * t^2 + c * t + d;
    Vc = 3 * a * t^2 + 2 * b * t + c;
    Ac = 6 * a * t + 2 * b;
end

function dqc = computeJointVelocities(l1, l2, l3, qc, Vc)
    % Computes joint velocities using Jacobian pseudoinverse
    [Jkc3, ~, ~] = jak(l1, l2, l3, qc, [0; 0; 0]);
    [Jkc2, ~, ~] = jak(l1, l2, 0, qc, [0; 0; 0]);
    dqc = (0.5 * pinv(Jkc3) + 0.5 * pinv(Jkc2)) * Vc;
end

function ddqc = computeJointAccelerations(l1, l2, l3, qc, dqc, Ac)
    % Computes joint accelerations using Jacobian and its time derivative
    [Jkc3, dt_Jkc3, ~] = jak(l1, l2, l3, qc, dqc);
    [Jkc2, dt_Jkc2, ~] = jak(l1, l2, 0, qc, dqc);
    ddqc = (0.5 * pinv(Jkc3) + 0.5 * pinv(Jkc2)) * (Ac - (0.5 * dt_Jkc3 + 0.5 * dt_Jkc2) * qc);
end

function [Jk, dt_Jk, Xd] = computeJacobians(l11, l12, l13, l21, l22, l23, l31, l32, l33, l41, l42, l43, qc, dqc)
    % Computes Jacobians and their time derivatives
    [Jk1, dt_Jk1, X1] = jak(l11, l12, l13, qc, dqc);
    [Jk2, dt_Jk2, X2] = jak(l21, l22, l23, qc, dqc);
    [Jk3, dt_Jk3, X3] = jak(l31, l32, l33, qc, dqc);
    [Jk4, dt_Jk4, X4] = jak(l41, l42, l43, qc, dqc);

    Jk = [Jk1; Jk2; Jk3; Jk4];
    dt_Jk = [dt_Jk1; dt_Jk2; dt_Jk3; dt_Jk4];
    Xd = [X1; X2; X3; X4];
end
