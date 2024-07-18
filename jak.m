function [Jk, dt_Jk, X] = jak(l1, l2, l3, q, dq)

% Calculate the Jacobian matrix Jk
Jk = [-l1 * sin(q(1)) - l2 * sin(q(1) + q(2)) - l3 * sin(q(1) + q(2) + q(3)), -l2 * sin(q(1) + q(2)) - l3 * sin(q(1) + q(2) + q(3)), -l3 * sin(q(1) + q(2) + q(3));
       l1 * cos(q(1)) + l2 * cos(q(1) + q(2)) + l3 * cos(q(1) + q(2) + q(3)),  l2 * cos(q(1) + q(2)) + l3 * cos(q(1) + q(2) + q(3)),  l3 * cos(q(1) + q(2) + q(3))];

% Calculate the time derivative of the Jacobian matrix dt_Jk
dt_Jk = [-l1 * cos(q(1)) * dq(1) - l2 * cos(q(1) + q(2)) * (dq(1) + dq(2)) - l3 * cos(q(1) + q(2) + q(3)) * (dq(1) + dq(2) + dq(3)), -l2 * cos(q(1) + q(2)) * (dq(1) + dq(2)) - l3 * cos(q(1) + q(2) + q(3)) * (dq(1) + dq(2) + dq(3)), -l3 * cos(q(1) + q(2) + q(3)) * (dq(1) + dq(2) + dq(3));
         -l1 * sin(q(1)) * dq(1) - l2 * sin(q(1) + q(2)) * (dq(1) + dq(2)) - l3 * sin(q(1) + q(2) + q(3)) * (dq(1) + dq(2) + dq(3)), -l2 * sin(q(1) + q(2)) * (dq(1) + dq(2)) - l3 * sin(q(1) + q(2) + q(3)) * (dq(1) + dq(2) + dq(3)), -l3 * sin(q(1) + q(2) + q(3)) * (dq(1) + dq(2) + dq(3))];

% Calculate the end-effector position X
X = [l1 * cos(q(1)) + l2 * cos(q(1) + q(2)) + l3 * cos(q(1) + q(2) + q(3));
     l1 * sin(q(1)) + l2 * sin(q(1) + q(2)) + l3 * sin(q(1) + q(2) + q(3))];

end
