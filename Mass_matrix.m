function [xMs] = Mass_matrix(x)
    global J1 J2 J3 m1 m2 m3 l1 l2 l3

    % Extract joint angles and their derivatives from input vector x
    q1 = x(1);
    q2 = x(2);
    q3 = x(3);
    dq1 = x(4);
    dq2 = x(5);
    dq3 = x(6);

    % Compute elements of the mass matrix
    xMc=[J2+J3+m2*l1*l2*cos(q2)+2*m3*l1*l2*cos(q2)+m3*l2*l3*cos(q3)+m3*l1*l3*cos(q2+q3)+J1+(1/4)*m1*l1^2+m3*l2^2+(1/4)*m3*l3^2+m2*l1^2+m3*l1^2+(1/4)*m2*l2^2, J2+J3+(1/2)*m2*l1*l2*cos(q2)+m3*l1*l2*cos(q2)+m3*l2*l3*cos(q3)+(1/2)*m3*l1*l3*cos(q2+q3)+m3*l2^2+(1/4)*m3*l3^2+(1/4)*m2*l2^2, J3+(1/2)*m3*l2*l3*cos(q3)+(1/2)*m3*l1*l3*cos(q2+q3)+(1/4)*m3*l3^2
    J2+J3+(1/2)*m2*l1*l2*cos(q2)+m3*l1*l2*cos(q2)+m3*l2*l3*cos(q3)+(1/2)*m3*l1*l3*cos(q2+q3)+m3*l2^2+(1/4)*m3*l3^2+(1/4)*m2*l2^2,m3*l2*l3*cos(q3)+(1/4)*m2*l2^2+m3*l2^2+(1/4)*m3*l3^2+J2+J3, (1/2)*m3*l2*l3*cos(q3)+(1/4)*m3*l3^2+J3
    J3+(1/2)*m3*l2*l3*cos(q3)+(1/2)*m3*l1*l3*cos(q2+q3)+(1/4)*m3*l3^2,(1/2)*m3*l2*l3*cos(q3)+(1/4)*m3*l3^2+J3,J3+(1/4)*m3*l3^2];


    % Assemble the mass matrix
    xMs = [eye(3), zeros(3); zeros(3), xMc];
end
