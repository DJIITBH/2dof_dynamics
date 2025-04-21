function dydt = robot_dynamics(t,y,J,b,L,k,R,V1,V2,m1,m2,l1,l2)
    g = 9.812;

    angle1= y(1);
    angle2 = y(2);
    omega1 = y(3);
    omega2 = y(4);
    acc1 = y(5);
    acc2 = y(6);

    T_l1 = (m1*l1*l1/3 + m2*l2*l2/3 + m2*l1*l1 + m2*l1*l2*cos(angle2))*acc1 + (m2*l2*l2/3 + (m2*l1*l2*cos(angle2))/2)*acc2 ...
            - (m2*l1*l2*sin(angle2))*omega1*omega2 - ((m2*l1*l2)/2)*omega2*omega2 + (m1/2 + m2)*g*l1*cos(angle1) + ((m2*g*l2)/2)*cos(angle1 + angle2);

    T_l2 = ((m2*l2*l2)/3)*acc2 + ((m2*l2*l2)/3 + ((m2*l1*l2)/2)*cos(angle2))*acc1 + (m2*l1*l2*sin(angle2)/2)*omega1*omega1 + ((m2*g*l2)/2)*cos(angle1 + angle2);

    % Derivatives
    dydt = zeros(6, 1);
    dydt(1)=omega1;
    dydt(2)=omega2;
    dydt(3)=acc1;
    dydt(4)=acc2;
    dydt(5)= (V1*k/(L*J)) - (k^2/(L*J)*omega1) - (R*b/(L*J)*omega1) - ((b*L+J*R)/(L*J)*acc1 - R*T_l1);
    dydt(6)= (V2*k/(L*J)) - (k^2/(L*J)*omega2) - (R*b/(L*J)*omega2) - ((b*L+J*R)/(L*J)*acc2 - R*T_l2);

end

