function [x,y] = forward_kinematics(th1, th2, l1, l2)

    %forward kinematics
    T0_1 = [cos(th1) -sin(th1) 0 l1*cos(th1);
            sin(th1) cos(th1) 0 l1*sin(th1);
            0        0        1   0;
            0        0         0    1;];
    
    
    T1_2 = [cos(th2) -sin(th2) 0 l2*cos(th2);
            sin(th2) cos(th2) 0 l2*sin(th2);
            0        0        1   0;
            0        0         0    1;];
    
    
    T0_2 = T0_1 * T1_2;

    x = T0_2(1,4);
    y = T0_2(2,4);

end






