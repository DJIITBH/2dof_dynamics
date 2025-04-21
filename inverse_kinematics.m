function [th1, th2] = inverse_kinematics(x,y,l1,l2)

    if (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2) >= 0

       disp("positive")


    else 
        disp("negative")
    end
    th2 = acos((x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2))
    
    d = sqrt(x^2 + y^2);
    
    gamma = acos((l1^2 + d^2 - l2^2)/(2*l1*d));

    th11 = atan2(y,x) - gamma

    if th11<0
        th1 = th11 + 6.28;
    else
        th1 = th11;
    end


end
