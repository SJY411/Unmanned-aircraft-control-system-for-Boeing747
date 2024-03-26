function R = Quaternion2Rotation(quaternion)
    %converts a quaternion attitude to a rotation matrix
    e0 = quaternion(1);
    e1 = quaternion(2);
    e2 = quaternion(3);
    e3 = quaternion(4);

    R = [1-2*e2^2-2*e3^2  2*e1*e2+2*e0*e3  2*e1*e3-2*e0*e2;
         2*e1*e2-2*e0*e3  1-2*e1^2-2*e3^2  2*e2*e3+2*e0*e1;
         2*e1*e3+2*e0*e2  2*e2*e3-2*e0*e1  1-2*e1^2-2*e2^2];
end
