
function [phi, theta, psi] = Quaternion2Euler(quaternion)
    % converts a quaternion attitude to an euler angle attitude
    e0 = quaternion(1);
    e1 = quaternion(2);
    e2 = quaternion(3);
    e3 = quaternion(4);
    phi = atan2(2*(e0*e1+e2*e3), e0^2+e3^2-e1^2-e2^2);
    theta = asin(2*(e0*e2-e1*e3));
    psi = atan2(2*(e0*e3+e1*e2),e0^2+e1^2-e2^2-e3^2);
end
