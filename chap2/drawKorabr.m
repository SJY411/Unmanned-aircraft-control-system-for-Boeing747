function drawKorabr(u)
%太空船动画
    pn = u(1);
    pe = u(2);
    pd = u(3);
    phi = u(4);
    theta = u(5);
    psi = u(6);
    t = u(7);
    persistent spacecraft_handle

    if t==0
        figure(1), clf
        spacecraft_handle = drawKorabrBody(pn, pe, pd, phi, theta, psi, [],'normal');
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the view angle for figure
        axis([-10,10,-10,10,-10,10]);
        hold on
    % at every other time step, redraw base and rod
    else 
        drawKorabrBody(pn, pe, pd, phi, theta, psi, spacecraft_handle);
    end
end
    
function handle = drawKorabrBody(pn, pe, pd, phi, theta, psi, handle, mode)
    %绘制船体
    NED = spacecraftPoints;
    NED = rotateKorabr(NED, phi, theta, psi);
    NED = translateKorabr(NED, pn, pe, pd);
    %NED坐标系到matlab坐标系XYZ的转换
    R = [
        0, 1, 0;
        1, 0, 0;
        0, 0, -1;
        ];
    XYZ = R*NED;
    if isempty(handle)
        handle = plot3(XYZ(1,:), XYZ(2,:), XYZ(3,:),'-k', 'EraseMode', mode);
    else
        set(handle, 'XData', XYZ(1,:), 'YData', XYZ(2,:), 'ZData', XYZ(3,:));
        drawnow
    end
end

function XYZ = spacecraftPoints
% 在NED坐标系设定飞船各点
    XYZ = [
        1 1 0;
        1 -1 0;
        -1 -1 0;
        -1 1 0;
        1 1 0;
        1 1 -2;
        1 -1 -2;
        1 -1 0;
        1 -1 -2;
        -1 -1 -2;
        -1 -1 0;
        -1 -1 -2;
        -1 1 -2;
        -1 1 0;
        -1 1 -2;
        1 1 -2;
        1 1 0;
        1.5 1.5 0;
        1.5 -1.5 0;
        -1.5 -1.5 0;
        -1 -1 0;
        -1.5 -1.5 0;
        -1.5 1.5 0;
        -1 1 0;
        -1.5 1.5 0;
        1.5 1.5 0;
        ]';
end

function XYZ = rotateKorabr(XYZ, phi, theta, psi)
    R_roll = [
        1, 0, 0;
        0, cos(phi), -sin(phi);
        0, sin(phi), cos(phi)];
    R_pitch = [
        cos(theta), 0, sin(theta);
        0, 1, 0;
        -sin(theta), 0, cos(theta)];
    R_yaw = [
        cos(psi), -sin(psi), 0;
        sin(psi), cos(psi), 0;
        0, 0, 1];
    R = R_roll*R_pitch*R_yaw;
    XYZ = R*XYZ;
end

function XYZ = translateKorabr(XYZ, pn, pe, pd)
    XYZ = XYZ + repmat([pn;pe;pd], 1, size(XYZ, 2));
end