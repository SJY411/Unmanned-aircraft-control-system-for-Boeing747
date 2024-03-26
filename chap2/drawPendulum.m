function drawPendulum(u)
%倒立摆动画
    y = u(1);
    theta = u(2);
    t = u(3);
    
    L = 1;
    gap = 0.01;
    width = 1.0;
    height = 0.1;
    
    persistent base_handle
    persistent rod_handle
    
    if t == 0
        figure(1), clf
        track_width = 3;
        plot([track_width, track_width], [0, 0], 'k');
        hold on
        base_handle = drawBase(y, width, height, gap, [], 'normal');
        rod_handle = drawRod(y, theta, L, gap, height, [], 'normal');
        axis([-track_width, track_width, -L, 2*track_width-L]);
    else
        drawBase(y, width, height, gap, base_handle);
        drawRod(y, theta, L, gap, height, rod_handle);
    end
end

%绘制倒立摆底座
function handle = drawBase(y, width, height, gap, handle, mode)
    X = [y-width/2, y+width/2, y+width/2, y-width/2];
    Y = [gap, gap, gap+height, gap+height];
    if isempty(handle)
        handle = fill(X, Y, 'm', 'EraseMode', mode);
    else
        set(handle, 'XData', X, 'YData', Y);
    end
end

%绘制倒立摆摆杆
function handle = drawRod(y, theta, L, gap, height, handle, mode)
    X = [y, y+L*sin(theta)];
    Y = [gap+height, gap+height+L*cos(theta)];
    if isempty(handle)
        handle = plot(X, Y, 'g', 'EraseMode', mode);
    else
        set(handle, 'XData', X, 'YData', Y);
    end
end
