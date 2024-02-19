function angle = arctan2_function(y, x)
% 输入：点云的横坐标x值，y值
% 输出：点云的角度，以度为单位

    if(x>=0 && y>=0)
        angle =  atan(y/x); % angle =  180/pi * atan(y/x)
    elseif(x<0 && y>0)
        angle = (atan(y/x)+pi); % angle = (180/pi) * (atan(y/x)+pi)
    elseif(x<0 && y<0)
        angle = (atan(y/x)-pi); % angle = (180/pi) * (atan(y/x)-pi)
    elseif(x>=0 && y<0)
        angle = atan(y/x); % angle = (180/pi) * atan(y/x)
    else
        angle = pi; % angle = (180/pi) * pi
    end

end