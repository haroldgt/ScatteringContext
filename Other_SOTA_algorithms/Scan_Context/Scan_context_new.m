function context_ringsXsectors = Scan_context_new(points2, Hfov, max_range, rings_num, sectors_num)
% 输入：matlab自带数组类型的一帧点云数据，降采样处理(0为不做，0.x表示以该尺寸网格进行降采样)要进行分区域的环数，分区域的扇段数，
% 输出：该点云对应的context描述子矩阵，大小为rings_num*sectors_num。

% 相关的信息
FramePtcloud = points2;
points_num = size(FramePtcloud, 1);
rings_gap = max_range / rings_num; % 每环的径长
sectors_angle_gap = Hfov / sectors_num; % 每环中每个扇段的角度范围长

context_ringsXsectors = zeros(rings_num, sectors_num); % 创建一个context空矩阵。
% 各点分配到对应的bin区域中，直接进行编码得到scan context描述子。
for points_ith = 1 : points_num
    pointsXYZ_ith = FramePtcloud(points_ith, :); % 获得当前点的XYZ坐标
    pointsXYZ_r = sqrt(pointsXYZ_ith(1)*pointsXYZ_ith(1) + pointsXYZ_ith(2)*pointsXYZ_ith(2));  % 获得当前点XY的半径长。
    if(pointsXYZ_r > max_range)
        continue; % 剔除范围外的点。
    elseif(pointsXYZ_r == 0)
        points_ith_rings_index = rings_num;
    else
        located_n_rings = pointsXYZ_r/rings_gap;
        points_ith_rings_index = rings_num + 1 - ceil(located_n_rings); % matrix's row index. note that: 7 = 3/0.5 + 1
    end
    
    theta = arctan2_function(pointsXYZ_ith(2), pointsXYZ_ith(1)) * (180/pi) * (-1); % 水平角度值。
    if ((theta < 30) || (theta >150) )
        located_n_sectors = 0;
        continue; % 剔除范围外的点。
    elseif(theta == 30)
        points_ith_sectors_index = 1;
    else
        located_n_sectors = (theta - 30)/sectors_angle_gap;
        points_ith_sectors_index = ceil(located_n_sectors);
    end
    
    encodeValue_in_bins = context_ringsXsectors(points_ith_rings_index, points_ith_sectors_index);
    ibin_max = max(encodeValue_in_bins, abs(pointsXYZ_ith(:, 3)));
    context_ringsXsectors(points_ith_rings_index, points_ith_sectors_index) = ibin_max; % 得到context描述子矩阵
end
end