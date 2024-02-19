LidarData_path = 'G:/big-data/Semantic-KITTI/LIDAR/dataset/sequences/02/velodyne/000361.bin'; % 一帧点云的文件路径
fidq = fopen(LidarData_path, 'rb'); 
raw_data1_property4 = fread(fidq, [4 inf], 'single'); % x, y, z, intensity
fclose(fidq);
points = raw_data1_property4(1:3,:)'; 
rows_index = all(points==0,2);
points(rows_index,:) = [];
ptcloud = pointCloud(points);
[this_part_pol_log, key] = FreSCo(ptcloud);

[best_offset, fresco_dist] = computeFrescoDist(this_part_pol_log, this_part_pol_log);

function [img_out] = applyGaussian(img_in)
    sigma = 1;  %设定标准差值，该值越大，滤波效果（模糊）愈明显
    window = double(uint8(3*sigma)*2 + 1);  %设定滤波模板尺寸大小
    %fspecial('gaussian', hsize, sigma)产生滤波掩模
    G = fspecial('gaussian', window, sigma);
    img_out = imfilter(img_in, G, 'conv','replicate','same');
end