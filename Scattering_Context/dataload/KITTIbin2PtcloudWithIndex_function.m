function points2 = KITTIbin2PtcloudWithIndex_function(LidarData_path, index)
% 输入：盛放所有点云二进制文件的文件夹路径、某个二进制文件的索引
% 输出：对应的点云文件
%% 点云二进制文件夹路径 
if( length(num2str(index)) == 5 )
    bin_path = strcat(LidarData_path, '0', num2str(index), '.bin');
elseif( length(num2str(index)) == 4 )
    bin_path = strcat(LidarData_path, '00', num2str(index), '.bin');
elseif (length(num2str(index)) == 3)
    bin_path = strcat(LidarData_path, '000', num2str(index), '.bin');
elseif (length(num2str(index)) == 2)
    bin_path = strcat(LidarData_path, '0000', num2str(index), '.bin');
elseif (length(num2str(index)) == 1)
    bin_path = strcat(LidarData_path, '00000', num2str(index), '.bin');
end

%% Read 
fid = fopen(bin_path, 'rb'); raw_data = fread(fid, [4 inf], 'single'); fclose(fid);
points = raw_data(1:3,:)'; 
points(:, 3) = points(:, 3); % z in car coord.

%% {选取120范围下数据，见'/数据分布图/点云数据分布.jpg'图中数据示例}
% 1.剔除y>=0的点%
% idy = find(points(:, 2)>=0);
% points(idy, :) = [];
% 2.再剔除x==0的点%
% idx = find(points(:, 1)==0);
% points(idx, :) = [];
% 3.剔除原点处的点%
rows_index1 = all(points==0,2);
points(rows_index1,:) = [];

% points2 = points;
% 2.再剔除不符合范围内的点。此步有点多余，划分scan context的时候会剔除掉120°范围外的点。
points2 = zeros(size(points,1),3);
for i = 1 : size(points, 1)
    yaw = arctan2_function(points(i,2),points(i,1));
    %if ((yaw>0) && (yaw > 4*pi/6) && (yaw < (6*pi/6))) || ((yaw<0) && (yaw> -6*pi/6) && (yaw < -4*pi/6))%后
    if ((yaw>0) && (yaw > 0*pi/6) && (yaw < (2*pi/6))) || ((yaw<0) && (yaw > -2*pi/6) && (yaw < 0*pi/6))%前ok
    %if ((yaw>0) && (yaw > 3*pi/6) && (yaw < (6*pi/6))) || ((yaw<0) && (yaw > -6*pi/6) && (yaw < -5*pi/6))%左后
    %if ((yaw>0) && (yaw > 0*pi/6) && (yaw < (3*pi/6))) || ((yaw<0) && (yaw > -1*pi/6) && (yaw < 0*pi/6))%左前
    %if ((yaw>0) && (yaw > 1*pi/6) && (yaw < (5*pi/6)))%左
    %if ((yaw<0) && (yaw > -5*pi/6) && (yaw < -1*pi/6))%右
        points2(i,:) = points(i,:);
    else
        continue;
    end
end
rows_index2 = all(points2==0,2);
points2(rows_index2,:) = [];

end % end of function
