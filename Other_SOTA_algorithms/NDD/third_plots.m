% plane_points = load('E:\matrix_labs\lab3_persistent_homology\matlab_examples\plane_points.mat').plane_points;
% surface_points = load('E:\matrix_labs\lab3_persistent_homology\matlab_examples\surface_points.mat').surface_points;
% corner_points = load('E:\matrix_labs\lab3_persistent_homology\matlab_examples\corner_points.mat').corner_points;
% noise_points = load('E:\matrix_labs\lab3_persistent_homology\matlab_examples\noise_points.mat').noise_points;
% 
% figure(1);clf;
% if(isempty(plane_points) == 0)
%     scatter3(plane_points(:,1), plane_points(:,2), plane_points(:,3),'g.');
%     hold on;
% end
% if(isempty(surface_points) == 0)
%     scatter3(surface_points(:,1), surface_points(:,2), surface_points(:,3), 'b.');
%     hold on;
% end
% if(isempty(corner_points) == 0)
%     scatter3(corner_points(:,1), corner_points(:,2), corner_points(:,3),'r.');
%     hold on;
% end
% % if(isempty(noise_points) == 0)
% %     scatter3(noise_points(:,1), noise_points(:,2), noise_points(:,3), 'r.');
% % end
% set(gcf,'position',[10 60 550 550]);
% axis equal;

% load_javaplex
% import edu.stanford.math.plex4.*;
% disp('hello javaplex!');

LidarData_path = 'G:/big-data/Semantic-KITTI/LIDAR/dataset/sequences/00/velodyne/000005.bin'; % 一帧点云的文件路径
% pcd_data = load(LidarData_path).points2(:,1:3);
Vfov_up = 3; % 激光雷达上半部分垂直角度值。yes
Vfov_down = -25; %激光雷达下半部分垂直角度值。yes
Hfov = 120; %激光雷达水平视角范围。
Vfov_resolution = 1.5; %激光雷达垂直角度分辨率。
Hfov_resolution = 2; %激光雷达水平角度分辨率。
max_range = 80; %激光雷达最远测量范围。yes
theta_start = 68; % 水平起始角度yes
theta_end = 188; % 水平起始角度yes
rings_num = 20; % 划分用的圆环数，即Nr。yes
sectors_num = 60; % 每环划分的扇段数，即Ns。
plies_num = 20; % 垂直方向的phi分辨率，即Np.yes

% rings_num = 28; % 划分用的圆环数，即Nr。
% sectors_num = 60; % 每环划分的扇段数，即Ns。
boundary = 20; % 用来区分大视野和小视野的边界线，单位m。yes
enough_node_diff_num = 100; % 可接受的帧序差别阈值（剔除相邻帧的影响）。
candidates_num = 10;
js_threshold = 0.23;
corners_num_threshold = 100;

fidq = fopen(LidarData_path, 'rb'); 
raw_data1_property4 = fread(fidq, [4 inf], 'single'); % x, y, z, intensity
fclose(fidq);
raw_points = raw_data1_property4(1:4,:)';
points = raw_points(:,1:3);

% 点云下采样
ptcloud = pointCloud(points);
gridStep = 0.75; % cubic grid downsampling(m). 
ptcloud1 = pcdownsample(ptcloud, 'gridAverage', gridStep);
points = ptcloud1.Location;

%剔除原点
rows_index = all(points==0,2);
points(rows_index,:) = [];
%剔除x = 0的点，为计算theta做准备。
idx = find(points(:, 1)==0);
points(idx, :) = [];
%剔除z = 0的点，为计算phi做准备。
idz = find(points(:, 3)==0);
points(idz, :) = [];
%剔除传感器范围外的点。
iframe_r = sqrt(points(:, 1).*points(:, 1) + points(:, 2).*points(:, 2) + points(:, 3).*points(:, 3));
id_r = find(iframe_r>=max_range);
points(id_r,:) = [];

%寻找各象限内的点,计算极坐标theta,r
id1 = find(points(:, 1)>=0 & points(:, 2)>=0); %x>0,y>=0
Quadrant1st_points = points(id1, :);
Quadrant1st_thata = 180/pi * atan(Quadrant1st_points(:, 2)./Quadrant1st_points(:, 1));
Quadrant1st_r = sqrt(Quadrant1st_points(:, 1).*Quadrant1st_points(:, 1) + Quadrant1st_points(:, 2).*Quadrant1st_points(:, 2));

id2 = find(points(:, 1)<0 & points(:, 2)>0);%x<0,y>0
Quadrant2ed_points = points(id2, :);
Quadrant2ed_thata = 180/pi * (atan(Quadrant2ed_points(:, 2)./Quadrant2ed_points(:, 1))+pi);
Quadrant2ed_r = sqrt(Quadrant2ed_points(:, 1).*Quadrant2ed_points(:, 1) + Quadrant2ed_points(:, 2).*Quadrant2ed_points(:, 2));

id3 = find(points(:, 1)<0 & points(:, 2)<0);%x<0,y<0
Quadrant3rd_points = points(id3, :);
Quadrant3rd_thata = (180/pi) * (atan(Quadrant3rd_points(:, 2)./Quadrant3rd_points(:, 1))+pi);
Quadrant3rd_r = sqrt(Quadrant3rd_points(:, 1).*Quadrant3rd_points(:, 1) + Quadrant3rd_points(:, 2).*Quadrant3rd_points(:, 2));

id4 = find(points(:, 1)>=0 & points(:, 2)<0);%x>=0,y<0
Quadrant4th_points = points(id4, :);
Quadrant4th_thata = (180/pi) * (atan(Quadrant4th_points(:, 2)./Quadrant4th_points(:, 1))+2*pi);
Quadrant4th_r = sqrt(Quadrant4th_points(:, 1).*Quadrant4th_points(:, 1) + Quadrant4th_points(:, 2).*Quadrant4th_points(:, 2));

points_output = [Quadrant1st_points;Quadrant2ed_points;Quadrant3rd_points;Quadrant4th_points];
points_r = [Quadrant1st_r;Quadrant2ed_r;Quadrant3rd_r;Quadrant4th_r];
points2pol_thata = [Quadrant1st_thata+90;Quadrant2ed_thata+90;Quadrant3rd_thata+90;Quadrant4th_thata+90-360]; % 改变起始轴，即y轴负半轴开始沿逆时针方向。

%计算垂直方向上phi值。
id_Zpos = find(points_output(:, 3)>0);
points_Zpos = points_output(id_Zpos, :);
points_r_Zpos = points_r(id_Zpos, :);
points2pol_thata_Zpos = points2pol_thata(id_Zpos, :);
points_phi_Zpos = 180/pi * atan(points_r_Zpos./points_Zpos(:,3));

id_Zneg = find(points_output(:, 3)<0);
points_Zneg = points_output(id_Zneg, :);
points_r_Zneg = points_r(id_Zneg, :);
points2pol_thata_Zneg = points2pol_thata(id_Zneg, :);
points_phi_Zneg = 180/pi * (atan(points_r_Zneg./points_Zneg(:,3))+pi);

pcd = [points_Zpos;points_Zneg]; %n,3
pcd_r_theta = [[points_r_Zpos;points_r_Zneg],[points2pol_thata_Zpos;points2pol_thata_Zneg]]; %n,1
% pcd2pol_theta = [points2pol_thata_Zpos;points2pol_thata_Zneg]; %n,1
pcd_phi_theta = [[points_phi_Zpos;points_phi_Zneg],[points2pol_thata_Zpos;points2pol_thata_Zneg]]; %n,1

%计算描述子
img1 = zeros(rings_num, sectors_num);
img2 = zeros(plies_num, sectors_num);

r_step = max_range / rings_num;
t_step = (theta_end-theta_start)/sectors_num;
p_step = (abs(Vfov_up)+ abs(Vfov_down))/plies_num;

for r = 1:rings_num
    for t = 1:sectors_num
        r_start = (r-1)*r_step;
        r_end = r*r_step;
        t_start = theta_start + (t-1)*t_step;
        t_end = theta_start + t*t_step;
        ith_bin_index = find((pcd_r_theta(:,1)>r_start & pcd_r_theta(:,1)<=r_end) & (pcd_r_theta(:,2)>t_start & pcd_r_theta(:,2)<=t_end));
        ith_bin_points = pcd(ith_bin_index,:);
        disp(size(ith_bin_points,1));
        %计算每个bin的score,entropy
        if size(ith_bin_points,1) > 4
             points_in_bin_mean = mean(ith_bin_points);
             points_in_bin_cov = cov(ith_bin_points);
                [U,S,V] = svd(points_in_bin_cov);
                if S(3,3) < 0.001 * S(1,1)
                    S(3,3) = 0.001 * S(1,1);
                    points_in_bin_cov = U*S*V';
                end
                  [~, posDef] = chol(points_in_bin_cov);                
                if posDef ~= 0
                    % If the covariance matrix is not positive definite,
                    % disregard the contributions of this cell.
                    continue;
                end
                %entropy
                N = 4;
                entropy = (N/2)*(1+log(2*pi)) + 0.5*log(det(points_in_bin_cov));% log
                %points_in_bin_covinv = inv(points_in_bin_cov);
                q = ith_bin_points - points_in_bin_mean; 
                gaussianValue = -q / points_in_bin_cov * q'/2;
                score = sum(exp(diag(gaussianValue)));
        else
            entropy = 0;
            score = 0;
        end
        img1(r,t) = score + 1.3^entropy;
        %img1(r,t) = score + 1.3^entropy;
    end
end

for p = 1:plies_num
    for t = 1:sectors_num
        p_start = (90-abs(Vfov_up))+(p-1)*p_step;
        p_end = (90-abs(Vfov_up))+p*p_step;
        t_start = theta_start + (t-1)*t_step;
        t_end = theta_start + t*t_step;
        ith_bin_index = find((pcd_phi_theta(:,1)>p_start & pcd_phi_theta(:,1)<=p_end) & (pcd_phi_theta(:,2)>t_start & pcd_phi_theta(:,2)<=t_end));
        ith_bin_points = pcd(ith_bin_index,:);
        
%         iframe_r = sqrt(ith_bin_points(:, 1).*ith_bin_points(:, 1) + ith_bin_points(:, 2).*ith_bin_points(:, 2) + ith_bin_points(:, 3).*ith_bin_points(:, 3));
%         id_r = find(iframe_r>=boundary);
%         ith_bin_points(id_r,:) = [];
        
        disp(size(ith_bin_points,1));
        %计算每个bin的score,entropy
        if size(ith_bin_points,1) >=4
             points_in_bin_mean = mean(ith_bin_points);
             points_in_bin_cov = cov(ith_bin_points);
                [U,S,V] = svd(points_in_bin_cov);
                if S(3,3) < 0.001 * S(1,1)
                    S(3,3) = 0.001 * S(1,1);
                    points_in_bin_cov = U*S*V';
                end
                  [~, posDef] = chol(points_in_bin_cov);                
                if posDef ~= 0
                    % If the covariance matrix is not positive definite,
                    % disregard the contributions of this cell.
                    continue;
                end
                %entropy
                N = 4;
                entropy = (N/2)*(1+log(2*pi)) + 0.5*log(det(points_in_bin_cov));% log
                %points_in_bin_covinv = inv(points_in_bin_cov);
                q = ith_bin_points - points_in_bin_mean; 
                gaussianValue = -q / points_in_bin_cov * q'/2;
                score = sum(exp(diag(gaussianValue)));
        else
            entropy = 0;
            score = 0;
        end
        img2(p,t) = score + 1.3^entropy;
    end
end

id_theta = find((pcd_phi_theta(:,1)>87 & pcd_phi_theta(:,1)<=115) & (pcd_phi_theta(:,2)>68 & pcd_phi_theta(:,2)<=188));
points_e = pcd(id_theta, :);
disp(max(points_phi_Zneg));

% a = [1,2;
%     2,3;
%     3,4;];
% id = find(a(:,1)>1 & a(:,2)>3 & a(:,2)<=4);
% [context_frontpoints,context_verticalpoints,context_frontIntensity,context_verticalIntensity] = lean_context(pcd_data, Vfov_up, Vfov_down, Hfov, max_range, Vfov_resolution, Hfov_resolution,rings_num,boundary);
% front_number = max(max(context_frontIntensity));
% disp(front_number);
% vertical_number = max(max(context_verticalIntensity));
% disp(vertical_number);
% check_points = context_frontpoints{6,32};

ptcloud = pointCloud(points_e); %当前帧的所有点云信息保存在了ptcloud中，ptcloud的数据类型是matlab自带函数产生的。

figure(1);clf; % 新建一个空窗口。
pcshow(ptcloud);colormap jet;title('colormap显示的一帧点云'); % 显示第一帧的点云。显示设置
set(gcf,'position',[10 60 550 550]); % 设定plot输出图片的显示窗口位置和尺寸。参数含义为：窗口屏幕显示的位置x,窗口屏幕显示的位置y,图像宽度,图像高度
xlabel("x");ylabel("y");zlabel("z"); % 显示x轴，y轴，z轴

figure(2);clf; % 新建一个空窗口
imagesc(img2); % 显示第一帧点云对应的scan context描述子。
set(gcf,'position',[600 100 600 210]); % 设定plot输出图片的显示窗口位置和尺寸。参数含义为：窗口屏幕显示的位置x,窗口屏幕显示的位置y,图像宽度,图像高度
xlabel('sectors');ylabel('rings'); % 设置坐标值名称
title(strcat('第一帧点云按rings=', num2str(80), ', sectors=', num2str(120), '划分生成的scan context'));
caxis([min(min(img2)), max(max(img2))]);colormap turbo;colorbar; % 颜色显示范围0-传感器最大测量高度，并显示颜色条。

figure(3);clf; % 新建一个空窗口
imagesc(img1); % 显示第一帧点云对应的scan context描述子。
set(gcf,'position',[600 400 600 210]); % 设定plot输出图片的显示窗口位置和尺寸。参数含义为：窗口屏幕显示的位置x,窗口屏幕显示的位置y,图像宽度,图像高度
xlabel('sectors');ylabel('rings'); % 设置坐标值名称
title(strcat('第一帧点云按rings=', num2str(80), ', sectors=', num2str(120), '划分生成的scan context'));
caxis([min(min(img1)), max(max(img1))]);colormap turbo;colorbar; % 颜色显示范围0-传感器最大测量高度，并显示颜色条。

% max_dimension = 3; % 定义最高维，形成由0，1维单形构成的2维复形
% max_filtration_value = 0.4; % 最大滤值0.0061
% num_divisions = 1000; % division
% stream = api.Plex4.createVietorisRipsStream(check_points, max_dimension,max_filtration_value, num_divisions);% 创建stream
% persistence = api.Plex4.getModularSimplicialAlgorithm(max_dimension, 2);%计算同调
% intervals = persistence.computeIntervals(stream);%获取条形码
% 
% h0_endpoints = homology.barcodes.BarcodeUtility.getEndpoints(intervals, 0, false); %获得H1特征（H0,H1,H2）下的持续同调结果。
% h1_endpoints = homology.barcodes.BarcodeUtility.getEndpoints(intervals, 1, false); %获得H1特征（H0,H1,H2）下的持续同调结果。
% h2_endpoints = homology.barcodes.BarcodeUtility.getEndpoints(intervals, 2, false); %获得H1特征（H0,H1,H2）下的持续同调结果。
% options.filename = 'string';
% options.max_filtration_value = max_filtration_value;
% options.max_dimension = max_dimension - 1;
% plot_barcodes(intervals, options); % 可视化条形码