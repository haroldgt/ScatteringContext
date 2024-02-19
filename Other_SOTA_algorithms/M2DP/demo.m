% Demo of the M2DP descriptor described in the following paper:
%
% Li He, Xiaolong Wang and Hong Zhang, M2DP: A Novel 3D Point Cloud 
% Descriptor and Its Application in Loop Closure Detection, IROS 2016.
%
% Li He, Dept. of Computing Science, University of Alberta
% lhe2@ualberta.ca

%% 0. Please run mex CountVote2D.c in Matlab
% mex CountVote2D.c

%% 1. Read a point cloud
% 000000.bin is from KITTI dataset of sequence 00.
% http://www.cvlibs.net/datasets/kitti/eval_odometry.php
[data1, numData1] = ReadBinData('G:/big-data/Semantic-KITTI/LIDAR/dataset/sequences/00/velodyne/000000.bin');
[data2, numData2] = ReadBinData('G:/big-data/Semantic-KITTI/LIDAR/dataset/sequences/00/velodyne/003695.bin');
% display the input data
figure(10);grid on;hold on
pcshow(data1);title('Input point cloud');

%% 2. M2DP
% desM2DP: descriptor of data; A: 2D signatures of data.
tstart = tic;
[desM2DP1, A1] = M2DP(data1);
[desM2DP2, A2] = M2DP(data2);
telapsed = toc(tstart);
disp(['Processing time of M2DP on ' num2str(numData1) ' points: ' num2str(telapsed) ' seconds']);
dist_gap  = pdist([desM2DP1'; desM2DP2'],'euclidean'); % 计算任意两时刻的真实距离差异。

% 2.查看PR曲线 %
figure(11);clf; % 新建一个空窗口
imagesc(A1); % 显示第一帧点云对应的scan context描述子。
set(gcf,'position',[600 300 600 300]); % 设定plot输出图片的显示窗口位置和尺寸。参数含义为：窗口屏幕显示的位置x,窗口屏幕显示的位置y,图像宽度,图像高度
xlabel('sectors');ylabel('rings'); % 设置坐标值名称
title(strcat('第一帧点云按rings=', num2str(80), ', sectors=', num2str(120), '划分生成的scan context'));
caxis([0, max(max(A1))]);colormap turbo;colorbar; % 颜色显示范围0-传感器最大测量高度，并显示颜色条。

figure(12);clf; % 新建一个空窗口
imagesc(A2); % 显示第一帧点云对应的scan context描述子。
set(gcf,'position',[600 300 600 300]); % 设定plot输出图片的显示窗口位置和尺寸。参数含义为：窗口屏幕显示的位置x,窗口屏幕显示的位置y,图像宽度,图像高度
xlabel('sectors');ylabel('rings'); % 设置坐标值名称
title(strcat('第一帧点云按rings=', num2str(80), ', sectors=', num2str(120), '划分生成的scan context'));
caxis([0, max(max(A2))]);colormap turbo;colorbar; % 颜色显示范围0-传感器最大测量高度，并显示颜色条。

function [data, numData] = ReadBinData(nameDataFile)

fid = fopen(nameDataFile,'r');
if fid==-1
    disp('Cannot find input file\n');
    data=[];
    numData=[];
    return;
end

% get the size of file
fseek(fid,0,'eof');
fsize = ftell(fid);
% number points = total bytes / 4 per float32 / 4 float32 per row
numData = floor(fsize/16);

% now, read data
fseek(fid,0,'bof');
data = fread(fid,fsize,'float32');
fclose(fid);

% reshape data
data = reshape(data,[4, numData]);
data = data';
% ignoring the last feature, laser intensity
data = data(:,1:3);
end