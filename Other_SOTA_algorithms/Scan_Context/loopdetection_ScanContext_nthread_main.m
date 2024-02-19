clear; clc;
%%代码功能：基于scan context描述子对semantic-kitti数据集进行回环检测。
% global Hfov;
% global max_range;
% global rings_num;
% global sectors_num;
% global enough_node_diff_num;
% global candidates_num;
% global loop_thres;
% global ringkeys;
% global ScanContexts;
% global translations_gt;
% global exist_loop_gt;
% global nodes_num;

%% { 设定要查看数据的文件夹路径。 }
LidarData_path = 'G:/big-data/Semantic-KITTI/LIDAR/dataset/sequences/00/velodyne/'; % 一帧点云的文件路径
Groundtruth_path = 'E:/matrix_labs/lab0_loopGroundtruth_by_pose/semantic-kitti/00.mat';
translations_gt = load(Groundtruth_path).groundtruth(:,1:2);
exist_loop_gt = load(Groundtruth_path).groundtruth(:,3);
nodes_num = size(exist_loop_gt,1); % 当前数据序列的帧数，每帧看成一个节点。

%% { 第二步：设定相关初始参数 }
Hfov = 120; %激光雷达水平视角范围。
max_range = 80; %激光雷达最远测量范围。
rings_num = 80; % 划分用的圆环数，即Nr。
sectors_num = 120; % 每环划分的扇段数，即Ns。
enough_node_diff_num = 50; % 可接受的帧序差别阈值（剔除相邻帧的影响）。
candidates_num = 10;
detection_loop = zeros(nodes_num,1); % nx1: 检测出的回环帧号。

%% { 实战：读取数据序列进行不同阈值下的回环检测 }
loop_thres = 0.12;

% 1. 参数设置
ringkeys = []; % 准备存放各Scan context对应的ringkey。
ScanContexts = {}; % 准备存放各Scan context。

% 先把所有帧的scan context和ringkey都处理出来。
for i = 0 : 400
    fprintf('加载帧次：%d\n', i);
    ith_query_pcd = KITTIbin2PtcloudWithIndex_function(LidarData_path, i); % 根据当前帧索引读取当前帧的点云,包含剔除范围外的处理。
    ith_q_scancontext = Scan_context_new(ith_query_pcd, Hfov, max_range, rings_num, sectors_num);
    ScanContexts{end+1} = ith_q_scancontext; % 当前帧的scan context都保存到ScanContexts数组中
    
    ith_q_ringkey = ScanContext2RingKey_function(ith_q_scancontext); % 计算当前帧scan context的ringkey
    ringkeys = [ringkeys; ith_q_ringkey]; % 当前帧的ringkey保存到ringkeys矩阵中
end

k1 = cell2mat(ScanContexts(20));

% % opening multi-threads.
% parpool(2); % 启动
% data = Composite();
% data{1} = 1:200; % 数据划分
% data{2} = 201:400; 
% spmd 
%     output = scancontext_detection(data); %每个线程各自计算结果。
% end
% disp(length(output{1}));
% out1 = output{1}; %cpu1根据函数计算1：100的结果，所有这些结果同时传给out1
% disp(output{2});
% out2 = output{2}; %cpu2根据函数计算201:400的结果，所有这些结果同时传给out2
% delete(gcp('nocreate')) % 关闭并行池
% detection_loop = [out1,out2]'; %合并所有线程的结果
% 
% % 检测结果统计
% detection_result = zeros(nodes_num,2); % nx2: 是否检测出回环，是否正确。
% for i = 0 : 400
%     candidateID_of_NearestDist = detection_loop(i+1);
%     if(candidateID_of_NearestDist == 0)
%         continue; % 虽然有候选帧，但没有检测出回环。
%     else
%         detection_result(i+1,1) = 1; % 当前回环帧检测到回环。note that: 帧号从0开始，矩阵索引从1开始。
%         % 检测到的这个回环帧是否正确
%         dist_gap  = pdist([translations_gt(i + 1,:); translations_gt(candidateID_of_NearestDist,:)],'euclidean'); % 计算任意两时刻的真实距离差异。
%         if((dist_gap < 4) || (dist_gap == 4))
%             detection_result(i+1,2) = 1;
%         else
%             detection_result(i+1,2) = 0;
%         end
%     end
% end
% 
% function candidateID_of_NearestDist = scancontext_detection(i)
%     global enough_node_diff_num;
%     global candidates_num;
%     global loop_thres;
%     global ringkeys;
%     global ScanContexts;
%     disp(i);
%     ith_q_scancontext = cell2mat(ScanContexts(1,i+1)); %ScanContexts{i+1}，元组在多线程里必须用圆括号访问。
%     ith_q_ringkey = ringkeys(i+1, :);
% 
%     if(i <= enough_node_diff_num)
%        candidateID_of_NearestDist = 0; % query超过候选帧数量才开始检测。
%        return 
%     end
% 
%     % 创建KDtree聚类的对象,并剔除相邻帧的干扰。
%     KDtree_object = createns(ringkeys(1:i-enough_node_diff_num,:),"NSMethod","kdtree"); 
% 
%     % 根据ringkey，对当前帧聚类出与其相近的candidates_num个候选帧。返回值是这candidates_num个候选帧的索引
%     AllCandidates_index = knnsearch(KDtree_object, ith_q_ringkey,"K", candidates_num); 
% 
%     if(isempty(AllCandidates_index))
%         candidateID_of_NearestDist = 0;
%         return
%     end
% 
%     candidateID_of_NearestDist = 0; % 用来保存所有候选帧中与当前待检测回环帧之间距离最近的那个候选帧的索引值
%     CurrentQueryFrame_NearestDist = inf; % 用来保存所有候选帧中与当前待检测回环帧之间距离最近的距离值
% 
%     for ith_candidate = 1 : length(AllCandidates_index)
%         ithCandidate_ID = AllCandidates_index(ith_candidate); % 获取候选帧的id
%         ithCandidate_ScanContext = ScanContexts{ithCandidate_ID}; % 获取候选帧的scan context
% 
%         Dist_to_ithCandidate = CosineDistance_function(ith_q_scancontext, ithCandidate_ScanContext); % 每一个候选帧都与当前待检测回环帧计算差异性距离
% 
%         if(Dist_to_ithCandidate > loop_thres)
%             continue; % 如果当前候选帧与当前待检测回环帧之间的差异性距离超过了所设出现回环的差异性距离阈值loop_thres，那么此候选帧也将被剔除。
%         end
% 
%         % 经过将所有候选帧逐个与当前待检测回环帧进行差异性距离比较，最后得到与当前待检测回环帧差异性距离最小的那个候选帧。把它作为回环帧。
%         if(Dist_to_ithCandidate < CurrentQueryFrame_NearestDist)
%             CurrentQueryFrame_NearestDist = Dist_to_ithCandidate; % 更新最小距离值，所有候选帧都参与计算后会保存与待检测回环帧之间的最小距离值。
%             candidateID_of_NearestDist = ithCandidate_ID; % 更新最小距离值下候选帧名称索引，所有候选帧都参与计算后会保存与待检测回环帧之间的最小距离值下的候选帧名称索引，即回环帧名称。
%         end
%     end
% end

%% {多线程的一个简单示例}
%多线程的一个简单示例。
parpool(2);
data = Composite();
data{1} = 1; % 数据划分
data{2} = 201; %
spmd 
    %q1 = scancontext_detection(data);
    output = test(data);
end
disp(length(output{1}));
out1 = output{1}; %cpu1根据函数计算1：100的结果，所有这些结果同时传给q
disp(output{2});
out2 = output{2};
delete(gcp('nocreate')) % 关闭并行池
function tv = test(data1)
    global ringkeys;
    %tv = data1+1
    ith_q_scancontext = ringkeys(data1+1, :);
    tv = ith_q_scancontext(60, 80);
    disp(data1);
end

%% {查看结果}
% ptcloud = pointCloud(ith_query_pcd); %当前帧的所有点云信息保存在了ptcloud中，ptcloud的数据类型是matlab自带函数产生的。

% figure(1);clf; % 新建一个空窗口。
% pcshow(ptcloud);colormap jet;title('colormap显示的一帧点云'); % 显示第一帧的点云。显示设置
% set(gcf,'position',[10 60 550 550]); % 设定plot输出图片的显示窗口位置和尺寸。参数含义为：窗口屏幕显示的位置x,窗口屏幕显示的位置y,图像宽度,图像高度
% xlabel("x");ylabel("y");zlabel("z"); % 显示x轴，y轴，z轴
% 
% figure(2);clf; % 新建一个空窗口
% imagesc(ith_q_scancontext); % 显示第一帧点云对应的scan context描述子。
% set(gcf,'position',[600 300 600 300]); % 设定plot输出图片的显示窗口位置和尺寸。参数含义为：窗口屏幕显示的位置x,窗口屏幕显示的位置y,图像宽度,图像高度
% xlabel('sectors');ylabel('rings'); % 设置坐标值名称
% title(strcat('第一帧点云按rings=', num2str(80), ', sectors=', num2str(120), '划分生成的scan context'));
% caxis([min(points2(:, 3)), max(points2(:, 3))]);colormap turbo;colorbar; % 颜色显示范围0-传感器最大测量高度，并显示颜色条。