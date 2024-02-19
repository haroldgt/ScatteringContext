clear; clc;
%%代码功能：基于scan context描述子对rulman数据集进行回环检测。

%% { 设定要查看数据的文件夹路径。 }
LidarData_path = 'G:/big-data/mulran/DCC03/lidar/'; % 一帧点云的文件路径
Groundtruth_path = 'E:/matrix_labs/lab0_loopGroundtruth_by_pose/mulran/dcc03.mat';
translations_gt = load(Groundtruth_path).groundtruth(1:end-1,1:2);
exist_loop_gt = load(Groundtruth_path).groundtruth(1:end-1,3);
nodes_num = size(exist_loop_gt,1); % 当前数据序列的帧数，每帧看成一个节点。

%% { 第二步：设定相关初始参数 }
Hfov = 120; %激光雷达水平视角范围。
max_range = 120; %激光雷达最远测量范围。
rings_num = 30; % 划分用的圆环数，即Nr。
sectors_num = 60*3; % 每环划分的扇段数，即Ns。
enough_node_diff_num = 150; % 可接受的帧序差别阈值（剔除相邻帧的影响）。
candidates_num = 50;
gridStep = 0.75;

%% { 实战：读取数据序列进行不同阈值下的回环检测 }
% loop_thres = 0.12;

% 1. 参数设置
M2DPS = []; % 准备存放各Scan context对应的ringkey。
%ScanContexts = {}; % 准备存放各Scan context。
candidates_group = {}; % 存放每个query的候选帧号。
candidates_gaps = {}; % 存放每个候选帧与query的距离。
Description_consumption = [];
Retrieval_consumption = [];

for i = 0 : (nodes_num - 1)
    fprintf('%d\n', i);
    ith_query_pcd = KITTIbin2PtcloudWithIndex_function(LidarData_path, i); % 根据当前帧索引读取当前帧的点云,包含剔除范围外的处理。
    ith_query_pcd = pointCloud(ith_query_pcd);
    ptcloud = pcdownsample(ith_query_pcd, 'gridAverage', gridStep);
    ith_query_pcd =ptcloud.Location;
    t0 = tic;
    [ith_q_m2dp, A1] = M2DP(ith_query_pcd); %注意，输出是列向量192*1
    tCount0 = toc(t0);
    Description_consumption = [Description_consumption; tCount0;];
    %ith_q_scancontext = Scan_context_new(ith_query_pcd, Hfov, max_range, rings_num, sectors_num);
    %ScanContexts{end+1} = ith_q_scancontext; % 当前帧的scan context都保存到ScanContexts数组中

    %ith_q_ringkey = ScanContext2RingKey_function(ith_q_scancontext); % 计算当前帧scan context的ringkey
    M2DPS = [M2DPS; ith_q_m2dp']; % 当前帧的ringkey保存到ringkeys矩阵中

    if(i <= enough_node_diff_num)
       candidates_group{end+1} = []; %没有候选帧也得存一下。
       candidates_gaps{end+1} = [];
       continue; % query超过候选帧数量才开始检测。 
    end
    t1 = tic;
    % 创建KDtree聚类的对象,并剔除相邻帧的干扰。
    KDtree_object = createns(M2DPS(1:i-enough_node_diff_num,:),"NSMethod","kdtree"); 

    % 根据ringkey，对当前帧聚类出与其相近的candidates_num个候选帧。返回值是这candidates_num个候选帧的索引
    [AllCandidates_index,AllCandidates_gaps] = knnsearch(KDtree_object, ith_q_m2dp',"K", candidates_num); 
    % 剔除当前待回环检测帧受相邻帧的影响。
    % checkneighber = (AllCandidates_index - (i+1))*(-1);
    % idnear = find(checkneighber <= enough_node_diff_num);
    % AllCandidates_index(idnear) = [];
    if(isempty(AllCandidates_index))
        candidates_group{end+1} = []; %没有候选帧也得存一下。
        candidates_gaps{end+1} = [];
    else
        candidates_group{end+1} = AllCandidates_index;
        candidates_gaps{end+1} = AllCandidates_gaps;
    end
    
%     candidates_gaps_ith = [];
%     for ith_candidate = 1 : length(AllCandidates_index)
%         ithCandidate_ID = AllCandidates_index(ith_candidate); % 获取候选帧的id
%         ithCandidate_ScanContext = ScanContexts{ithCandidate_ID}; % 获取候选帧的scan context
% 
%         %if((i+1) - ithCandidate_ID < enough_node_diff_num)
%         %    continue;
%         %end
% 
%         Dist_to_ithCandidate = CosineDistance_function(ith_q_scancontext, ithCandidate_ScanContext); % 每一个候选帧都与当前待检测回环帧计算差异性距离
%         candidates_gaps_ith = [candidates_gaps_ith, Dist_to_ithCandidate];
%     end
%     candidates_gaps{end+1} = candidates_gaps_ith;
    tCount1 = toc(t1);
    Retrieval_consumption = [Retrieval_consumption; tCount1;];
end

results = [];
for loop_thres = 0.02:0.02:1
    detection_loop = zeros(nodes_num,2); % nx2: 是否检测出回环，是否正确。
    % 计算每个阈值下对应的Recall和Precision.
    for i = 0 : (nodes_num - 1)
        fprintf('%f, %d\n', loop_thres, i);
%         ith_q_scancontext = ScanContexts{i+1}; % 获取当前帧的scan context。
        AllCandidates_index  = candidates_group{i+1}; % 获取当前帧在之前操作中获取的候选帧。
        AllCandidates_gaps  = candidates_gaps{i+1};
        if(isempty(AllCandidates_index) || isempty(AllCandidates_gaps))
            continue;
        end
        candidateID_of_NearestDist = 0; % 用来保存所有候选帧中与当前待检测回环帧之间距离最近的那个候选帧的索引值
        CurrentQueryFrame_NearestDist = inf; % 用来保存所有候选帧中与当前待检测回环帧之间距离最近的距离值
        
        % 找出阈值范围内距离差异最小的候选帧号。
        for ith_candidate = 1 : length(AllCandidates_index)
            ithCandidate_ID = AllCandidates_index(ith_candidate); % 获取候选帧的id
            ithCandidate_gap = AllCandidates_gaps(ith_candidate); % 获取候选帧与query的距离。

            %if((i+1) - ithCandidate_ID < enough_node_diff_num)
            %    continue;
            %end

            % Dist_to_ithCandidate = CosineDistance_function(ith_q_scancontext, ithCandidate_ScanContext); % 每一个候选帧都与当前待检测回环帧计算差异性距离

            if(ithCandidate_gap > loop_thres)
                continue; % 如果当前候选帧与当前待检测回环帧之间的差异性距离超过了所设出现回环的差异性距离阈值loop_thres，那么此候选帧也将被剔除。
            end

            % 经过将所有候选帧逐个与当前待检测回环帧进行差异性距离比较，最后得到与当前待检测回环帧差异性距离最小的那个候选帧。把它作为回环帧。
            if(ithCandidate_gap < CurrentQueryFrame_NearestDist)
                CurrentQueryFrame_NearestDist = ithCandidate_gap; % 更新最小距离值，所有候选帧都参与计算后会保存与待检测回环帧之间的最小距离值。
                candidateID_of_NearestDist = ithCandidate_ID; % 更新最小距离值下候选帧名称索引，所有候选帧都参与计算后会保存与待检测回环帧之间的最小距离值下的候选帧名称索引，即回环帧名称。
            end
        end

        % 检测结果统计
        if(candidateID_of_NearestDist == 0)
            continue; % 虽然有候选帧，但没有检测出回环。
        else
            detection_loop(i+1,1) = 1; % 当前回环帧检测到回环。note that: 帧号从0开始，矩阵索引从1开始。
            % 检测到的这个回环帧是否正确
            dist_gap  = pdist([translations_gt(i + 1,:); translations_gt(candidateID_of_NearestDist,:)],'euclidean'); % 计算任意两时刻的真实距离差异。
            if((dist_gap < 4) || (dist_gap == 4))
                detection_loop(i+1,2) = 1;
            else
                detection_loop(i+1,2) = 0;
            end
        end

    end
    
    %计算本次阈值下检测的召回率和精度。
    [scan_recall,scan_precision] = RecallandPrecision(exist_loop_gt, detection_loop);
    results = [results; scan_recall,scan_precision];
end

idn = find(isnan(results));
results(idn) = 1;

%耗时
disp('Description average consumption:');
disp(sum(Description_consumption)/size(Description_consumption,1));
disp('Retrieval average consumption:');
disp(sum(Retrieval_consumption)/size(Retrieval_consumption,1));

% 2.查看PR曲线 %
figure(1);clf; % 新建一个空窗口。
plot(results(:,1),results(:,2))
set(gcf,'color','w');
xlabel("Recall");ylabel("Precision");
title(strcat('semantic-kitti数据集', num2str(0), num2str(0), '序列下scan context回环检测结果'));