% Authors: Donglin Zhu, Zhongli Wang
% Institution: Beijing Jiaotong University
% Emails: zdl_edu@sina.com, zlwang@bjtu.edu.cn
% Paper: Scattering Context: Dual-view Encoding-Based 3D Descriptor and Efficient Search Strategy for Loop Closure Detection
% Demo: Evaluation on KITTI dataset.
clear; clc;
addpath(genpath('..\dataload\'));
addpath(genpath('..\descriptor\'));
addpath(genpath('..\distance\'));
addpath(genpath('..\evaluate\'));

%% { Data Preparation }
LidarData_path = 'G:/big-data/Semantic-KITTI/LIDAR/dataset/sequences/00/velodyne/'; % 00 sequence path
Groundtruth_path = 'E:/matrix_labs/Scattering_Context/Groundtruth_pose/KITTI/00.mat'; % groundtruth file
translations_gt = load(Groundtruth_path).groundtruth(:,1:2); % groundtruth pose
exist_loop_gt = load(Groundtruth_path).groundtruth(:,3); % 1 presents current node exist the real loop
nodes_num = size(exist_loop_gt,1); % Total number of nodes

%% { Parameter Settings }
Vfov_up = 3; % maximum vertical viewing angle
Vfov_down = -25; % minimum vertical viewing angle
max_range = 80; % Sensor sensing range
rings_num = 20; % Set rings parameter
sectors_num = 60*3; % Set sectors parameter
plies_num = 20; % Set plies parameter
enough_node_diff_num = 50; % n adjacent nodes number
candidates_num = 25; % candidate frames number

%% { Loop Closure Detection }
STCKeys = []; % Key Database
STCDescriptors = {}; % Descriptor Database
candidates_group = {}; % log candidate frame IDs of each query
candidates_gaps = {}; % log similarity distance

for i = 0 : (nodes_num - 1)
    fprintf('%d\n', i);
    ith_query_pcd = KITTIbin2PtcloudWithIndex_function(LidarData_path, i); % loading the point cloud.
    ith_query_pcd = pointCloud(ith_query_pcd);
    [img,img2] = STC(ith_query_pcd, sectors_num, rings_num, max_range, Vfov_up, Vfov_down, plies_num);
    img_ndd = [img;img2];
    ith_q_scatteringcontext = img_ndd(:,61:120); % choose one-third data of each scan.
    STCDescriptors{end+1} = ith_q_scatteringcontext; % current query's descriptor be added to the Descriptor Database.

    ith_q_ringkey = sum(ith_q_scatteringcontext,2);
    STCKeys = [STCKeys; ith_q_ringkey']; % current query's key be added to the Key Database.

    if(i <= enough_node_diff_num) % In total n previously adjacent nodes are excluded.
       candidates_group{end+1} = [];
       candidates_gaps{end+1} = [];
       continue;
    end

    % Creating a KD-tree object
    KDtree_object = createns(STCKeys(1:i-enough_node_diff_num,:),"NSMethod","kdtree"); 

    % Searching candidate frames
    AllCandidates_index = knnsearch(KDtree_object, ith_q_ringkey',"K", candidates_num); 
    if(isempty(AllCandidates_index))
        candidates_group{end+1} = [];
        candidates_gaps{end+1} = [];
    else
        candidates_group{end+1} = AllCandidates_index;
    end
    
    % Calculating similarity distance by the Pairwise Comparison
    candidates_gaps_ith = [];
    for ith_candidate = 1 : length(AllCandidates_index)
        ithCandidate_ID = AllCandidates_index(ith_candidate); % Nearest Neighbor IDs
        ithCandidate_ScanContext = STCDescriptors{ithCandidate_ID};

        NDD_sim = corr_sim(cat(3,ith_q_scatteringcontext(1:20,:),ith_q_scatteringcontext(21:end,:)), cat(3,ithCandidate_ScanContext(1:20,:),ithCandidate_ScanContext(21:end,:)));
        Dist_to_ithCandidate = 1- abs(NDD_sim);
        candidates_gaps_ith = [candidates_gaps_ith, Dist_to_ithCandidate];
    end
    candidates_gaps{end+1} = candidates_gaps_ith;
end

results = []; % log detection results
for loop_thres = 0.02:0.02:1 % loop_thres represents the acceptance threshold in paper.
    detection_loop = zeros(nodes_num,2); % 1st column represents whether loop be detected, 2ed column represents Is it detected correctly.
    for i = 0 : (nodes_num - 1)
        fprintf('%f, %d\n', loop_thres, i);
        ith_q_scatteringcontext = STCDescriptors{i+1};
        AllCandidates_index  = candidates_group{i+1};
        AllCandidates_gaps  = candidates_gaps{i+1};
        if(isempty(AllCandidates_index) || isempty(AllCandidates_gaps))
            continue;
        end
        candidateID_of_NearestDist = 0;
        CurrentQueryFrame_NearestDist = inf;
        
        % Finding the one corresponding to the minimum distance as the final loop
        for ith_candidate = 1 : length(AllCandidates_index)
            ithCandidate_ID = AllCandidates_index(ith_candidate);
            ithCandidate_gap = AllCandidates_gaps(ith_candidate);

            if(ithCandidate_gap > loop_thres)
                continue;
            end

            if(ithCandidate_gap < CurrentQueryFrame_NearestDist)
                CurrentQueryFrame_NearestDist = ithCandidate_gap;
                candidateID_of_NearestDist = ithCandidate_ID;
            end
        end

        % Calculating Recall and Precision.
        if(candidateID_of_NearestDist == 0)
            continue;
        else
            detection_loop(i+1,1) = 1;
            % If a ground truth pose distance between a query and a matched node is less than 4m, the detection is considered as true positive.
            dist_gap  = pdist([translations_gt(i + 1,:); translations_gt(candidateID_of_NearestDist,:)],'euclidean');
            if((dist_gap < 4) || (dist_gap == 4))
                detection_loop(i+1,2) = 1;
            else
                detection_loop(i+1,2) = 0;
            end
        end

    end
    
    % log all detection results
    [scan_recall,scan_precision] = RecallandPrecision(exist_loop_gt, detection_loop);
    results = [results; scan_recall,scan_precision];
end

idn = find(isnan(results));
results(idn) = 1;

%% { Show Precision-Recall Curve }
figure(1);clf;
plot(results(:,1),results(:,2))
set(gcf,'color','w');
xlabel("Recall");ylabel("Precision");
title(strcat(' Precision-recall curve of Scattering Context on KITTI', num2str(0), num2str(0), ' dataset'));