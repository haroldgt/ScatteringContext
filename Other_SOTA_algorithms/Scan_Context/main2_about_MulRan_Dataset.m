% Authors: Donglin Zhu, Zhongli Wang
clear; clc;
addpath(genpath('.\matlab\descriptor\'));
addpath(genpath('.\matlab\matcher\'));
addpath(genpath('.\matlab\reader\'));

%% { Data Preparation }
LidarData_path = 'G:/big-data/mulran/Sejong01/lidar/';
Groundtruth_path = 'E:/matrix_labs/lab0_loopGroundtruth_by_pose/mulran/sejong01.mat';
translations_gt = load(Groundtruth_path).groundtruth(1:end-1,1:2);
exist_loop_gt = load(Groundtruth_path).groundtruth(1:end-1,3);
nodes_num = size(exist_loop_gt,1);

%% { Parameter Settings }
max_range = 120;
rings_num = 30;
sectors_num = 60*3;
enough_node_diff_num = 150;
candidates_num = 50;

%% { Loop Closure Detection }
ringkeys = [];
ScanContexts = {};
candidates_group = {};
candidates_gaps = {};
Description_consumption = [];
Retrieval_consumption = [];

for i = 0 : (nodes_num - 1)
    fprintf('%d\n', i);
    ith_query_pcd = KITTIbin2PtcloudWithIndex_function(LidarData_path, i);
    ith_query_pcd = pointCloud(ith_query_pcd);
    t0 = tic;
    ith_q_scancontext = Ptcloud2ScanContext( ith_query_pcd, sectors_num, rings_num, max_range );
    tCount0 = toc(t0);
    Description_consumption = [Description_consumption; tCount0;];
    
    ith_q_sc = [ith_q_scancontext(:,1:30),ith_q_scancontext(:,end-29:end)];
    ScanContexts{end+1} = ith_q_sc;

    ith_q_ringkey = ScanContext2RingKey(ith_q_sc);
    ringkeys = [ringkeys; ith_q_ringkey];

    if(i <= enough_node_diff_num)
       candidates_group{end+1} = [];
       candidates_gaps{end+1} = [];
       continue;
    end
    t1 = tic;
   
    KDtree_object = createns(ringkeys(1:i-enough_node_diff_num,:),"NSMethod","kdtree"); 

    AllCandidates_index = knnsearch(KDtree_object, ith_q_ringkey,"K", candidates_num); 
    if(isempty(AllCandidates_index))
        candidates_group{end+1} = [];
        candidates_gaps{end+1} = [];
    else
        candidates_group{end+1} = AllCandidates_index;
    end
    
    candidates_gaps_ith = [];
    for ith_candidate = 1 : length(AllCandidates_index)
        ithCandidate_ID = AllCandidates_index(ith_candidate);
        ithCandidate_ScanContext = ScanContexts{ithCandidate_ID};

        Dist_to_ithCandidate = DistanceBtnScanContexts(ith_q_sc, ithCandidate_ScanContext);
        candidates_gaps_ith = [candidates_gaps_ith, Dist_to_ithCandidate];
    end
    candidates_gaps{end+1} = candidates_gaps_ith;
    tCount1 = toc(t1);
    Retrieval_consumption = [Retrieval_consumption; tCount1;];
end

results = [];
for loop_thres = 0.02:0.02:1
    detection_loop = zeros(nodes_num,2);
    
    for i = 0 : (nodes_num - 1)
        fprintf('%f, %d\n', loop_thres, i);
        ith_q_sc = ScanContexts{i+1};
        AllCandidates_index  = candidates_group{i+1};
        AllCandidates_gaps  = candidates_gaps{i+1};
        if(isempty(AllCandidates_index) || isempty(AllCandidates_gaps))
            continue;
        end
        candidateID_of_NearestDist = 0;
        CurrentQueryFrame_NearestDist = inf;
        
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

        if(candidateID_of_NearestDist == 0)
            continue;
        else
            detection_loop(i+1,1) = 1;
            
            dist_gap  = pdist([translations_gt(i + 1,:); translations_gt(candidateID_of_NearestDist,:)],'euclidean');
            if((dist_gap < 4) || (dist_gap == 4))
                detection_loop(i+1,2) = 1;
            else
                detection_loop(i+1,2) = 0;
            end
        end

    end
    
    [scan_recall,scan_precision] = RecallandPrecision(exist_loop_gt, detection_loop);
    results = [results; scan_recall,scan_precision];
end

idn = find(isnan(results));
results(idn) = 1;

disp('Description average consumption:');
disp(sum(Description_consumption)/size(Description_consumption,1));
disp('Retrieval average consumption:');
disp(sum(Retrieval_consumption)/size(Retrieval_consumption,1));

%% { Show Precision-Recall Curve }
figure(1);clf;
plot(results(:,1),results(:,2))
set(gcf,'color','w');
xlabel("Recall");ylabel("Precision");