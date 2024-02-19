LidarData_path = 'G:/big-data/Semantic-KITTI/LIDAR/dataset/sequences/00/velodyne/'; % 一帧点云的文件路径
pcd = KITTIbin2PtcloudWithIndex_function(LidarData_path, 0);
pcd2 = KITTIbin2PtcloudWithIndex_function(LidarData_path, 5);
pcd = pointCloud(pcd);
pcd2 = pointCloud(pcd2);

[ith_q_frescontext,ith_q_ringkey] = FreSCo(pcd);
[ith_q_frescontext2,ith_q_ringkey2] = FreSCo(pcd2);

[best_offset, Dist_to_ithCandidate] = computeFrescoDist(ith_q_frescontext, ith_q_frescontext2);
