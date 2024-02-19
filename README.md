# Scattering Context: Dual-view Encoding-Based 3D Descriptor and Fast Search Strategy for Loop Closure Detection
### State Key Lab of Rail Traffic Control & Safety, Beijing Jiaotong University(BJTU)
This repo contains the source code and dataset for our paper:
<br>
[**PMF-SLAM: Pose-guided and Multi-scale Feature Interaction Based Semantic SLAM for Autonomous Wheel Loader**](https://ieeexplore.ieee.org/document/10405860/)
<br>
[paper](https://ieeexplore.ieee.org/document/10405860/)
![SAMe3d](/Figs/Backbone_work.jpg)

# Data Download and Preparation
To evaluate the LCD performance, you will need to **download** the required datasets.

- SemanticKITTI Dataset- [Baidu Drive](https://pan.baidu.com/s/1LL2LItLEQpOt4HLWodTpWQ?pwd=qaos)(access code: qaos).
<br> We have provided the groundtruth Pose of this dataset in Groundtruth_pose folder.
```
./
├── 
├── ...
└── data_path/
    ├──sequences
        ├── 00/          
        │   ├── velodyne/	
        |   |	├── 000000.bin
        |   |	├── 000001.bin
        |   |	└── ...
        │   └── labels/ 
        |       ├── 000000.label
        |       ├── 000001.label
        |       └── ...
        ├── 02/ 
        ├── 05/ 
        │   ├── velodyne/	
        |    	├── 000000.bin
        |    	├── 000001.bin
        |    	└── ...
        ├── 06/
        ├── ...

```
- MulRan Dataset- [download](https://sites.google.com/view/mulran-pr/home).
<br> We have provided the groundtruth Pose of this dataset in Groundtruth_pose folder. Sejong01 sequence can also be downloaded from [Google Drive](https://drive.google.com/file/d/17Lo-fgDgkeLxDTVdY-Sn7pj0xoeCL9nE/view?usp=sharing).
```
./
├── 
├── ...
└── data_path/
    ├──mulran
        ├── Sejong01/      
        │   	└── lidar/
	│		├── 000000.bin          
        |               ├── 000001.bin
	│		└── ...
        ├── Riverside02/
	|   	└── lidar/
	|		├── 000000.bin          
	|		├── 000001.bin
	|		└── ...
        ├── ...
```
- Freiburg Campus Dataset - [download](https://drive.google.com/file/d/1eJFcMEEcNQynoNqeYoyBIZi0I4Gs1mH6/view?usp=sharing).
<br> We have provided the groundtruth Pose of this dataset in Groundtruth_pose folder.
```
./
├── 
├── ...
└── data_path/
    ├──Freiburg
        └── lidar/   	
            	├── 000000.bin
                ├── 000001.bin
            	└── ...
```
- UrbanLoco Dataset - [download](https://github.com/weisongwen/UrbanLoco).
<br> We have provided the groundtruth Pose of this dataset in Groundtruth_pose folder. CALombardStreet sequence can also be downloaded from [Google Drive](https://drive.google.com/file/d/1-av6k8BvpDnchmYWyNnlD4foK5D2vCEe/view?usp=sharing).
```
./
├── 
├── ...
└── data_path/
    ├──UrbanLoco
        └── CALombardStreet/      
           	└── lidar/
                       ├── 000000.bin          
                       ├── 000001.bin
                       └── ...
```

# Run & Validate STC algorithm
Scattering_Context folder is our proposed method. 
<br> Next, taking the method of this paper as an example, the experimental process is described.

- To test on **KITTI dataset**, Open Scattering_Context/main/main1_about_KITTI_Dataset.m file
```
Modify the specific sequence path in line 13.
Modify the groundtruth pose file path in line 14.
```
- To test on **MulRan dataset**, Open Scattering_Context/main/main2_about_KITTI_Dataset.m file
```
Modify the specific sequence path in line 13.
Modify the groundtruth pose file path in line 14.
```
- To test on **Freiburg Campus dataset**, Open Scattering_Context/main/main3_about_KITTI_Dataset.m file
```
Modify the specific sequence path in line 13.
Modify the groundtruth pose file path in line 14.
```
- To test on **UrbanLoco dataset**, Open Scattering_Context/main/main4_about_KITTI_Dataset.m file
```
Modify the specific sequence path in line 13.
Modify the groundtruth pose file path in line 14.
```
# Run & Validate other algorithms (M2DP, NDD, Scan Context, FreSCo)
Similar to the above step, it is performed under the corresponding folder (e.g., Other_SOTA_algorithms folder). Other_SOTA_algorithms folder is other algorithms.

## Acknowledgments
We thanks for the opensource codebases, [M2DP](https://github.com/LiHeUA/M2DP), [Scan Context](https://github.com/asdfghjkl623/scancontext/tree/master/matlab
), [NDD](https://github.com/zhouruihao1001/NDD) and [FreSCo](https://github.com/soytony/FreSCo).
