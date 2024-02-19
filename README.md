# Scattering Context: Dual-view Encoding-Based 3D Descriptor and Fast Search Strategy for Loop Closure Detection
### State Key Lab of Rail Traffic Control & Safety (BJTU)
This repo contains the source code and dataset for our paper:
<br>
[**PMF-SLAM: Pose-guided and Multi-scale Feature Interaction Based Semantic SLAM for Autonomous Wheel Loader**](https://ieeexplore.ieee.org/document/10405860/)
<br>
[paper](https://ieeexplore.ieee.org/document/10405860/)
![SAMe3d](/Figs/Backbone_work.jpg)

# Data Download and Preparation
To evaluate the LCD performance, you will need to **download** the required datasets.

- SemanticKITTI - [Baidu Drive](https://pan.baidu.com/s/1LL2LItLEQpOt4HLWodTpWQ?pwd=qaos)(access code: qaos)
- We have provided the groundtruth Pose of this dataset (in Groundtruth_pose folder).
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
        └── 06/
	       └── ...

```
- Sany (ours) - [Baidu Drive](https://pan.baidu.com/s/10F5ezH4LgT9glGZ_A16BuQ)(Acquiring copyright)
```
./
├── 
├── ...
└── data_path/
    ├──sany
        ├── Mixing_station(MS)/ # Mixing station scene.       
        │   	└── sequences/
	│		├── 00/ # for training          
	│		│   ├── velodyne/	
	│		|   |	├── xxx.bin
	│		|   |	├── xxx.bin
	│		|   |	└── ...
	│		│   └── labels/ 
	│		|       ├── xxx.label
	│		|       ├── xxx.label
	│		|       └── ...
	│		├── 01/ # for validation
	│		└── 02/ # for testing
        └── points(PG)/ # Proving ground scene.
	   	└── sequences/
			├── 00/ # for training          
			|   └── ...
			├── 01/ # for validation
			└── 02/ # for testing
```
- nuScenes - [Baidu Drive](https://pan.baidu.com/s/1TF80roYGuIm6FhDo0DBmgg?pwd=ai67)(access code: ai67)
```
./
├── 
├── ...
└── data_path/
    ├──nuscenes
        ├── lidarseg/   
        ├── maps/
	├── samples/
        │   └── LIDAR_TOP/	
        |    	├── n008-2018-05-21-11-06-59-0400_LIDAR_TOP_1526915243547836.pcd.bin
        |    	└── ...
	├── v1.0-trainval/
	├── nuscenes_infos_train.pkl/
	├── nuscenes_infos_val.pkl/
        └── nuscenes_infos_test.pkl/
```

# Step3.Train & Validate

Note: In you virtual env(e.g. SAMe3d) establied from above Step1 to run the belows.
## Pretrained Models
-- We provide a pretrained model [LINK](https://pan.baidu.com/s/15_1Z8cfxTB6reRiA05bG8Q?pwd=wf40) (access code: wf40)

- To train on **Sany-Mixing Station dataset**, run
```
 python train.py --config_path config/sany_mixing_parameters.yaml --device 0
```
- To train on **Sany-Proving ground dataset**, run
```
 python train.py --config_path config/sany_points_parameters.yaml --device 0
```
- To train on **nuScenes dataset**, run
```
 python train_nuscene.py --config_path config/nuScenes.yaml --device 0
```
- To train on **SemanticKITTI dataset**, run
```
 python train.py --config_path config/parameters.yaml --device 0
```

## Acknowledgments
We thanks for the opensource codebases, [Cylinder3D](https://github.com/xinge008/Cylinder3D) and [spconv](https://github.com/traveller59/spconv)
