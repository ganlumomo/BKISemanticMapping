# BKISemanticMapping
Bayesian Spatial Kernel Smoothing for Scalable Dense Semantic Mapping

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_csm.png" width="300"><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_bki.png" width="300">
<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_csm_variance.png" width="300"><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_bki_variance.png" width="300">


| Sequence  | Method | Car | Bicycle | Motorcycle | Truck | Other Vehicle | Person | Bicyclist | Motorcyclist | Road | Parking | Sidewalk | Other Ground | Building | Fence | Vegetation | Trunk | Terrain | Pole | Traffic Sign | Average | 
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 00 | Sq.-KNN | 92.1 | 18.3 | 55.0 | 76.5 | 62.9 | 34.2 | 52.0 | 61.4 | 94.7 | 71.0 | 87.9 | 1.2 | 89.8 | 54.6 | 82.2 | 53.1 | 79.3 | 38.6 | 51.5 | 60.9 |
| | S-CMS | 95.6 | 23.5 | 69.8 | 88.3 | 74.4 | 47.9 | 71.6 | 56.9 | 96.3 | 78.1 | 91.2 | 3.1 | 93.6 | 64.2 | 87.4 | 70.1 | 83.5 | 61.1 | 70.7 | 69.9 |
| | S-BKI | 96.9 | 26.5 | 75.8 | 93.5 | 80.1 | 61.5 | 77.5 | 71.0 | 96.2 | 79.2 | 91.5 | 6.6 | 94.6 | 66.5 | 88.9 | 73.4 | 84.5 | 65.8 | 76.2 | 75.0 |
| 01 | Sq.-KNN | 83.8 | n/a | n/a | n/a | 82.9 | n/a | n/a | 67.9 | 92.6 | n/a | n/a | 70.5 | 58.0 | 71.4 | 72.1 | 18.0 | 71.5 | 21.8 | 68.9 | 64.0 |
| | S-CMS | 89.8 | n/a | n/a | n/a | 91.0 | n/a | n/a | 70.3 | 93.4 | n/a | n/a | 74.2 | 64.4 | 73.8 | 75.1 | 26.3 | 74.7 | 31.9 | 78.7 | 70.3 |
| | S-BKI | 91.0 | n/a | n/a | n/a | 96.0 | n/a | n/a | 70.7 | 94.3 | n/a | n/a | 75.2 | 67.1 | 75.1 | 76.4 | 30.6 | 76.1 | 36.2 | 81.4 | 72.5 |
| 02 | Sq.-KNN | 90.9 | 14.5 | 50.8 | n/a | 56.4 | 38.6 | n/a | 59.9 | 93.9 | 68.1 | 84.9 | 50.9 | 79.1 | 66.1 | 82.5 | 48.9 | 68.3 | 25.7 | 35.9 | 59.7 |
| | S-CSM | 95.4 | 28.5 | 73.4 | n/a | 80.3 | 60.3 | n/a | 75.1 | 94.8 | 74.4 | 87.4 | 61.7 | 85.0 | 71.8 | 86.7 | 66.9 | 72.9 | 43.5 | 55.7 | 71.4 |
| | S-BKI | 95.8 | 31.1 | 76.4 | n/a | 83.3 | 62.5 | n/a | 79.5 | 94.8 | 75.0 | 87.4 | 63.6 | 85.6 | 72.1 | 87.1 | 68.8 | 73.4 | 45.9 | 60.4 | 73.1 |
| 03 | Sq.-KNN | 88.4 | 21.9 | n/a | 12.4 | 60.1 | 16.3 | n/a | n/a | 92.8 | 57.9 | 83.2 | n/a | 77.4 | 70.1 | 79.3 | 41.6 | 62.3 | 35.9 | 47.3 | 56.5 |
| | S-CSM | 92.4 | 29.7 | n/a | 23.1 | 65.4 | 17.6 | n/a | n/a | 94.3 | 69.4 | 86.9 | n/a | 80.4 | 73.8 | 83.2 | 52.3 | 66.9 | 53.5 | 62.0 | 63.0 |
| | S-BKI | 94.5 | 42.4 | n/a | 48.8 | 73.6 | 23.8 | n/a | n/a | 94.3 | 73.2 | 87.2 | n/a | 82.1 | 74.7 | 84.1 | 55.7 | 67.4 | 57.3 | 66.7 | 68.0 |
| 04 | Sq.-KNN | 84.9 | n/a | n/a | n/a | 68.1 | 20.8 | n/a | n/a | 95.8 | 26.1 | 68.4 | 61.5 | 49.3 | 76.4 | 82.6 | 14.0 | 67.6 | 36.0 | 44.6 | 56.9 |
| | S-CSM | 88.3 | n/a | n/a | n/a | 71.2 | 23.2 | n/a | n/a | 96.5 | 40.5 | 72.5 | 64.0 | 52.1 | 78.5 | 85.5 | 19.4 | 72.5 | 50.8 | 57.6 | 62.3 |
| | S-BKI | 87.7 | n/a | n/a | n/a | 82.5 | 37.3 | n/a | n/a | 96.2 | 55.7 | 72.3 | 68.3 | 56.9 | 80.3 | 87.1 | 24.4 | 72.7 | 55.5 | 67.0 | 67.5 |
| 05 | Sq.-KNN | 89.1 | 8.6 | 15.4 | 82.5 | 70.9 | 31.0 | 55.0 | n/a | 94.7 | 84.8 | 85.0 | 61.5 | 87.0 | 72.4 | 75.5 | 30.3 | 64.6 | 27.6 | 39.5 | 59.8 |
| | S-CSM | 93.4 | 15.4 | 28.9 | 86.4 | 78.4 | 39.8 | 69.4 | n/a | 96.4 | 90.1 | 88.5 | 70.0 | 90.9 | 77.7 | 81.2 | 46.8 | 69.5 | 47.6 | 57.4 | 68.2 |
| | S-BKI | 93.2 | 27.4 | 46.0 | 89.0 | 84.1 | 47.5 | 83.3 | n/a | 94.2 | 88.0 | 83.6 | 75.2 | 92.4 | 75.3 | 82.1 | 53.5 | 69.5 | 50.2 | 63.3 | 72.1 |
| 06 | Sq.KNN | 85.4 | 17.1 | 50.2 | 86.7 | 66.1 | 27.6 | 64.3 | n/a | 87.6 | 56.0 | 74.9 | 66.5 | 83.9 | 38.4 | 61.9 | 32.0 | 89.5 | 40.1 | 52.7 | 60.1 |
| | S-CSM | 91.8 | 22.7 | 62.5 | 89.8 | 75.4 | 43.3 | 92.1 | n/a | 91.1 | 68.2 | 80.4 | 70.5 | 89.4 | 49.3 | 69.7 | 50.1 | 92.2 | 60.0 | 77.9 | 70.9 |
| | S-BKI | 92.6 | 28.7 | 67.9 | 93.5 | 81.4 | 62.7 | 95.4 | n/a | 90.3 | 70.7 | 79.9 | 71.8 | 91.7 | 53.6 | 73.7 | 54.7 | 91.9 | 66.4 | 84.8 | 75.1 |
| 07 | Sq.-KNN | 92.4 | 21.3 | 64.0 | 83.6 | 69.8 | 53.2 | 63.6  | n/a | 93.9 | 75.9 | 89.3 | n/a | 90.9 | 59.7 | 76.5 | 45.9 | 82.8 | 40.2 | 54.0 | 68.1 |
| | S-CSM | 94.9 | 25.9 | 76.8 | 82.6 | 81.5 | 64.2 | 88.0 | n/a | 95.8 | 80.9 | 92.0 | n/a | 93.8 | 66.6 | 80.8 | 59.8 | 84.7 | 55.4 | 73.2 | 76.2 |
| | S-BKI | 93.8 | 29.2 | 80.2 | 82.7 | 87.8 | 70.1 | 92.7 | n/a | 93.9 | 77.0 | 87.7 | n/a | 94.1 | 63.4 | 81.4 | 84.1 | 84.5 | 53.2 | 77.6 | 77.2 |
| 08 | Sq.-KNN | 86.7 | 14.4 | 24.6 | 21.0 | 23.3 | 23.5 | 40.9 | n/a | 90.1 | 32.4 | 74.8 | 1.2 | 79.6 | 42.7 | 79.2 | 36.5 | 71.1 | 28.3 | 24.8 | 44.1 |
| | S-CSM | 90.5 | 23.0 | 34.9 | 26.8 | 29.1 | 32.4 | 49.4 | n/a | 92.6 | 38.7 | 79.0 | 1.1 | 84.6 | 51.6 | 83.3 | 48.3 | 72.9 | 44.1 | 31.6 | 50.8 |
| | S-BKI | 92.3 | 30.0 | 39.7 | 29.3 | 32.1 | 38.8 | 54.7 | n/a | 92.9 | 40.9 | 79.9 | 1.1 | 86.6 | 54.6 | 84.9 | 52.3 | 74.2 | 47.9 | 34.7 | 53.7 |
| 09 | Sq.-KNN | 89.2 | 5.3 | 48.0 | 79.8 | 61.3 | 37.3 | n/a | n/a | 91.0 | 59.0 | 79.9 | 38.9 | 80.9 | 62.9 | 77.0 | 32.3 | 61.7 | 31.8 | 52.6 | 58.2 |
| | S-CSM | 93.9 | 12.2 | 71.9 | 85.6 | 71.6 | 47.5 | n/a | n/a | 91.8 | 67.0 | 83.1 | 23.4 | 88.9 | 65.7 | 82.6 | 42.9 | 64.9 | 52.4 | 53.0 | 64.6 |
| | S-BKI | 96.0 | 22.8 | 80.2 | 90.5 | 79.7 | 60.7 | n/a | n/a | 91.7 | 70.0 | 83.8 | 30.7 | 90.8 | 69.1 | 84.0 | 46.3 | 66.0 | 59.1 | 58.2 | 69.4 |
| 10 | Sq.-KNN | 84.0 | 8.1 | 36.2 | 49.3 | 10.2 | 40.9 | n/a | n/a | 89.4 | 59.6 | 78.5 | 42.7 | 76.7 | 64.2 | 77.6 | 29.0 | 67.8 | 30.7 | 47.9 | 52.0 |
| | S-CSM | 91.0 | 14.6 | 51.8 | 67.7 | 16.6 | 52.8 | n/a | n/a | 92.1 | 69.7 | 83.7 | 51.3 | 81.7 | 70.0 | 82.2 | 43.3 | 72.4 | 51.7 | 64.1 | 62.1 |
| | S-BKI | 93.8 | 24.6 | 60.3 | 76.2 | 21.2 | 65.0 | n/a | n/a | 92.3 | 73.4 | 84.8 | 54.5 | 83.0 | 71.2 | 83.4 | 47.3 | 73.4 | 56.2 | 67.9 | 66.4 |
| Average | Sq.-KNN | 87.9 | 14.4 | 43.0 | 61.5 | 57.5 | 32.3 | 55.2 | 63.1 | 92.4 | 59.1 | 80.7 | 43.9 | 77.5 | 61.7 | 76.9 | 34.7 | 71.5 | 32.4 | 47.2 | 57.6 |
| | S-CSM | 92.5 | 21.7 | 58.7 | 68.8 | 66.8 | 42.9 | 74.1 | 67.4 | 94.1 | 67.7 | 84.5 | 46.6 | 82.3 | 67.5 | 81.6 | 47.8 | 75.2 | 50.2 | 62.0 | 65.9 |
| | S-BKI | 93.4 | 29.2 | 65.8 | 75.4 | 72.9 | 93.0 | 80.7 | 73.7 | 93.7 | 72.3 | 83.8 | 49.0 | 84.1 | 68.7 | 83.0 | 53.7 | 75.8 | 54.0 | 67.1 | 72.1 |


## Getting Started

### Building with catkin

```bash
catkin_ws/src$ git clone https://github.com/ganlumomo/BKISemanticMapping
catkin_ws/src$ cd ..
catkin_ws$ catkin_make
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Building using Intel C++ compiler (optional for better speed performance)
```bash
catkin_ws$ source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64
catkin_ws$ catkin_make -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Running the Demo

```bash
$ roslaunch semantic_bki toy_example_node.launch
```


## Semantic Mapping using KITTI dataset

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_05.png" width=320><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_15.png" width=540>

### Download Data
Please download [data_kitti_15](https://drive.google.com/file/d/1dIHRrsA7rZSRJ6M9Uz_75ZxcHHY96Gmb/view?usp=sharing) and uncompress it into the data folder.

### Running
```bash
$ roslaunch semantic_bki kitti_node.launch
```
You will see semantic map in RViz. It also projects 3D grid onto 2D image for evaluation, stored at data/data_kitti_05/reproj_img.

### Evaluation
Evaluation code is provided in kitti_evaluation.ipynb. You may modify the directory names to run it.


## Semantic Mapping using SemanticKITTI dataset

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/semantic_kitti_seq05.png" width=430><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/semantic_kitti_seq04.png" width=430>

### Download Data
Please download [semantickitti_04](https://drive.google.com/file/d/19Dv1jQqf-VGKS2qvbygFlUzQoSvu17E5/view?usp=sharing) and uncompress it into the data folder.

### Running
```bash
$ roslaunch semantic_bki semantickitti_node.launch
```
You will see semantic map in RViz. It also query each ground truth point for evaluation, stored at data/semantickitti_04/evaluations.

### Evaluation
Evaluation code is provided in semantickitti_evaluation.ipynb. You may modify the directory names to run it, or follow the guideline in [semantic-kitti-api](https://github.com/PRBonn/semantic-kitti-api) for evaluation.


## Relevant Publications

If you found this code useful, please cite the following:

Bayesian Spatial Kernel Smoothing for Scalable Dense Semantic Mapping ([PDF](https://arxiv.org/pdf/1909.04631.pdf))
```
@article{gan2019bayesian,
  title={Bayesian Spatial Kernel Smoothing for Scalable Dense Semantic Mapping},
  author={Gan, Lu and Zhang, Ray and Grizzle, Jessy W and Eustice, Ryan M and Ghaffari, Maani},
  journal={arXiv preprint arXiv:1909.04631},
  year={2019}
}
```

Learning-Aided 3-D Occupancy Mapping with Bayesian Generalized Kernel Inference ([PDF](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8713569))
```
@article{Doherty2019,
  doi = {10.1109/tro.2019.2912487},
  url = {https://doi.org/10.1109/tro.2019.2912487},
  year = {2019},
  publisher = {Institute of Electrical and Electronics Engineers ({IEEE})},
  pages = {1--14},
  author = {Kevin Doherty and Tixiao Shan and Jinkun Wang and Brendan Englot},
  title = {Learning-Aided 3-D Occupancy Mapping With Bayesian Generalized Kernel Inference},
  journal = {{IEEE} Transactions on Robotics}
}
```
