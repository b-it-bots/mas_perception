The current model is trained with sklearn version 0.14.1.

# How to train a model
## Install sklearn

Run:
```sh
sudo pip install sklearn
```
## Install Cython

Run:
```sh
sudo pip install Cython
```
## Get Python-pcl

Clone this repository from b-it-bots. Run:
```sh
git clone git@github.com:b-it-bots/python-pcl.git
```
## Install Python-pcl

Go inside the python-pcl folder and run the following:
```sh
sudo python setup.py install
```
## Training a classifier

Find the train_classifier.py file inside your catkin workspace:
```sh
/home/*user*/*catkin_workspace_name*/src/mas_perception/mcr_object_recognition_mean_circle/ros/tools/train_classifier.py
```
And run it with the given parameters:
```sh
python train_classifier.py --dataset *add path to the training folder here*
```
