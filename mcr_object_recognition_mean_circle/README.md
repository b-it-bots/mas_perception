The current model is trained with sklearn version 0.14.1.

# How to train a model
## Install sklearn

Run:
pip install sklearn

## Install Cython

Run:
sudo pip install Cython

## Get Python-pcl

Clone this repository from b-it-bots. Run:
git clone git@github.com:b-it-bots/python-pcl.git

## Install Python-pcl

Go inside the python-pcl folder and run the following:
sudo python setup.py install

## Training a classifier

Find the train_classifier.py file inside your catkin workspace:

/home/*user*/*catkin_workspace_name*/src/mas_perception/mcr_object_recognition_mean_circle/ros/tools/train_classifier.py

And run it with the given parameters:

python train_classifier.py --dataset *add path to the training folder here*
