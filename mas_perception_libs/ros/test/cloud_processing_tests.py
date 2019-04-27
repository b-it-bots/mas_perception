#!/usr/bin/env python
import os
import yaml
import numpy as np
from unittest import TestCase
from sensor_msgs.msg import Image as ImageMsg, PointCloud2
from mas_perception_libs import BoundingBox2D
from mas_perception_libs.utils import get_bag_file_msg_by_type, get_package_path, cloud_msg_to_cv_image, \
    cloud_msg_to_image_msg, crop_organized_cloud_msg, crop_cloud_to_xyz, transform_point_cloud_trans_quat, \
    PlaneSegmenter


PACKAGE = 'mas_perception_libs'
TEST_NAME = 'cloud_processing'


class CloudProcessingTest(TestCase):

    _cloud_messages = None
    _crop_boxes_num = None
    _transform_num = None
    _plane_segmenter_configs = None

    def setUp(self):
        super(CloudProcessingTest, self).setUp()

        config_file = os.path.join('config', TEST_NAME + '.yaml')
        self.assertTrue(os.path.exists(config_file), 'test configuration file does not exist: ' + config_file)

        with open(config_file, 'r') as infile:
            configs = yaml.load(infile)

        # load point clouds from bag file
        bag_file_config = configs['bag_file']
        bag_file_path = get_package_path(bag_file_config['package'], *bag_file_config['path'])
        self.assertTrue(os.path.exists(bag_file_path),
                        'bag file containing point clouds does not exist: ' + bag_file_path)
        self._cloud_messages = get_bag_file_msg_by_type(bag_file_path, 'sensor_msgs/PointCloud2')
        self.assertTrue(self._cloud_messages and len(self._cloud_messages) > 0,
                        "no 'sensor_msgs/PointCloud2' message in bag file: " + bag_file_path)

        # load configurations for cloud utility functions
        self._crop_boxes_num = configs['crop_boxes_num']
        self._transform_num = configs['transform_num']

        # load configurations for plane segmentation
        self._plane_segmenter_configs = configs['plane_segmenter']

    def test_cloud_msg_to_image_conversion(self):
        for cloud_msg in self._cloud_messages:
            cv_image = cloud_msg_to_cv_image(cloud_msg)
            self.assertIs(type(cv_image), np.ndarray,
                          "'cloud_msg_to_cv_image' does not return type 'numpy.ndarray'")
            image_msg = cloud_msg_to_image_msg(cloud_msg)
            self.assertIs(type(image_msg), ImageMsg,
                          "'cloud_msg_to_image_msg' does not return type 'sensor_msgs/Image'")

    def test_cloud_cropping(self):
        for cloud_msg in self._cloud_messages:
            np.random.seed(1234)
            cloud_width = cloud_msg.width
            cloud_height = cloud_msg.height
            for i in range(self._crop_boxes_num):
                x = np.random.randint(0, cloud_width)
                y = np.random.randint(0, cloud_height)
                if i < self._crop_boxes_num / 2:
                    # normal case
                    width = np.random.randint(0, cloud_width - x)
                    height = np.random.randint(0, cloud_height - y)
                else:
                    # box has region outside of frame dimension
                    width = np.random.randint(cloud_width - x, cloud_width)
                    height = np.random.randint(cloud_height - y, cloud_height)
                box = BoundingBox2D(box_geometry=(x, y, width, height))
                cropped_cloud = crop_organized_cloud_msg(cloud_msg, box)
                self.assertIs(type(cropped_cloud), PointCloud2,
                              "'crop_organized_cloud_msg' does not return type 'sensor_msgs/PointCloud2'")
                self.assertTrue(box.x + cropped_cloud.width <= cloud_width and
                                box.y + cropped_cloud.height <= cloud_height,
                                "'crop_organized_cloud_msg' does not handle large box dimensions correctly")

                cropped_xyz = crop_cloud_to_xyz(cloud_msg, box)
                self.assertTrue(type(cropped_xyz) == np.ndarray and len(cropped_xyz.shape) == 3,
                                "'crop_cloud_to_xyz' did not return a 3 dimensional numpy array")
                self.assertTrue(box.x + cropped_xyz.shape[0] <= cloud_width and
                                box.y + cropped_xyz.shape[1] <= cloud_height,
                                "'crop_cloud_to_xyz' does not handle large box dimensions correctly")

    def test_transform_point_cloud_trans_quat(self):
        frame_name = 'test_frame'
        for cloud_msg in self._cloud_messages:
            np.random.seed(1234)
            for _ in range(self._transform_num):
                translation = np.random.rand(3) * 1.0       # scale to 1 meter
                rotation = np.random.rand(4)                # quaternion
                transformed_cloud = transform_point_cloud_trans_quat(cloud_msg, translation, rotation, frame_name)
                self.assertIs(type(transformed_cloud), PointCloud2,
                              "'transform_point_cloud_trans_quat' does not return type 'sensor_msgs/PointCloud2'")
                self.assertTrue(transformed_cloud.header.frame_id == frame_name)

    def test_plane_segmenter(self):
        plane_segmenter = PlaneSegmenter()

        # set plane segmentation configurations
        config_file_info = self._plane_segmenter_configs['config_file']
        config_file_path = get_package_path(config_file_info['package'], *config_file_info['path'])
        self.assertTrue(os.path.exists(config_file_path),
                        'config file for PlaneSegmenter does not exist: ' + config_file_path)
        with open(config_file_path) as infile:
            configs = yaml.load(infile)
        plane_segmenter.set_params(configs)

        # extract transformation info for point clouds
        transform_info = self._plane_segmenter_configs['target_frame']
        translation = transform_info['translation']
        rotation = transform_info['rotation']
        frame_name = transform_info['name']
        for cloud_msg in self._cloud_messages:
            transformed_cloud = transform_point_cloud_trans_quat(cloud_msg, translation, rotation, frame_name)
            # TODO (minhnh) test if filtering does what it's supposed to do, test edge cases
            _ = plane_segmenter.filter_cloud(transformed_cloud)
            # TODO(minhnh) test multiple plane segmentation, test if returned cloud is filtered
            plane_list, _ = plane_segmenter.find_planes(transformed_cloud)
            self.assertTrue(len(plane_list.planes) > 0, 'plane segmentation did not detect any plane')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PACKAGE, TEST_NAME, CloudProcessingTest)
