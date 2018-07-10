import numpy as np
import cv2

from mas_perception_libs._cpp_wrapper import _draw_labeled_boxes


def draw_labeled_boxes(image, boxes, thickness=2, font_scale=1.0, copy=True):
    if copy:
        drawn_image = image.copy()
    else:
        drawn_image = image
    return _draw_labeled_boxes(drawn_image, boxes, thickness, font_scale)


def bgr_dict_from_classes(classes):
    """
    get dictionary of BGR colors span over HSV hue range from a list of classes

    :param classes: list of colors
    :return: dictionary {key=class name, value=color}
    """
    class_num = len(classes)
    np.random.seed(1234)
    np.random.shuffle(classes)

    hue = (np.linspace(0, 1, class_num) * 255).astype(np.uint8).reshape(1, class_num, 1)
    sat = np.ones((1, class_num, 1), dtype=np.uint8) * 127      # saturation 50%
    bright = np.ones((1, class_num, 1), dtype=np.uint8) * 127   # value/brightness 50%

    hsv = np.concatenate((hue, sat, bright), axis=2)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).astype(int)

    return {class_name: tuple(bgr[0][index]) for index, class_name in enumerate(classes)}
