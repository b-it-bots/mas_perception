import os
from abc import ABCMeta, abstractmethod
import numpy as np
from cv_bridge import CvBridge
from utils import get_classes_in_data_dir, process_image_message


class ImageClassifier(object):
    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        self._classes = kwargs.get('classes', None)

        if self._classes is None:
            class_file = kwargs.get('class_file', None)
            if class_file is not None and os.path.exists(class_file):
                self._classes = ImageClassifier.read_classes_from_file(class_file)

        if self._classes is None:
            data_dir = kwargs.get('data_dir', None)
            if data_dir is None:
                raise ValueError('no class definition specified')
            if not os.path.exists(data_dir):
                raise ValueError('Directory does not exist: ' + data_dir)

            self._classes = get_classes_in_data_dir(data_dir)

    @property
    def classes(self):
        return self._classes

    @abstractmethod
    def classify(self, image_messages):
        pass

    @staticmethod
    def write_classes_to_file(classes, outfile_path):
        with open(outfile_path, 'w') as outfile:
            outfile.write('\n'.join(classes))

    @staticmethod
    def read_classes_from_file(infile):
        with open(infile) as f:
            content = f.readlines()
            return [x.strip() for x in content]


class ImageClassifierTest(ImageClassifier):
    def __init__(self, **kwargs):
        super(ImageClassifierTest, self).__init__(**kwargs)

    def classify(self, image_messages):
        import random
        indices = list(range(len(image_messages)))
        classes = [self.classes[random.randint(0, len(self.classes) - 1)] for _ in indices]
        probabilities = [random.random() for _ in indices]
        return indices, classes, probabilities


class KerasImageClassifier(ImageClassifier):
    def __init__(self, **kwargs):
        from keras.models import Model, load_model

        super(KerasImageClassifier, self).__init__(**kwargs)

        self._model = kwargs.get('model', None)
        model_path = kwargs.get('model_path', None)
        if self._model is None:
            if model_path is not None:
                self._model = load_model(model_path)
                # see https://github.com/keras-team/keras/issues/6462
                self._model._make_predict_function()
            else:
                raise ValueError('No model object or path passed received')

        if not isinstance(self._model, Model):
            raise ValueError('model is not a Keras Model object')

        if len(self._classes) != self._model.output_shape[-1]:
            raise ValueError('number of classes ({0}) does not match model output shape ({1})'
                             .format(len(self._classes), self._model.output_shape[-1]))

        self._img_preprocess_func = kwargs.get('img_preprocess_func', None)

        # assume input shape is 3D with channel dimension to be 3
        self._target_size = tuple(i for i in self._model.input_shape if i != 3 and i is not None)

        # CvBridge for ROS image conversion
        self._cv_bridge = CvBridge()

    def classify(self, image_messages):
        if len(image_messages) == 0:
            return [], [], []

        np_images = [process_image_message(msg, self._cv_bridge, self._target_size, self._img_preprocess_func)
                     for msg in image_messages]

        image_array = []
        indices = []
        for i in range(len(np_images)):
            if np_images[i] is None:
                continue

            image_array.append(np_images[i])
            indices.append(i)

        image_array = np.array(image_array)
        preds = self._model.predict(image_array)
        class_indices = np.argmax(preds, axis=1)
        confidences = np.max(preds, axis=1)
        predicted_classes = [self._classes[i] for i in class_indices]

        return indices, predicted_classes, confidences
