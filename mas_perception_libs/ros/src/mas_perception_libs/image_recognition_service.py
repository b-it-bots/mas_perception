import os
from importlib import import_module
import rospy
from mcr_perception_msgs.srv import ImageRecognition, ImageRecognitionRequest, ImageRecognitionResponse
from image_classifier import ImageClassifier


class RecognizeImageService(object):
    def __init__(self, classifer_class, model_dir, service_name):
        if not issubclass(classifer_class, ImageClassifier):
            raise ValueError('classifer is not of ImageClassifier type')

        self._classifier_class = classifer_class
        self._model_dir = model_dir

        # dictionary of classifiers - key is the model_path
        self._classifiers = {}
        self._recog_service = rospy.Service(service_name, ImageRecognition, self.handle_recognize_image)

    def handle_recognize_image(self, req):
        print('\nnumber of images: ' + str(len(req.images)) + '\n')
        if req.model_name not in self._classifiers:
            model_path = os.path.join(self._model_dir, req.model_name + '.h5')
            class_file = os.path.join(self._model_dir, req.model_name + '.txt')
            rospy.loginfo('recognition model path: ' + model_path)
            rospy.loginfo('recognition class file path: ' + class_file)
            self._classifiers[req.model_name] = self._classifier_class(model_path=model_path, class_file=class_file)

        indices, classes, probabilities = self._classifiers[req.model_name].classify(req.images)
        response = ImageRecognitionResponse()
        response.indices = indices
        response.classes = classes
        response.probabilities = probabilities
        return response


class ImageRecognitionServiceProxy(object):
    def __init__(self, service_name, model_name,  preprocess_input_module=None):
        rospy.wait_for_service(service_name, timeout=5.0)
        try:
            self._recog_proxy = rospy.ServiceProxy(service_name, ImageRecognition)
        except rospy.ServiceException as e:
            rospy.logerr('failed to get proxy for service ' + e.message)
            raise

        self._model_name = model_name

        self._preprocess_input_func = None
        if preprocess_input_module:
            self._preprocess_input_func = getattr(import_module(preprocess_input_module), 'preprocess_input')

    def classify_image_messages(self, image_messages, done_callback_func=None):
        request = ImageRecognitionRequest()
        request.images = image_messages
        request.model_name = self._model_name
        response = self._recog_proxy(request)
        if done_callback_func is not None:
            done_callback_func()

        return response.indices, response.classes, response.probabilities
