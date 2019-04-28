import os
from importlib import import_module
import rospy
from mcr_perception_msgs.srv import RecognizeImage, RecognizeImageRequest, RecognizeImageResponse
from image_classifier import ImageClassifier


class RecognizeImageService(object):
    """
    Image recognition service which uses ImageClassifier instances to classify images
    """
    def __init__(self, classifier_class, model_dir, service_name):
        """
        :param classifier_class: ImageClassifier extension for classifying images
        :param model_dir: path to the model directory
        :type model_dir: str
        :param service_name: name of recognition service
        :type service_name: str
        """
        if not issubclass(classifier_class, ImageClassifier):
            raise ValueError('classifer is not of ImageClassifier type')

        self._classifier_class = classifier_class
        self._model_dir = model_dir

        # dictionary of classifiers - key is the model_path
        self._classifiers = {}
        self._recog_service = rospy.Service(service_name, RecognizeImage, self.handle_recognize_image)

    def handle_recognize_image(self, req):
        """
        Callback for recognition service

        :param req: service request
        :type req: RecognizeImageRequest
        :rtype: RecognizeImageResponse
        """
        rospy.loginfo('number of images to recognize: ' + str(len(req.images)))
        if req.model_name not in self._classifiers:
            model_path = os.path.join(self._model_dir, req.model_name + '.h5')
            class_file = os.path.join(self._model_dir, req.model_name + '.txt')
            rospy.loginfo('recognition model path: ' + model_path)
            rospy.loginfo('recognition class file path: ' + class_file)
            self._classifiers[req.model_name] = self._classifier_class(model_path=model_path, class_file=class_file)

        indices, classes, probabilities = self._classifiers[req.model_name].classify(req.images)
        response = RecognizeImageResponse()
        response.indices = indices
        response.classes = classes
        response.probabilities = probabilities
        return response


class RecognizeImageServiceProxy(object):
    """
    Interacts with a RecognizeImageService instance to classify images
    """
    def __init__(self, service_name, model_name, preprocess_input_module=None):
        """
        :param service_name:
        :param model_name: name of image classification model
        :param preprocess_input_module: module containing 'preprocess_input' method to be called on images
                                        TODO(minhnh) not used
        """
        rospy.wait_for_service(service_name, timeout=5.0)
        try:
            self._recog_proxy = rospy.ServiceProxy(service_name, RecognizeImage)
        except rospy.ServiceException as e:
            rospy.logerr('failed to get proxy for service ' + e.message)
            raise

        self._model_name = model_name

        self._preprocess_input_func = None
        if preprocess_input_module:
            self._preprocess_input_func = getattr(import_module(preprocess_input_module), 'preprocess_input')

    def classify_image_messages(self, image_messages, done_callback_func=None):
        """ Call image recognition service to classify image messages """
        request = RecognizeImageRequest()
        request.images = image_messages
        request.model_name = self._model_name
        response = self._recog_proxy(request)
        if done_callback_func is not None:
            done_callback_func()

        return response.indices, response.classes, response.probabilities
