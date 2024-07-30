#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
import numpy as np
from PIL import Image as PilImage
from tensorflow.python.compiler.tensorrt import trt_convert as trt
from grasp_n_sorter.srv import classifyImg, classifyImgResponse
import rospkg

# Model
rospack = rospkg.RosPack()
package_path = rospack.get_path('grasp_n_sorter')
model_path = package_path + '/models/pvc_classifier_hand.h5'
model = tf.keras.models.load_model(model_path)

# Classes
class_labels = ['codo', 'neplo', 'tee', 'union'] 

# Function to preprocess the image
def preprocess_image(cv_image):
    img = PilImage.fromarray(cv_image)
    img = img.resize((224, 224))
    img_array = np.array(img)
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    return img_array

# Function to make predictions
def predict(cv_image):
    img_array = preprocess_image(cv_image)
    predictions = model.predict(img_array)
    predicted_class = np.argmax(predictions, axis=1)
    predicted_label = class_labels[predicted_class[0]]
    return predicted_label, np.max(predictions, axis=1)[0]

class ImageClassifier:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        rospy.init_node('image_classifier')
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.service = rospy.Service('classify_image', classifyImg, self.handle_classification)
        rospy.spin()

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logingo(e)
    
    def handle_classification(self, req):
        if req.confirmation == 1 and self.cv_image is not None:
            predicted_label, confidence = predict(self.cv_image)
            return classifyImgResponse(class_name = predicted_label, confidence_level = confidence)
        else:
            return classifyImgResponse(class_name = 'Not available', confidence_level = -1)

        
if __name__ == "__main__":
    classifier = ImageClassifier()
