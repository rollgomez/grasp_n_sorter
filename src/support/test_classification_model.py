import tensorflow as tf
import numpy as np
from PIL import Image
from tensorflow.python.compiler.tensorrt import trt_convert as trt

# Load model
model = tf.keras.models.load_model('./models/pvc_classifier_model.h5')

# Define the class labels (adjust this according to your actual class labels)
class_labels = ['codo', 'neplo', 'tee', 'union'] 

# Function to preprocess the image
def preprocess_image(image_path):
    img = Image.open(image_path)
    img = img.resize((224, 224))
    img_array = np.array(img)
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    return img_array

# Function to make predictions
def predict(image_path):
    img_array = preprocess_image(image_path)
    predictions = model.predict(img_array)
    predicted_class = np.argmax(predictions, axis=1)
    predicted_label = class_labels[predicted_class[0]]
    return predicted_label, np.max(predictions, axis=1)[0]

if __name__ == "__main__":

    image_path = './examples/167037.webp'  

    predicted_label, confidence = predict(image_path)
    print(f'The predicted label is: {predicted_label}, and the confidence level is {confidence*100}%')