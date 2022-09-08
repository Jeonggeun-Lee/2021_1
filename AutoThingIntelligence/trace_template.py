#!/usr/bin/env python3.6

import math

import colorsys
import os
import time
import rospy

from timeit import default_timer as timer
import numpy as np
from PIL import Image as PILImage
from io import BytesIO

from keras import backend as K
from keras.models import load_model
from keras.layers import Input

from yolo3.model import yolo_eval, yolo_body, tiny_yolo_body
from yolo3.utils import letterbox_image
from keras.utils import multi_gpu_model

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from scout_msgs.msg import ScoutStatus

from sensor_msgs.msg import LaserScan

global width, height

class YOLO(object):
    # edited here
    _defaults = {
        "model_path": 'human_final.h5',
        "anchors_path": 'tiny_yolo_anchors.txt',
        "classes_path": 'human_classes.txt',
        "score" : 0.3,
        "iou" : 0.45,
        "model_image_size" : (416, 416),
        "gpu_num" : 1,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults) # set up default values
        self.__dict__.update(kwargs) # and update with user overrides
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.sess = K.get_session()
        self.boxes, self.scores, self.classes = self.generate()

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'

        # Load model, or construct model and load weights.
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        is_tiny_version = num_anchors==6 # default setting
        try:
            self.yolo_model = load_model(model_path, compile=False)
        except:
            self.yolo_model = tiny_yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes) \
                if is_tiny_version else yolo_body(Input(shape=(None,None,3)), num_anchors//3, num_classes)
            self.yolo_model.load_weights(self.model_path) # make sure model, anchors and classes match
        else:
            assert self.yolo_model.layers[-1].output_shape[-1] == \
                num_anchors/len(self.yolo_model.output) * (num_classes + 5), \
                'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
        np.random.seed(10101)  # Fixed seed for consistent colors across runs.
        np.random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        np.random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        if self.gpu_num>=2:
            self.yolo_model = multi_gpu_model(self.yolo_model, gpus=self.gpu_num)
        boxes, scores, classes = yolo_eval(self.yolo_model.output, self.anchors,
                len(self.class_names), self.input_image_shape,
                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def detect_image(self, image):
        start = timer()

        if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                              image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')

        print(image_data.shape)
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        print('Found {} boxes for {}'.format(len(out_boxes), 'img'))

        font = ImageFont.truetype(font='font/FiraMono-Medium.otf',
                    size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = (image.size[0] + image.size[1]) // 300

        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = self.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            print(label, (left, top), (right, bottom))

            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            # My kingdom for a good redistributable image drawing library.
            for i in range(thickness):
                draw.rectangle(
                    [left + i, top + i, right - i, bottom - i],
                    outline=self.colors[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=self.colors[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw

        end = timer()
        print(end - start)
        return image

    def get_bbox(self, image):
        if image is not None:
            start = timer()

            if self.model_image_size != (None, None):
                assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
                assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
                boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
            else:
                new_image_size = (image.width - (image.width % 32),
                                image.height - (image.height % 32))
                boxed_image = letterbox_image(image, new_image_size)
            image_data = np.array(boxed_image, dtype='float32')

            image_data /= 255.
            image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

            out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.yolo_model.input: image_data,
                    self.input_image_shape: [image.size[1], image.size[0]],
                    # K.learning_phase(): 0
                })
            
            return out_boxes, out_scores, out_classes
        else:
            return -1, -1, -1

    def close_session(self):
        self.sess.close()

class TraceTemplate(object):
    yolo = None
    is_running=False

    def __init__(self):
        self.yolo = YOLO()
        rospy.init_node('camera', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # rospy.Subscriber('/scout_status', ScoutStatus, self.callback_scout_status)

        # rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_image)

        rospy.spin()

    def callback_scout_status(self, data):
        # scout status callback function
        pass
    
    def callback_lidar(self, data):
        # lidar callback function
        pass

    def callback_image(self, data):
        # camera callback function

        # /image_jpeg/compressed topic의 값을 bytearray 형태로 읽어옴
        read_bytes = bytearray(data.data)

        # yolo가 아직 초기화되지 않은 경우 pass
        if self.yolo is None:
            pass

        # 현재 detection을 진행 중인 경우 pass
        if self.is_running:
            pass

        self.is_running = True

        # bytearray 형태의 값을 PIL.Image 타입으로 변환
        pil_image = PILImage.open(BytesIO(read_bytes))

        # yolov3로 detection 진행
        boxes, scores, classes = self.yolo.get_bbox(pil_image)

        # detection 결과가 있는 경우
        if len(boxes) >=1:
            print("111")
            print("boxes:", boxes, "scores:", scores, "classes:", classes)

            global width, height
            width, height = 640, 480

            # boxes가 여러 개인 경우 어떤 bbox 값을 사용할 것인지 지정
            top, left, bottom, right = boxes[0]
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(height, np.floor(bottom + 0.5).astype('int32'))
            right = min(width, np.floor(right + 0.5).astype('int32'))

            # edit code here
            
            scout_cmd_vel_msg = Twist()
            '''
            human_center = (left+right)/2
            screen_center_to_human_center = width/2-human_center
            
            if bottom-top > 280 :
                scout_cmd_vel_msg.angular.z = 0
                scout_cmd_vel_msg.linear.x = 0
            elif bottom-top >180 :        
                scout_cmd_vel_msg.angular.z = 0.0001*screen_center_to_human_center
                scout_cmd_vel_msg.linear.x = 1.0
            elif bottom-top > 80 :
                scout_cmd_vel_msg.angular.z = 0.00001*screen_center_to_human_center
                scout_cmd_vel_msg.linear.x = 1.05
            '''
            scout_cmd_vel_msg.angular.z = 0
            scout_cmd_vel_msg.linear.x = 1.0
            self.cmd_vel_pub.publish(scout_cmd_vel_msg)

            

        else:
            print("Cannot Detect Human!")
        
        time.sleep(0.01)
        self.is_running = False

if __name__ == "__main__":
    try:
        traceTemplate = TraceTemplate()
    except rospy.ROSInterruptException:
        print("exception")
    
    traceTemplate.yolo.close_session()
