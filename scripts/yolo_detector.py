import rospy
from std_msgs.msg import Bool
import cv2
import numpy as np

class YOLODetector:
    def __init__(self):
        self.human_detected_pub = rospy.Publisher('human_detected', Bool, queue_size=10)
        rospy.init_node('yolo_detector', anonymous=True)
        self.rate = rospy.Rate(10)
        self.cap = cv2.VideoCapture(0)
        self.net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def detect_human(self, frame):
        height, width, channels = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if class_id == 0 and confidence > 0.5:
                    return True
        return False

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                human_detected = self.detect_human(frame)
                self.human_detected_pub.publish(human_detected)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        detector.cap.release()
        cv2.destroyAllWindows()
