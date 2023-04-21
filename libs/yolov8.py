from pcms.openvino_models import Yolov8

class box:


class YOLO:
    def __init__(self, model_name):
        self.model = Yolov8(model_name)

    def __call__(self, *arg, **kwargs):
        76