import time
from argparse import Namespace

import numpy as np
import torch
from celery import Celery, Task
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadImage, letterbox
from utils.general import set_logging, check_img_size, non_max_suppression, scale_coords
from utils.torch_utils import select_device, TracedModel, load_classifier, time_synchronized

app = Celery('tasks', backend='rpc://', broker='pyamqp://')


class DetectionTask(Task):
    def __init__(self):
        opt = Namespace()
        weights, view_img, imgsz, trace = (
            opt.weights,
            opt.view_img,
            opt.img_size,
            not opt.no_trace,
        )
        self.opt = opt

        # Initialize
        set_logging()
        self.device = device = select_device(opt.device)
        self.half = half = device.type != "cpu"  # half precision only supported on CUDA

        # Load model
        self.model = model = attempt_load(weights, map_location=device)  # load FP32 model
        self.stride = stride = int(model.stride.max())  # model stride
        self.imgsz = check_img_size(imgsz, s=stride)  # check img_size

        if trace:
            model = TracedModel(model, device, opt.img_size)

        if half:
            model.half()  # to FP16

        # Second-stage classifier
        classify = False
        if classify:
            modelc = load_classifier(name="resnet101", n=2)  # initialize
            modelc.load_state_dict(
                torch.load("weights/resnet101.pt", map_location=device)["model"]
            ).to(device).eval()

    def run(self, frame):
        if frame is None:
            return
            # Set Dataloader
        dataset = LoadImage(frame, img_size=self.imgsz, stride=self.stride)

        model = self.model
        device = self.device
        imgsz = self.imgsz
        opt = self.opt

        # Get names and colors
        names = model.module.names if hasattr(model, "module") else model.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

        # Run inference
        if device.type != "cpu":
            model(
                torch.zeros(1, 3, imgsz, imgsz)
                .to(device)
                .type_as(next(model.parameters()))
            )  # run once
        old_img_w = old_img_h = imgsz
        old_img_b = 1

        t0 = time.time()
        im0s = frame

        img = letterbox(im0s, self.imgsz, stride=self.stride)
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if device.type != "cpu" and (
                old_img_b != img.shape[0]
                or old_img_h != img.shape[2]
                or old_img_w != img.shape[3]
        ):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                model(img, augment=opt.augment)[0]

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():  # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment=opt.augment)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(
            pred,
            opt.conf_thres,
            opt.iou_thres,
            classes=opt.classes,
            agnostic=opt.agnostic_nms,
        )
        t3 = time_synchronized()

        # Apply Classifier
        # if classify:
        #     pred = apply_classifier(pred, modelc, img, im0s)

        # Process detections

        dets = {
            "frame": frame,
            "detections": {},
            "colors": colors,
        }
        for i, det in enumerate(pred):  # detections per image

            s, im0, frame = "", im0s, getattr(dataset, "frame", 0)

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape
                ).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                    dets["detections"][names[int(c)]] = {
                        "class": int(c),
                        "color": colors[int(c)],
                        "count": n.item(),
                        "boxes": [],
                        "confidences": [],
                    }

                for d in det:
                    c = d[-1]
                    dets["detections"][names[int(c)]]["boxes"].append(
                        d[:4].tolist()
                    )
                    dets["detections"][names[int(c)]]["confidences"].append(
                        d[4].tolist()
                    )

            # Print time (inference + NMS)
            dets["inference_t"] = 1e3 * (t2 - t1)
            dets["nms_t"] = 1e3 * (t3 - t2)

        print(dets)
        print(f"Done. ({time.time() - t0:.3f}s)")
        return dets


detect = app.tasks[DetectionTask.name]
