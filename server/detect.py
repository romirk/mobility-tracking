import queue
import time
from multiprocessing import Queue
from pathlib import Path

import torch
from numpy import random

from models.yolo import attempt_load
from utils.datasets import LoadImage
from utils.general import (
    apply_classifier,
    check_img_size,
    increment_path,
    non_max_suppression,
    scale_coords,
    set_logging,
)
from utils.torch_utils import (
    TracedModel,
    load_classifier,
    select_device,
    time_synchronized,
)

RAW_IMG_Q = Queue(10)
PROCESSED_Q = Queue(10)


def detect(opt, save_img=False):
    source, weights, view_img, save_txt, imgsz, trace = (
        opt.source,
        opt.weights,
        opt.view_img,
        opt.save_txt,
        opt.img_size,
        not opt.no_trace,
    )
    save_img = not opt.nosave and not source.endswith(".txt")  # save inference images
    webcam = (
            source.isnumeric()
            or source.endswith(".txt")
            or source.lower().startswith(("rtsp://", "rtmp://", "http://", "https://"))
    )

    # Directories
    save_dir = Path(
        increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok)
    )  # increment run
    (save_dir / "labels" if save_txt else save_dir).mkdir(
        parents=True, exist_ok=True
    )  # make dir

    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != "cpu"  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size

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

    while True:
        entry = RAW_IMG_Q.get(block=True)
        if entry is None:
            break
        source = entry["source"]
        frame = entry["frame"]

        # Set Dataloader
        dataset = LoadImage(frame, img_size=imgsz, stride=stride)

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

        dets = {
            "source": source,
            "frame": frame,
            "uuid": entry["uuid"],
            "detections": {},
            "colors": colors,
            "captured": entry["timestamp"],
        }
        t0 = time.time()
        for img, im0s, _ in dataset:
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
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
            if classify:
                pred = apply_classifier(pred, modelc, img, im0s)

            # Process detections
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
                # print(
                #     f"{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS",
                #     dets,
                # )
        try:
            PROCESSED_Q.put_nowait(dets)
        except queue.Full:
            print("Processed Queue is full! Dropping detections")

    print(f"Done. ({time.time() - t0:.3f}s)")
