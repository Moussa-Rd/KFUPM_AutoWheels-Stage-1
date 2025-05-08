#!/usr/bin/env python3
import os
import signal
import time
from threading import Thread
import numpy as np
import cv2
from multiprocessing import Process, Event

from pal.products.qcar import QCar, QCarGPS, QCarRealSense
from pal.utilities.math import wrap_to_pi
from hal.content.qcar_functions import QCarEKF
from hal.utilities.image_processing import ImageProcessing
from hal.products.mats import SDCSRoadMap
from ultralytics import YOLO

# ===================== Experiment Configuration ===========================
tf = 6000
startDelay = 0.1
controllerUpdateRate = 100

v_ref = 0.7
K_p, K_i, K_d = 10, 5, 5
K_stanley = 1

nodeSequence = [10,1,13,19,17,20,22,9,0,2,4,6,8,10]

FAR_THRESHOLD = 10
STOP_SIGN_L_THRESHOLD = 80
STOP_SIGN_U_THRESHOLD = 50
TF_L_THRESHOLD = 18
TF_U_THRESHOLD = 30

imageWidth  = 640
imageHeight = 480

# ====================== Helper Funcion ====================================

def check_traffic_light(x1, y1, x2, y2, im_cpu):
  
    d = 0.3 * (x2 - x1)
    R_center = (int((x1 + x2) / 2), int((3 * y1 + y2) / 4))
    Y_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    G_center = (int((x1 + x2) / 2), int((y1 + 3 * y2) / 4))

    mask = np.zeros((imageHeight, imageWidth), dtype='uint8')
    maskR = cv2.circle(mask.copy(), R_center, int(d / 2), 1, -1)
    maskY = cv2.circle(mask.copy(), Y_center, int(d / 2), 1, -1)
    maskG = cv2.circle(mask.copy(), G_center, int(d / 2), 1, -1)
    im_hsv = cv2.cvtColor(im_cpu, cv2.COLOR_RGB2HSV)
    vR = np.sum(im_hsv[:, :, 2] * maskR) / np.count_nonzero(maskR)
    vY = np.sum(im_hsv[:, :, 2] * maskY) / np.count_nonzero(maskY)
    vG = np.sum(im_hsv[:, :, 2] * maskG) / np.count_nonzero(maskG)
    mean = (vR + vY + vG) / 3
    threshold = (np.max(np.array([vR, vY, vG])) - np.min(np.array([vR, vY, vG]))) * 0.25

    if vR > mean and vR - mean > threshold:
        status = 'red'
    if vY > mean and vY - mean > threshold:
        status = 'yellow'
    if vG > mean and vG - mean > threshold:
        status = 'green'
    return status if status else 'unkown'

def sig_handler(*args):
    raise KeyboardInterrupt
signal.signal(signal.SIGINT, sig_handler)

# ====================== Initial Setup =====================================
roadmap = SDCSRoadMap(leftHandTraffic=False)
waypointSequence = roadmap.generate_path(nodeSequence)
initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()

calibrate = False
calibrationPose = [0,2,-np.pi/2]

# ===================== Controllers =======================================
class SpeedController:
    def __init__(self, kp, ki, kd):
        self.maxThrottle = 0.3
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_e = None
        self.ei = 0

    def update(self, v, v_target, dt):
        e = v_target - v
        self.ei += dt*e
        de = 0.0 if (self.prev_e is None or dt<1e-3) else (e-self.prev_e)/dt
        self.prev_e = e
        u = self.kp*e + self.ki*self.ei + self.kd*de
        return float(np.clip(u, -self.maxThrottle, self.maxThrottle))

class SteeringController:
    def __init__(self, waypoints, k, cyclic=True):
        self.maxSteeringAngle = np.pi/6
        self.wp = waypoints
        self.N = waypoints.shape[1]
        self.wpi = 0
        self.k = k
        self.cyclic = cyclic
    def update(self, p, th, speed):
        wp1 = 0.99*self.wp[:, self.wpi % (self.N-1)]
        wp2 = 0.99*self.wp[:, (self.wpi+1) % (self.N-1)]
        v = wp2 - wp1
        mag = np.linalg.norm(v)
        if mag<1e-6: return 0.0
        uv = v/mag
        tangent = np.arctan2(uv[1], uv[0])
        s = np.dot(p-wp1, uv)
        if s>=mag and (self.cyclic or self.wpi<self.N-2):
            self.wpi += 1
        ep = wp1 + s*uv
        ct = ep - p
        dir_err = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)
        ect = np.linalg.norm(ct)*np.sign(dir_err)
        psi = wrap_to_pi(tangent - th)
        delta = wrap_to_pi(psi + np.arctan2(self.k*ect, speed))
        return float(np.clip(delta, -self.maxSteeringAngle, self.maxSteeringAngle))


# ===================== Control Process ===================================
def control_process(kill_evt: Event, stop_evt: Event, red_evt: Event):
    # drop-off/pick-up + final
    target_sequence = [
        (np.array([0.125,  4.395]), 3.0),
        (np.array([-0.905, 0.800]), 3.0),
        (np.array([-1.282,-0.45991]), -1)
    ]
    target_index = 0
    in_pause = False
    pause_start = None
    ramp_start = None
    v_target = v_ref

    speed_ctrl = SpeedController(K_p,K_i,K_d)
    steer_ctrl = SteeringController(waypoints=waypointSequence, k=K_stanley)

    qcar = QCar(readMode=1, frequency=controllerUpdateRate)
    ekf = QCarEKF(x_0=initialPose)
    gps = QCarGPS(initialPose=calibrationPose, calibrate=calibrate)

    with qcar, gps:
        t0 = time.time()
        t = 0.0
        leds = np.zeros(8)

        while not kill_evt.is_set() and t < tf+startDelay:
            tp = t
            t = time.time()-t0
            dt = t-tp

            # --- read & estimate state ---
            qcar.read()
            if gps.readGPS():
                y_gps = np.array([gps.position[0],gps.position[1],gps.orientation[2]])
                ekf.update([qcar.motorTach,0],dt,y_gps,qcar.gyroscope[2])
            else:
                ekf.update([qcar.motorTach,0],dt,None,qcar.gyroscope[2])
            x,y,th = ekf.x_hat.flatten()
            p = np.array([x,y]) + np.array([np.cos(th),np.sin(th)])*0.2
            v = float(qcar.motorTach)

            # --- target/pause logic ---
            if target_index < len(target_sequence):
                pos, pause_t = target_sequence[target_index]
                if (not in_pause) and np.linalg.norm(p-pos)<0.1:
                    print("target reached.")
                    leds = np.ones(8)
                    in_pause = True
                    pause_start = t
                    ramp_start = None
                if in_pause:
                    if pause_t>=0 and (t-pause_start)>=pause_t:
                        in_pause = False
                        leds = np.zeros(8)
                        target_index += 1
                        ramp_start = t
                # final
                if pause_t<0 and in_pause:
                    print("Final target reached.")
                    break

            raw_d = steer_ctrl.update(p, th, v)
            if ramp_start is not None:
                sr = min((t-ramp_start)/1.0,1.0)
                max_d = steer_ctrl.maxSteeringAngle*sr
                delta = np.clip(raw_d, -max_d, max_d)
            else:
                delta = 0.0

            if stop_evt.is_set() or red_evt.is_set() or in_pause or ( t < startDelay ):
                v_target = 0.0
                u = 0.0
                delta = 0.0
            else:
                if ramp_start is None: ramp_start = t
                ramp_dt = t-ramp_start
                if ramp_dt<1.0:
                        v_target = v_ref*(ramp_dt/1.0)
                else:
                        v_target = v_ref
                
                if abs(delta) > 0.15:
                    v_target = v_ref*0.9
                else:
                    v_target = v_ref

                u = max(0.0, speed_ctrl.update(v, v_target, dt))

            qcar.write(u, delta, leds)

        qcar.read_write_std(throttle=0, steering=0)


# ===================== Vision Process ====================================
def vision_process(kill_evt: Event, stop_evt: Event, red_evt: Event, yolo_model):
    sample_period = 0.1
    camera = QCarRealSense(
        frameWidthRGB=640,
        frameHeightRGB=480,
    )

    while not kill_evt.is_set():
        t0 = time.time()
        camera.read_RGB()
        frame = camera.imageBufferRGB
        image_center_x = frame.shape[1] // 2
        result = yolo_model.predict(
                    frame,
                    verbose=False,
                    classes=[9, 11, 33],
                    conf=0.5)[0]

        boxes   = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        confs   = result.boxes.conf.cpu().numpy()
        names   = result.names

        h_img, w_img = frame.shape[:2]
        for cls, conf, (x1,y1,x2,y2) in zip(classes, confs, boxes):
            x1,y1,x2,y2 = map(int,(x1,y1,x2,y2))

            if x2-x1 < FAR_THRESHOLD:
                continue

            label_text = f"{names[cls]} {conf:.2f}"
          

            if cls == 9:  # traffic light
                color_status = check_traffic_light(x1, y1, x2, y2, frame)
                cx, cy = (x1+x2)/2, (y1+y2)/2
                if (TF_L_THRESHOLD < x2-x1 < TF_U_THRESHOLD) and (0.25*w_img < cx < 0.75*w_img):
                    if (color_status != 'green') and not stop_evt.is_set():
                        print(f"  → CLOSE TO: ({label_text} - {color_status}) - box size: {x2-x1}x{y2-y1}")
                        print(f"  → RED LIGHT: stopping.")
                        stop_evt.set()
                    
                    if color_status == 'green' and stop_evt.is_set():
                        print(f"  → GREEN LIGHT: resuming.")
                        stop_evt.clear()

            elif cls == 11:  # stop sign
                if  x2-x1 >= STOP_SIGN_U_THRESHOLD:
                    if not stop_evt.is_set():
                        print(f"  → STOP SIGN: {label_text} - box size: {x2-x1}x{y2-y1}")
                        print(f"  → CLOSE STOP SIGN: stopping.")
                        stop_evt.set()
                        Thread(target=lambda: (time.sleep(2), stop_evt.clear()),
                                daemon=True).start()


        dt = time.time()-t0
        if dt < sample_period:
            time.sleep(sample_period-dt)

    camera.terminate()

# ========================= Main ============================================
if __name__ == '__main__':
    kill_event      = Event()
    stop_sign_event = Event()
    red_light_event = Event()

    signal.signal(signal.SIGINT, lambda *args: kill_event.set())
    print('Loading model')
    yolo_model = YOLO(model='./models/yolov8s-seg.pt', task='segment')
    print('Model loaded')
    
    p_ctrl = Process(
        target=control_process,
        args=(kill_event, stop_sign_event, red_light_event),
        name="ControlProc"
    )
    p_vision = Process(
        target=vision_process,
        args=(kill_event, stop_sign_event, red_light_event, yolo_model),
        name="VisionProc"
    )

    p_vision.start()
    p_ctrl.start()

    while p_ctrl.is_alive() and not kill_event.is_set():
        time.sleep(0.01)

    kill_event.set()
    p_ctrl.join()
    p_vision.join()

    input("Experiment complete. Press any key to exit…")
