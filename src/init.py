# USAGE
# python3 pan_tilt_tracking.py --cascade haarcascade_frontalface_default.xml

# import necessary packages
from multiprocessing import Manager
from multiprocessing import Process
from imutils.video import VideoStream
from pyimagesearch.objcenter import ObjCenter
from pyimagesearch.pid import PID
from adafruit_servokit import ServoKit
import adafruit_motor.servo
import RPi.GPIO as GPIO
import argparse
import signal
import time
import sys
import cv2

# init servos
kit = ServoKit(channels=16)

pan_pos = 90
pan_multiplier = 0.1

tilt_pos = 90
tilt_multiplier = 0.1

#init GPIO Pins
GPIO.setmode(GPIO.BCM)

Pin1 = 24
Pin2 = 23
Pin3 = 22
Pin4 = 27

GPIO.setup(Pin1, GPIO.OUT)
GPIO.setup(Pin2, GPIO.OUT)
GPIO.setup(Pin3, GPIO.OUT)
GPIO.setup(Pin4, GPIO.OUT)

GPIO.output(Pin1, GPIO.LOW)
GPIO.output(Pin2, GPIO.LOW)
GPIO.output(Pin3, GPIO.LOW)
GPIO.output(Pin4, GPIO.LOW)

# function to handle keyboard interrupt
def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")

    # reset servos to default position
    kit.servo[0].angle = 90
    kit.servo[1].angle = 90

    # exit
    sys.exit()


def obj_center(args, obj_x, obj_y, center_x, center_y):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # start the video stream and wait for the camera to warm up
    vs = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)

    # initialize the object center finder
    obj = ObjCenter(args["cascade"])

    # loop indefinitely
    while True:
        # grab the frame from the threaded video stream
        frame = vs.read()

        # calculate the center of the frame as this is where we will
        # try to keep the object
        (H, W) = frame.shape[:2]
        center_x.value = W // 2
        center_y.value = H // 2

        # find the object's location
        object_loc = obj.update(frame, (center_x.value, center_y.value))
        ((obj_x.value, obj_y.value), rect) = object_loc

        # extract the bounding box and draw it
        if rect is not None:
            (x, y, w, h) = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # display the frame to the screen
        cv2.imshow("Pan-Tilt Face Tracking", frame)
        cv2.waitKey(1)


def pid_process(output, p, i, d, obj_coord, center_coord):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # create a PID and initialize it
    p = PID(p.value, i.value, d.value)
    p.initialize()

    # loop indefinitely
    while True:
        # calculate the error
        error = center_coord.value - obj_coord.value

        # update the value
        output.value = p.update(error)


def in_range(val, start, end):
    # determine the input vale is in the supplied range
    return start <= val <= end


def set_servos(pan, tilt):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # loop indefinitely
    while True:
        # the pan and tilt angles are reversed
        # here is some TODO, need to get new camera first and tinker around
        pan_angle = -1 * pan.value
        tilt_angle = -1 * tilt.value
        print(pan_angle.__str__() + " " + tilt_angle.__str__())

        # if the pan angle is within the range, pan
        if in_range(pan_angle, 0, 180):
            kit.servo[0].angle = pan_angle

        # if the tilt angle is within the range, tilt
        if in_range(tilt_angle, 0, 180):
            kit.servo[1].angle = tilt_angle


# check to see if this is the main body of execution
if __name__ == "__main__":
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--cascade", type=str, required=True, help="path to input Haar cascade for face detection")
    args = vars(ap.parse_args())

    # start a manager for managing process-safe variables
    with Manager() as manager:
        # set integer values for the object center (x, y)-coordinates
        center_x = manager.Value("i", 0)
        center_y = manager.Value("i", 0)

        # set integer values for the object's (x, y)-coordinates
        obj_x = manager.Value("i", 0)
        obj_y = manager.Value("i", 0)

        # pan and tilt values will be managed by independed PIDs
        pan = manager.Value("i", 0)
        tlt = manager.Value("i", 0)

        # set PID values for panning
        pan_p = manager.Value("f", 0.09)
        pan_i = manager.Value("f", 0.08)
        pan_d = manager.Value("f", 0.002)

        # set PID values for tilting
        tilt_p = manager.Value("f", 0.11)
        tilt_i = manager.Value("f", 0.10)
        tilt_d = manager.Value("f", 0.002)

        # we have 4 independent processes
        # 1. objectCenter  - finds/localizes the object
        # 2. panning       - PID control loop determines panning angle
        # 3. tilting       - PID control loop determines tilting angle
        # 4. setServos     - drives the servos to proper angles based
        #                    on PID feedback to keep object in center
        process_object_center = Process(target=obj_center, args=(args, obj_x, obj_y, center_x, center_y))
        process_panning = Process(target=pid_process, args=(pan, pan_p, pan_i, pan_d, obj_x, center_x))
        process_tilting = Process(target=pid_process, args=(tlt, tilt_p, tilt_i, tilt_d, obj_y, center_y))
        process_set_servos = Process(target=set_servos, args=(pan, tlt))

        # start all 4 processes
        process_object_center.start()
        process_panning.start()
        process_tilting.start()
        process_set_servos.start()

        # join all 4 processes
        process_object_center.join()
        process_panning.join()
        process_tilting.join()
        process_set_servos.join()
