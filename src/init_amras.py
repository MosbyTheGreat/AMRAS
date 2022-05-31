# USAGE
# python3 pan_tilt_tracking.py -c frontalface.xml

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
servo_pan = 1
servo_tilt = 0

step_size = 10 # decrease to make movement smoother
servo_range = [1, 180]
servo_positions = [90, 90]

#init GPIO Pins
GPIO.setmode(GPIO.BCM)

firing_pin_1 = 24
firing_pin_2 = 23
firing_pin_3 = 22
firing_pin_4 = 27

GPIO.setup(firing_pin_1, GPIO.OUT)
GPIO.setup(firing_pin_2, GPIO.OUT)
GPIO.setup(firing_pin_3, GPIO.OUT)
GPIO.setup(firing_pin_4, GPIO.OUT)

GPIO.output(firing_pin_1, GPIO.LOW)
GPIO.output(firing_pin_2, GPIO.LOW)
GPIO.output(firing_pin_3, GPIO.LOW)
GPIO.output(firing_pin_4, GPIO.LOW)

# function to handle keyboard interrupt
def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")

    # reset servos to default position
    kit.servo[servo_pan].angle = 90
    kit.servo[servo_tilt].angle = 90

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


def set_servos_pid(pan, tilt):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # loop indefinitely
    while True:
        # the pan and tilt angles are reversed
        pan_angle = pan.value
        tilt_angle = tilt.value * -1
        print(pan_angle.__str__() + " " + tilt_angle.__str__())

        # if the pan angle is within the range, pan
        if in_range(pan_angle, servo_range[0], servo_range[1]):
            kit.servo[servo_pan].angle = pan_angle

        # if the tilt angle is within the range, tilt
        if in_range(tilt_angle, servo_range[0], servo_range[1]):
            kit.servo[servo_tilt].angle = tilt_angle


def set_servos(obj_x, obj_y, center_x, center_y):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # loop indefinitely
    while True:  
        error_x = obj_x.value - center_x.value
        new_pos_x = servo_positions[servo_tilt] + error_x
        smooth_move(new_pos_x, servo_tilt)
        
        error_y = obj_y.value + center_y.value
        new_pos_y = servo_positions[servo_pan] + error_y
        smooth_move(new_pos_y, servo_pan)

        print(error_x.__str__() + " " + error_y.__str__())


def smooth_move(angle, servo_nr):
    if in_range(angle, servo_range[0], servo_range[1]):
        if abs(angle - servo_positions[servo_nr]) < step_size:
            kit.servo[servo_nr].angle = angle
            servo_positions[servo_nr] = angle
        else:
            if angle < servo_positions[servo_nr]:
                if angle - step_size < servo_range[0]:
                    kit.servo[servo_nr].angle = servo_range[0]
                    servo_positions[servo_nr] = servo_range[0]
                else:
                    kit.servo[servo_nr].angle = angle - step_size
                    servo_positions[servo_nr] = angle - step_size
            else:
                if angle + step_size > servo_range[1]:
                    kit.servo[servo_nr].angle = servo_range[1]
                    servo_positions[servo_nr] = servo_range[1]
                else:
                    kit.servo[servo_nr].angle = angle + step_size
                    servo_positions[servo_nr] = angle + step_size


# check to see if this is the main body of execution
if __name__ == "__main__":
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--cascade", type=str, required=True, help="path to input Haar cascade for face detection")
    ap.add_argument("-p", "--pid", type=bool, required=False, default=False, help="whether to use PID or more basic movement") # PID not working at the moment
    args = vars(ap.parse_args())

    # set servos to startung position
    kit.servo[servo_pan].angle = servo_positions[servo_pan]
    kit.servo[servo_tilt].angle = servo_positions[servo_tilt]

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
        pan_p = manager.Value("f", 0.0)
        pan_i = manager.Value("f", 0.0)
        pan_d = manager.Value("f", 0.0)

        # set PID values for tilting
        tilt_p = manager.Value("f", 0.0)
        tilt_i = manager.Value("f", 0.0)
        tilt_d = manager.Value("f", 0.0)

        # we have 5 independent processes
        # 1. objectCenter  - finds/localizes the object
        # 2. panning       - PID control loop determines panning angle
        # 3. tilting       - PID control loop determines tilting angle
        # 4. setServosPID  - drives the servos to proper angles based
        #                    on PID feedback to keep object in center
        # 5. setServos     - drives the servos if not using PID, very basic

        if args["pid"]:
            process_object_center = Process(target=obj_center, args=(args, obj_x, obj_y, center_x, center_y))
            process_panning = Process(target=pid_process, args=(pan, pan_p, pan_i, pan_d, obj_x, center_x))
            process_tilting = Process(target=pid_process, args=(tlt, tilt_p, tilt_i, tilt_d, obj_y, center_y))
            process_set_servos_pid = Process(target=set_servos_pid, args=(pan, tlt))

            process_object_center.start()
            process_panning.start()
            process_tilting.start()
            process_set_servos_pid.start()

            process_object_center.join()
            process_panning.join()
            process_tilting.join()
            process_set_servos_pid.join()

        else:
            process_object_center = Process(target=obj_center, args=(args, obj_x, obj_y, center_x, center_y))
            process_set_servos = Process(target=set_servos, args=(obj_x, obj_y, center_x, center_y))

            process_object_center.start()
            process_set_servos.start()

            process_object_center.join()
            process_set_servos.join()
