from math import ceil
from multiprocessing import Manager
from multiprocessing import Process
from imutils.video import VideoStream
from pyimagesearch.objcenter import ObjCenter
from pyimagesearch.pid import PID
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import numpy as np
import argparse
import signal
import time
import sys
import cv2

# init servos
kit = ServoKit(channels=16)
servo_pan = 1
servo_tilt = 0

multiplier = 0.1 # for translation from error to servo movement
servo_range = [1, 180]
mid_offset = [0, 0] # if the camera is not mounted perfectly flat/centered

#init GPIO Pins
GPIO.setmode(GPIO.BCM)

firing_pins = [22, 23, 24, 27]

for pin in firing_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# camera parameters to undistort images
camera_matrix = np.array([[1522.44073,  0,          1188.03566],
                          [0,           1504.26034, 784.669829],
                          [0,           0,          1]])
dist_coefs = np.array([-0.31086836,  0.10644732,  0.0030072,  -0.0012158,  -0.01891462])


# function to handle keyboard interrupt
def signal_handler(sig, frame):
    # print a status message
    print("You pressed `ctrl + c`! Exiting...")

    # reset servos to default position
    kit.servo[servo_pan].angle = 90
    kit.servo[servo_tilt].angle = 90

    # exit
    sys.exit()


def obj_center(args, obj_x, obj_y, center_x, center_y, search_flag, camera_matrix, dist_coefs):
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

        if args["undistort"]:
            frame = cv2.undistort(frame, camera_matrix, dist_coefs, None, camera_matrix)

        # find the object's location
        object_loc = obj.update(frame, (center_x.value, center_y.value))
        ((obj_x.value, obj_y.value), rect) = object_loc


        # extract the bounding box and draw it
        if rect is not None:
            (x, y, w, h) = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            search_flag.value = 0
        else:
            search_flag.value = 1

        # display the frame to the screen
        if args["image"]:
            cv2.imshow("AMRAS Viewfinder", frame)
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
        pan_angle = pan.value + 90
        tilt_angle = (tilt.value * -1) + 90
        print("Pan Angle:", pan_angle, "Tilt Angle:", tilt_angle)

        # if the pan angle is within the range, pan
        if in_range(pan_angle, servo_range[0], servo_range[1]):
            kit.servo[servo_pan].angle = pan_angle

        # if the tilt angle is within the range, tilt
        if in_range(tilt_angle, servo_range[0], servo_range[1]):
            kit.servo[servo_tilt].angle = tilt_angle

        time.sleep(0.1)


def set_servos(obj_x, obj_y, center_x, center_y, servo_position_x, servo_position_y, search_flag):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    # give the camera time to warm up
    time.sleep(5.0)

    # shoot after x/10 seconds of being on target
    aim_timeout = 2
    aim_timeout_counter = aim_timeout

    firing_counter = 0

    # loop indefinitely
    while True:
        if search_flag.value == 0:
            error_x = obj_x.value - center_x.value + mid_offset[0]
            servo_position_x.value = smooth_move(error_x, servo_pan, servo_position_x.value)

            error_y = (obj_y.value - center_y.value + mid_offset[1]) * -1
            servo_position_y.value = smooth_move(error_y, servo_tilt, servo_position_y.value)

            if abs(error_x) < 10 and abs(error_y) < 10:
                if aim_timeout_counter == 0:
                    if (args["armed"]):
                        fire(firing_counter)
                        firing_counter = firing_counter + 1
                        time.sleep(2.0)
                    else:
                        print("Shoot!")
                    aim_timeout_counter = aim_timeout
                else:
                    if args["verbose"]:
                        print("Aim Timeout Counter:", aim_timeout)
                    aim_timeout_counter = aim_timeout_counter - 1
            else:
                if args["verbose"]:
                    print("Error x:", error_x, "Error y:", error_y)
                aim_timeout_counter = aim_timeout
        time.sleep(0.1)


def smooth_move(error, servo_nr, servo_position):
    new_pos = servo_position - ceil(error * multiplier)
    if in_range(new_pos, servo_range[0], servo_range[1]):
        kit.servo[servo_nr].angle = new_pos
        return new_pos
    else:
        return servo_position


def search_mode(servo_position_x, servo_position_y, search_flag):
    moving_direction = -1
    timeout = 50
    timeout_counter = timeout

    while True:
        if search_flag.value == 1:
            if timeout_counter == 0:
                kit.servo[servo_tilt].angle = 90
                servo_position_y.value = 90
                current_pos = servo_position_x.value

                if current_pos == 0 or current_pos == 180:
                    moving_direction = moving_direction * -1
                    kit.servo[servo_pan].angle = current_pos + moving_direction
                    servo_position_x.value = current_pos + moving_direction
                else:
                    kit.servo[servo_pan].angle = current_pos + moving_direction
                    servo_position_x.value = current_pos + moving_direction
            else:
                if args["verbose"]:
                    print("Timeout Counter:", timeout_counter)
                timeout_counter = timeout_counter -1
        else:
            timeout_counter = timeout
        time.sleep(0.1)


def fire(firing_counter):
    if firing_counter < 4:
        GPIO.output(firing_pins[firing_counter], GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(firing_pins[firing_counter], GPIO.LOW)
        if args["verbose"]:
            print("Projectile", firing_counter + 1, "fired!")
    else:
        if args["verbose"]:
            print("All projectiles fired!")


# check to see if this is the main body of execution
if __name__ == "__main__":
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--cascade", type=str, required=True, help="Path to input Haar cascade for detection")
    ap.add_argument("-a", "--armed", type=bool, required=False, default=False, help="Option to arm the turret")
    ap.add_argument("-p", "--pid", type=bool, required=False, default=False, help="Option to use PID or more basic movement") # PID not working at the moment
    ap.add_argument("-v", "--verbose", type=bool, required=False, default=False, help="Option to show console output")
    ap.add_argument("-i", "--image", type=bool, required=False, default=False, help="Option to draw image")
    ap.add_argument("-u", "--undistort", typ=bool, required=False, default=False, help="Option to undistort images")
    args = vars(ap.parse_args())

    print("Welcome to AMRAS, your room will now be defended against nosy roommates.")

    if args["pid"]:
        print("PID movement will be used.")
    else:
        print("Basic pan/tilt movement will be used.")

    if args["armed"]:
        print("Weapon system is armed and ready.")
    else:
        print("Weapons are not armed, AMRAS will only work as a high-tech scarecrow.")
    
    if args["verbose"]:
        print("AMRAS will report everythign in the console.")
    if args["image"]:
        print("Video feed will be drawn.")
    if args["undistort"]:
        print("Images will be undistorted.")

    print("------------------------------------------------------------------------")

    # set servos to startung position
    kit.servo[servo_pan].angle = 90
    kit.servo[servo_tilt].angle = 90

    # start a manager for managing process-safe variables
    with Manager() as manager:
        # set integer values for the object center (x, y)-coordinates
        center_x = manager.Value("i", 0)
        center_y = manager.Value("i", 0)

        # set integer values for the object's (x, y)-coordinates
        obj_x = manager.Value("i", 0)
        obj_y = manager.Value("i", 0)

        # set integer values for servo positions
        servo_position_x = manager.Value("i", 90)
        servo_position_y = manager.Value("i", 90)

        # initialize search flag
        search_flag = manager.Value("i", 0)

        # we have 5 independent processes
        # 1. objectCenter  - finds/localizes the object
        # 2. panning       - PID control loop determines panning angle
        # 3. tilting       - PID control loop determines tilting angle
        # 4. setServosPID  - drives the servos to proper angles based
        #                    on PID feedback to keep object in center
        # 5. setServos     - drives the servos if not using PID, very basic

        process_object_center = Process(target=obj_center, args=(args, obj_x, obj_y, center_x, center_y, search_flag, camera_matrix, dist_coefs))
        process_search_mode = Process(target=search_mode, args=(servo_position_x, servo_position_y, search_flag))
        process_object_center.start()
        process_search_mode.start()

        if args["pid"]:
            # pan and tilt values will be managed by independed PIDs
            pan = manager.Value("i", 0)
            tilt = manager.Value("i", 0)

            # set PID values for panning
            pan_p = manager.Value("f", 0.0)
            pan_i = manager.Value("f", 0.0)
            pan_d = manager.Value("f", 0.0)

            # set PID values for tilting
            tilt_p = manager.Value("f", 0.0)
            tilt_i = manager.Value("f", 0.0)
            tilt_d = manager.Value("f", 0.0)

            process_panning = Process(target=pid_process, args=(pan, pan_p, pan_i, pan_d, obj_x, center_x))
            process_tilting = Process(target=pid_process, args=(tilt, tilt_p, tilt_i, tilt_d, obj_y, center_y))
            process_set_servos_pid = Process(target=set_servos_pid, args=(pan, tilt))

            process_panning.start()
            process_tilting.start()
            process_set_servos_pid.start()

            process_panning.join()
            process_tilting.join()
            process_set_servos_pid.join()
        else:
            process_set_servos = Process(target=set_servos, args=(obj_x, obj_y, center_x, center_y, servo_position_x, servo_position_y, search_flag))
            process_set_servos.start()
            process_set_servos.join()

        process_object_center.join()
        process_search_mode.join()