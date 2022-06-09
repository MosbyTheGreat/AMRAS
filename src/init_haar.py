from multiprocessing import Manager
from multiprocessing import Process
from imutils.video import VideoStream
from pyimagesearch.objcenter import ObjCenter
from pyimagesearch.pid import PID
import argparse
import signal
import time
import sys
import cv2

# function to handle keyboard interrupt
def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")

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

        # start process
        process_object_center = Process(target=obj_center, args=(args, obj_x, obj_y, center_x, center_y))
        process_object_center.start()
        process_object_center.join()
