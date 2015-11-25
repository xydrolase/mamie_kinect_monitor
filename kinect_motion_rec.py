#!/usr/bin/env python

"""
Motion detector with RGB/IR dual cameras using Kinect.

Usage:

sudo nohup python kinect_motion_rec.py -p 500 -i 0.05 -b 21 -t 25 -s 0.5 --ir > run.log 2>&1 &
"""

from multiprocessing import Process, Queue
from argparse import ArgumentParser
from datetime import datetime, timedelta

import time
import sys
import signal

import freenect
import cv2
import numpy as np

signal_payload = None

def sig_usr_handler(signum, frame):
    global signal_payload

    if signum == signal.SIGUSR1:
        signal_payload = {'motion_mode': 'rgb', 'photo_mode': 'rgb'}
        print >> sys.stderr, "Switching to RGB mode..."
    elif signum == signal.SIGUSR2:
        signal_payload = {'motion_mode': 'ir', 'photo_mode': 'ir'}
        print >> sys.stderr, "Switching to IR mode..."

def img_diff(frames, gblur):
    frames = [cv2.GaussianBlur(f, (gblur, gblur), 0)
            for f in frames]

    return cv2.bitwise_or(
            cv2.absdiff(frames[2], frames[1]),
            cv2.absdiff(frames[1], frames[0])
        )

def s3_uploader(queue, interval):
    pass

def snap_and_save(args, frames=None):
    if args.photo_mode == args.motion_mode and frames:
        snap = np.vstack(frames[1:][::-1])
        yoff = 930
    else:
        yoff = 450
        if args.photo_mode == 'rgb':
            rgb = freenect.sync_get_video()[0]
            snap = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB) 
        else:
            gray = freenect.sync_get_video(0, freenect.VIDEO_IR_8BIT)[0]
            snap = cv2.equalizeHist(gray)

    fname = "snapshots/{0}.jpg".format(str(int(time.time() * 10)))
    
    dt_str = datetime.strftime(
        datetime.now() + timedelta(seconds=-5*3600), 
        '%Y-%m-%d %H:%M:%S')

    cv2.putText(snap, dt_str, (30, yoff), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
        (0, 0, 255), 2)

    cv2.imwrite(fname, snap, [int(cv2.IMWRITE_JPEG_QUALITY), 70])

    return fname

def main():
    global signal_payload 
    parser = ArgumentParser(description="Demo of motion detection.")
    parser.add_argument("-p", type=int, default=1000,
            dest="npixels", help="Threshold of the number of pixels changed.")
    parser.add_argument("-i", type=float, default=0.05,
            dest="interval", help="Photo snapping interval.")
    parser.add_argument("-s", type=float, default=0.5,
            dest="scan_interval", help="Interval for motion detection.")
    parser.add_argument("-b", type=int, default=21, dest="gblur",
            help="Gaussian blurring parameter.")
    parser.add_argument("-t", type=int, default=25, dest="threshold",
            help="Threshold for grayscale image difference.")
    parser.add_argument("--motion", choices=["ir", "rgb"], 
            dest="motion_mode", default="ir", 
            help="Select camera for motion detection.")
    parser.add_argument("--photo", choices=["ir", "rgb"], 
            dest="photo_mode", default="ir", 
            help="Select camera for snapping photos")

    args = parser.parse_args()

    prev_frame = None

    while True:
        frames = []

        for _ in range(2):
            if args.motion_mode == 'ir':
                gray = freenect.sync_get_video(0, freenect.VIDEO_IR_8BIT)[0]
                # Equalize the histogram to enhance contrast
                gray = cv2.equalizeHist(gray)

            else:
                _fdata = freenect.sync_get_video()[0]
                gray = cv2.cvtColor(_fdata, cv2.COLOR_RGB2GRAY)

            frames.append(gray)

            time.sleep(args.interval)

        frames.append(
                np.copy(frames[1]) if prev_frame is None else prev_frame)
        prev_frame = np.copy(frames[1])

        # computes image differences from two consecutive video frames, 
        # and one frame from last scan.
        delta = img_diff(frames, args.gblur)

        thresh = cv2.threshold(delta, args.threshold, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)

        npix = np.sum(thresh > 1)
        if npix > args.npixels and npix < 0.5 * 640 * 480:
            fname = snap_and_save(args, frames) 

            print >>sys.stderr, "{0}: {1} pixels changed.".format(
                    fname, npix)
    
        if signal_payload:
            for k, v in signal_payload.items():
                setattr(args, k, v)

            snap_and_save(args, None) 

            signal_payload = None
            prev_frame = None

        time.sleep(args.scan_interval)

    # proc_s3.join()

if __name__ == "__main__":
    signal.signal(signal.SIGUSR1, sig_usr_handler)
    signal.signal(signal.SIGUSR2, sig_usr_handler)

    main()
