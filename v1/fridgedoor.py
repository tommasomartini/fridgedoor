import logging
import time

import RPi.GPIO as GPIO
import cv2
import numpy as np
from tqdm import tqdm

logging.basicConfig(level=logging.DEBUG,
                    format='[{asctime}][{name}][{levelname}] {message}',
                    style='{')
_logger = logging.getLogger(__name__)


_THRESHOLD = 3000
_VIEW = False
_WARMUP_TIME_s = 5
_LIGHT_ON_TIME_s = 10
_CAMERA_ID = 0
_FPS_hz = 30
_CONTROL_PIN = 21


def _initialize_camera():
    cap = cv2.VideoCapture(_CAMERA_ID)
    if not cap.isOpened():
        raise RuntimeError('Cannot open camera {}'.format(_CAMERA_ID))

    _logger.debug('Camera {}: opened'.format(_CAMERA_ID))

    ret_fps = cap.set(cv2.CAP_PROP_FPS, _FPS_hz)
    if ret_fps is False:
        raise RuntimeError('Camera {}: failed to set '
                           'FPS to {} '.format(_CAMERA_ID, _FPS_hz))

    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    if actual_fps != _FPS_hz:
        _logger.warning('Camera {}: tried to set FPS to {}, '
                        'but actual FPS is {}'
                        .format(_CAMERA_ID, _FPS_hz, actual_fps))
    else:
        _logger.debug('Camera {}: set FPS to {}'.format(_CAMERA_ID, _FPS_hz))

    return cap


def _process_frame(frame):
    resized_frame = cv2.resize(frame, (100, 100))
    blurred_frame = cv2.blur(resized_frame, (10, 10))
    return blurred_frame


def _main():
    # Camera setup.
    cap = _initialize_camera()
    bgnd_subtractor = cv2.createBackgroundSubtractorMOG2()

    # GPIO setup.
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(_CONTROL_PIN, GPIO.OUT)

    # Warm-up: to allow the background subtractor to learn the background.
    _logger.debug('Start warm-up')

    with tqdm(total=int(_WARMUP_TIME_s * 1e3), desc='Warm-up') as pbar:
        start_warmup_time = time.time()
        prev_time_s = start_warmup_time
        while time.time() - start_warmup_time < _WARMUP_TIME_s:
            ret_val, frame = cap.read()
            if ret_val is False:
                _logger.debug('Missed frame')
                continue

            proc_frame = _process_frame(frame)
            fgnd_mask = bgnd_subtractor.apply(proc_frame)

            if _VIEW:
                cv2.imshow('Warm-up', fgnd_mask)
                cv2.waitKey(1)

            # Update the tqdm bar.
            time_diff_ms = int(round((time.time() - prev_time_s) * 1e3))
            pbar.update(time_diff_ms)
            prev_time_s = time.time()

    if _VIEW:
        cv2.destroyAllWindows()

    _logger.debug('Warm-up complete')

    light_on_start_time = None
    is_light_on = False
    while True:
        ret_val, frame = cap.read()
        if ret_val is False:
            continue

        proc_frame = _process_frame(frame)
        fgnd_mask = bgnd_subtractor.apply(proc_frame)
        ret_thr, bw_fgnd_mask = cv2.threshold(fgnd_mask,
                                              thresh=1,
                                              maxval=255,
                                              type=cv2.THRESH_BINARY)

        if _VIEW:
            cv2.imshow('Frame', frame)
            cv2.imshow('Processed frame', proc_frame)
            cv2.imshow('FG Mask', bw_fgnd_mask)

            keyboard = cv2.waitKey(1)
            if keyboard == ord('q') or keyboard == 27:
                break

        if light_on_start_time is not None:
            if time.time() - light_on_start_time < _LIGHT_ON_TIME_s:
                # The light must be on a little longer.
                continue

            # Light-on time is over.

            if is_light_on:
                # Time to switch the light off.
                GPIO.output(_CONTROL_PIN, GPIO.LOW)
                is_light_on = False
                _logger.debug('Light off')
                continue
            
            if time.time() - light_on_start_time < \
                    (_LIGHT_ON_TIME_s + _WARMUP_TIME_s):
                # Warm-up time after switching off the light.
                continue

            # Ready to restart a new detection cycle.
            light_on_start_time = None
            _logger.debug('Motion detector reset')
            continue

        if np.sum(bw_fgnd_mask / 255) > _THRESHOLD:
            # Motion detected!
            _logger.debug('Motion detected')
            GPIO.output(_CONTROL_PIN, GPIO.HIGH)
            is_light_on = True 
            light_on_start_time = time.time()

    if _VIEW:
        cv2.destroyAllWindows()

    GPIO.cleanup()
    cap.release()

    _logger.debug('Goodbye!')


if __name__ == '__main__':
    _main()
