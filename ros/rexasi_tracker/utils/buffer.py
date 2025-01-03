from collections import deque
from threading import Lock
from threading import Event
import queue
import copy

from rexasi_tracker.config.parameters.src.camera import FPS


class BlockingDeque(deque):
    def __init__(self, max_length):
        super().__init__(maxlen=max_length)
        self.not_empty = Event()
        self.popLock = Lock()
        self.appendLock = Lock()

    def append(self, elem):
        with self.appendLock:
            removedElement = True if len(self) == self.maxlen else False
            super().append(elem)
            self.not_empty.set()
        return removedElement

    def popleft(self):
        with self.popLock:
            self.not_empty.wait()
            with self.appendLock:
                if not (len(self) - 1):
                    self.not_empty.clear()
                toReturn = super().popleft()
        return toReturn


class Buffer(deque):
    """
    Inits a circular buffer of maxlen size.
    """

    def __init__(self, size):
        super(Buffer, self).__init__(maxlen=size)

    def __deepcopy__(self, memo):
        """
        Custom deep copy implementation for Buffer objects.
        """
        # Create a new Buffer instance with the same size
        new_buffer = Buffer(self.maxlen)

        # Create a shallow copy of the deque to iterate over
        items = list(self)

        # Copy elements from the original buffer to the new buffer
        for item in items:
            new_buffer.append(copy.deepcopy(item, memo))

        return new_buffer


def get_image_from_buffer(buffer: Buffer, timestamp: int, return_timestamp=False):
    # get image with closer timestamp
    found_idx = 0
    last_diff = -1
    for idx, r in enumerate(buffer):
        diff = abs(r[1] - timestamp)
        if last_diff == -1:
            last_diff = diff
            continue
        if diff <= last_diff:
            last_diff = diff
            found_idx = idx
        else:
            break
    if return_timestamp:
        return buffer[found_idx][0], buffer[found_idx][1]
    else:
        return buffer[found_idx][0]


def get_image_from_queue(frames: BlockingDeque, lastFrame, keypointTs: int, logger):
    endWhile = False
    frameToReturn, frameTs = [None, None]
    associated = False
    while not endWhile:
        if lastFrame is None:
            frameToReturn, frameTs = frames.popleft()
        logger.debug(f"keypoint timestamp: {keypointTs}, frame timestamp: {frameTs}")
        timestampDiff = keypointTs - frameTs
        if timestampDiff < pow(10, 9) / FPS and timestampDiff > 0:
            logger.debug(
                f"get image: timestamps are elmost equal, diff is {timestampDiff}"
            )
            endWhile = True
            associated = True
        elif keypointTs < frameTs:
            logger.debug(f"get image: keypoints is older then timestamp")
            endWhile = True

        # Note that if keypoints > frameTs the while is repeated that
        # implies that the current frame is discarded because the timestamp
        # of the keypoint is more recent than the timestamp of the frame
    return frameToReturn, frameTs, associated


def get_data_from_buffer(buffer: Buffer, timestamp: int):
    # get image with closer timestamp
    found_idx = 0
    last_diff = -1
    for idx, b in enumerate(buffer):
        diff = abs(b.timestamp - timestamp)
        if last_diff == -1:
            last_diff = diff
            continue
        if diff <= last_diff:
            last_diff = diff
            found_idx = idx
        else:
            break
    return buffer[found_idx]
