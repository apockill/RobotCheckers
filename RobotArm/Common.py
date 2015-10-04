__author__ = 'AlexThiel'
import cv2
import numpy as np
import time
import Variables as Var


class RectSelector:
    def __init__(self, win, callback):
        self.win = win
        self.callback = callback
        cv2.setMouseCallback(win, self.onmouse) #Makes it so that onmouse will run whenever the mouse moves
        self.drag_start = None
        self.drag_rect = None
    def onmouse(self, event, x, y, flags, param):
        x, y = np.int16([x, y]) # BUG
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
        if self.drag_start:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                xo, yo = self.drag_start
                x0, y0 = np.minimum([xo, yo], [x, y])
                x1, y1 = np.maximum([xo, yo], [x, y])
                self.drag_rect = None
                if x1-x0 > 0 and y1-y0 > 0:
                    self.drag_rect = (x0, y0, x1, y1)
            else:
                rect = self.drag_rect
                self.drag_start = None
                self.drag_rect = None
                if rect:
                    self.callback(rect)
    def draw(self, vis):
        if not self.drag_rect:
            return False
        x0, y0, x1, y1 = self.drag_rect
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        return True
    @property
    def dragging(self):
        return self.drag_rect is not None


class Timer:
    def __init__(self, timeLimit):
        self.timeLimit = timeLimit
        self.startTime = time.time()
        self.currentTime = 0
        self.currentTime = 0
        pass

    def wait(self, increment):
        time.sleep(increment)
        self.currentTime += increment

    def timeIsUp(self):
        return time.time() - self.startTime > self.timeLimit
