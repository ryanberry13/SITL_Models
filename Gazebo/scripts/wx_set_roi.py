"""
Set and display a ROI in a wxPanel.
"""

import copy
import cv2
import wx
import numpy as np


class ImagePanel(wx.Panel):
    """ """

    def __init__(self, parent, fps=30, width=640, height=480):
        wx.Panel.__init__(self, parent)

        # Array for default frame (black)
        self._frame = np.ndarray(
            (height, width, 3),
            dtype=np.uint8,
        )

        parent.SetSize((width, height))

        # Convert to RGB
        frame = cv2.cvtColor(self._frame, cv2.COLOR_BGR2RGB)

        # Convert to bitmap for wxFrame
        self.bmp = wx.Bitmap.FromBuffer(width, height, frame)

        self.timer = wx.Timer(self)
        self.timer.Start(int(1000 / fps))

        self.Bind(wx.EVT_MOUSE_EVENTS, self.OnMouseEvent)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_TIMER, self.NextFrame)

        # ROI editing
        self._roi = None
        self._roi_new = None
        self._roi_new_top_left = None
        self._roi_new_bot_right = None
        self._roi_editing = False
        self._roi_changed = False

    def OnMouseEvent(self, event):
        et = event.GetEventType()

        if et == wx.wxEVT_LEFT_DOWN:
            self._roi_new_top_left = event.GetPosition()
            self._roi_editing = True
        elif et == wx.wxEVT_LEFT_UP:
            self._roi_new_bot_right = event.GetPosition()
            self._roi_editing = False
            self._roi_changed = True

        if self._roi_editing:
            self._roi_new_bot_right = event.GetPosition()

    def OnPaint(self, event):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bmp, 0, 0)

    def NextFrame(self, event):

        frame = copy.deepcopy(self._frame)

        if self._roi_editing:
            x1, y1 = self._roi_new_top_left
            x2, y2 = self._roi_new_bot_right
            w = x2 - x1
            h = y2 - y1
            if w > 0 and h > 0:
                self._roi_new = [x1, y1, w, h]
                # Draw the new ROI in red
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)

        if self._roi_changed:
            self._roi = self._roi_new
            self._roi_new = None
            self._roi_changed = False

        if self._roi is not None:
            # Draw the current ROI in green
            x1, y1, w, h = self._roi
            x2 = x1 + w
            y2 = y1 + h
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Convert frame to bitmap for wxFrame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.bmp.CopyFromBuffer(frame)
        self.Refresh()


def main():
    app = wx.App()
    wx_frame = wx.Frame(None)
    wx_frame.Title = "Set ROI"
    image_panel = ImagePanel(wx_frame)
    wx_frame.Show()
    app.MainLoop()


if __name__ == "__main__":
    main()
