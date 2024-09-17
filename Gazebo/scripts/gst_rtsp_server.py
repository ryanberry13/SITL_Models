"""
GST RTSP Server Example

Adapted from

https://github.com/tamaggo/gstreamer-examples
https://github.com/tamaggo/gstreamer-examples/blob/master/test_gst_rtsp_server.py

Original code by Jerome Carretero (Tamaggo)
"""

# Copyright (c) 2015 Tamaggo Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import sys
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst
from gi.repository import GstRtspServer
from gi.repository import GLib


class MediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        GstRtspServer.RTSPMediaFactory.__init__(self)

    def do_create_element(self, url):
        s_src = "videotestsrc ! video/x-raw,rate=30,width=640,height=480,format=I420"
        s_h264 = "x264enc tune=zerolatency"
        pipeline_str = "( {s_src} ! queue max-size-buffers=1 name=q_enc ! {s_h264} ! rtph264pay name=pay0 pt=96 )".format(
            **locals()
        )
        if len(sys.argv) > 1:
            pipeline_str = " ".join(sys.argv[1:])
        print(pipeline_str)
        return Gst.parse_launch(pipeline_str)


class GstServer:
    def __init__(self):
        self.server = GstRtspServer.RTSPServer()
        media_factory = MediaFactory()
        media_factory.set_shared(True)
        mount_points = self.server.get_mount_points()
        mount_points.add_factory("/test", media_factory)
        self.server.attach(None)


def main():
    Gst.init(None)
    server = GstServer()
    loop = GLib.MainLoop()
    loop.run()


if __name__ == "__main__":
    main()
