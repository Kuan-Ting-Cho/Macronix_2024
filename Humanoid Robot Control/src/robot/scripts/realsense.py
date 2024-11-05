## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs

class realsense:
    def __init__(self):
        try:
            # Create a context object. This object owns the handles to all connected realsense devices
            self.pipeline = rs.pipeline()

            # Configure streams
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            # Start streaming
            self.pipeline.start(config)

        except Exception as e:
            print(e)
            pass

    def get_dist(self):
        while True:
            # This call waits until a new coherent set of frames is available on a device
            # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
            frames = self.pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            if not depth: 
                continue

            # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
            coverage = [0]*64
            dist = 0
            for y in range(50, 200):
                for x in range(100, 540):
                    dist += depth.get_distance(x, y)

            dist = dist / 66000
            print(dist)
            dist = 0
    
        exit(0)

if __name__ == "__main__":
    rs = realsense()
    rs.get_dist()
