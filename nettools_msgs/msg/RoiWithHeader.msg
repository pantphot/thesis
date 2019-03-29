std_msgs/Header header       # Two-integer timestamp that is expressed as seconds and nanoseconds. builtin_interfaces/Time stamp
                             # sequence number. string frame_id
sensor_msgs/RegionOfInterest roi#uint32 x_offset  Leftmost pixel of the ROI
                                #uint32 y_offset  Topmost pixel of the ROI
                                #uint32 height    # Height of ROI
                                #uint32 width     # Width of ROI
uint32 image_width     # original image width
