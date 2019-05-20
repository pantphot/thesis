int32 sec
uint32 nanosec
string frame_id
uint32 x_offset  #Leftmost pixel of the ROI
uint32 y_offset  #Topmost pixel of the ROI
uint32 height    # Height of ROI
uint32 width     # Width of ROI
bool do_rectify  ## True if a distinct rectified ROI should be calculated from the "raw"
                  # ROI in this message. Typically this should be False if the full image
                  # is captured (ROI not used), and True if a subwindow is captured (ROI
                  # used).
uint32 image_width     # original image width
uint32 image_height     # original image height
