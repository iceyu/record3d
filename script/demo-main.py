import numpy as np
from record3d import Record3DStream
import cv2
from threading import Event
from pathlib import Path
import argparse
import time
version = "0.1.2"
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--record",default=0 , help="Enable record. 0 : save. 1 : record") 
    parser.add_argument("--path", default=str(Path.cwd()) ,
                        help="Path save files")

    args = parser.parse_args()

    return args

class DemoApp:
    def __init__(self,record_path,record):
        self.event = Event()
        self.session = None
        self.record_path = record_path
        self.record = record

    def on_new_frame(self):
        """
        This method is called from non-main thread, therefore cannot be used for presenting UI.
        """
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        print('Stream stopped')

    def connect_to_device(self, dev_idx):
        print('Searching for devices')
        devs = Record3DStream.get_connected_devices()
        print('{} device(s) found'.format(len(devs)))
        for dev in devs:
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'
                               .format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)  # Initiate connection and start capturing

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])


    def start_processing_stream(self):
        save_frame_cnt = 0
        while True:
            self.event.wait()  # Wait for new frame to arrive

            # Copy the newly arrived RGBD frame
            depth = self.session.get_depth_frame()
            rgb = self.session.get_rgb_frame()
            intrinsic_mat = self.get_intrinsic_mat_from_coeffs(self.session.get_intrinsic_mat())
            # You can now e.g. create point cloud by projecting the depth map using the intrinsic matrix.
            
            # Postprocess it
            are_truedepth_camera_data_being_streamed = depth.shape[0] == 640
            if are_truedepth_camera_data_being_streamed:
                depth = cv2.flip(depth, 1)
                rgb = cv2.flip(rgb, 1)

            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            # Show the RGBD Stream
            cv2.imshow('RGB', rgb)
            cv2.imshow('Depth', depth)
            key = cv2.waitKey(1)
            if key == 27 :
                break
            save_flag = False
            if key == 115 or  key == 83:
                save_flag = True
   
            if self.record == '1'  :
                save_flag = True


            if save_flag == True:
                cv2.imwrite(str(self.record_path)+'/depth/'+'{:0>8d}'.format(save_frame_cnt)+'.pfm',depth)
                cv2.imwrite(str(self.record_path)+'/rgb/'+'{:0>8d}'.format(save_frame_cnt)+'.png',rgb)
                np.savetxt(str(self.record_path)+'/intrinsic/'+'{:0>8d}'.format(save_frame_cnt)+'.txt', intrinsic_mat, fmt="%f",delimiter=",")
                save_frame_cnt=save_frame_cnt+1

            self.event.clear()


if __name__ == '__main__':
    args = parse_args()

    record_path = Path(args.path+'/'+time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime()))
    depth_path = Path(str(record_path)+'/depth')
    intrinsic_path = Path(str(record_path)+'/intrinsic')
    rgb_path = Path(str(record_path)+'/rgb')
    record_path.mkdir(exist_ok=True)
    depth_path.mkdir(exist_ok=True)
    rgb_path.mkdir(exist_ok=True)
    intrinsic_path.mkdir(exist_ok=True)
    
    print("version : " + version)
    print("record path : " +str(record_path))

    app = DemoApp(record_path,args.record)
    app.connect_to_device(dev_idx=0)
    app.start_processing_stream()
