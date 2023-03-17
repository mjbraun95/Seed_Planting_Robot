import argparse
import numpy as np
import cv2

from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera

def setup(serialPort=None):
    port = None
    camera = None
    # if no serial port is specified, scan for available Tau Camera devices
    if serialPort is None:
        ports = Camera.scan()                      ## Scan for available Tau Camera devices

        if len(ports) > 0:
            port = ports[0]
    else:
        port = serialPort

    if port is not None:
        Camera.setRange(0, 4500)                   ## points in the distance range to be colored

        camera = Camera.open(port)                 ## Open the first available Tau Camera
        camera.setModulationChannel(0)             ## autoChannelEnabled: 0, channel: 0
        camera.setIntegrationTime3d(0, 1000)       ## set integration time 0: 1000
        camera.setMinimalAmplitude(0, 10)          ## set minimal amplitude 0: 80

        cameraInfo = camera.info()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")

    return camera

def run(camera):
    while True:

        frame = camera.readFrame(FrameType.DISTANCE)

        if frame:
            
            mat_depth = np.frombuffer(frame.data_depth, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)

            leftAvg = 0
            rightAvg = 0
            for array in mat_depth:
                arrayRow = 0
                for row in array:
                    curr = np.nanmean(row)
                    if (~np.isnan(curr)):
                        if (arrayRow < 80):
                            leftAvg += curr
                        else:
                            rightAvg += curr
                    arrayRow += 1
            
            # leftAvg /= (60*80)
            # rightAvg /= (60*80)
            # print("Left: ", leftAvg, " Right: ", rightAvg)
            if (leftAvg/(60*80) < 0.8 or rightAvg/(60*80) < 0.8):
                if (leftAvg < rightAvg):
                    print("Object detected closer to the left. Turn right")
                else:
                    print ("Object detected closer to the right. Turn left.")


            # mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
            # mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

            # # array is 160 rows, 3 columns
            # # first 80 rows is left half, last 80 rows is right half
            # # mat_depth_rgb is 60 rows/arrays, 160 columns
            # # first 30 rows is top half, last 30 rows is bottom half
            # leftAvg = 0
            # rightAvg = 0
            # for array in mat_depth_rgb:
            #     arrayRow = 0
            #     for row in array:
            #         if arrayRow < 80:
            #             leftAvg += np.mean(row)
            #         else:
            #             rightAvg += np.mean(row)
            #         arrayRow += 1
            #         # if (row[0] > 100 and row[1] < 50 and row[2] > 100):
            #         #     print("purple")
            
            # leftAvg /= (60*80)
            # rightAvg /= (60*80)
            # print("Left: ", leftAvg, " Right: ", rightAvg)
            # if (leftAvg > 125 or rightAvg > 125):
            #     if (leftAvg > rightAvg):
            #         print("Object detected closer to the left. Turn right")
            #     else:
            #         print ("Object detected closer to the right. Turn left.")


            # Upscalling the image
            # upscale = 4
            # img =  cv2.resize(mat_depth_rgb, (frame.width*upscale, frame.height*upscale))

            # cv2.imshow('Depth Map', img)

            # # Press "esc" to close camera window
            # if cv2.waitKey(1) == 27: break


def cleanup(camera):
    print('\nShutting down ...')
    cv2.destroyAllWindows()
    camera.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Senses when objects are near')
    parser.add_argument('--port', metavar='<serial port>', default=None,
                        help='Specify a serial port for the Tau Camera')
    args = parser.parse_args()

    camera = setup(args.port)

    if camera:
        try:
            run(camera)
        except Exception as e:
            print(e)

        cleanup(camera)
