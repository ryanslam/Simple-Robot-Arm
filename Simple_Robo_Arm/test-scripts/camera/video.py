import time
import sys
sys.path.append(sys.path[0] + '/../..')

import rsl.rr_two_link.camera as camera


# def main():
#     try:
#         n_frame = 0
#         for frame in camera.generate_video_old(framerate=45):
#             # show the frame
#             # cv2.imshow("Frame", frame)
#             n_frame += 1
#             if n_frame > 100:
#                 break
#                 # raise KeyboardInterrupt
#     except KeyboardInterrupt:
#         print(f'Exiting!')
#     except RuntimeError:
#         pass


# main()


cam = camera.CameraWrapper(framerate=32)
cam.start_video()
print(f'Raw stream estimation:')
read_rate = 120
while 1:
    cam.get_latest_frame()
    if cam._fps_estimator._n_frames > 100:
        cam.stop_video()
        break
    time.sleep(1/read_rate)

time.sleep(0.5)

print('\nEncoded stream estimation:')
cam.start_video()

for encoded in cam.encoded_video_generator():
    if cam._fps_estimator._n_frames > 100:
        print(f'')
        cam.stop_video()

        break

cam.clean_up()