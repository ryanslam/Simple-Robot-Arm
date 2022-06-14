import sys
sys.path.append(sys.path[0] + '/../..')

import camera
import numpy as np
import cv2

def display_image(image:np.ndarray):
    # display the image on screen and wait for a keypress
    cv2.imshow("Image", image)
    cv2.waitKey(0)
    # return image

if __name__ == '__main__':
    img = camera.take_image()
    
    print(img)
    print(type(img))
    display_image(img)

    cv2.destroyAllWindows()