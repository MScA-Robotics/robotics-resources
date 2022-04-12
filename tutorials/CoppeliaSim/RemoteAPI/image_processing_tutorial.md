# CoppeliaSim Python Remote API Image Processing

In this tutorial, we will introduce how to use OpenCV to process images from the CoppeliaSim vision sensors.

_This tutorial uses the bubbleRob simulation from the [bubbleRob tutorial](../bubbleRob/bubbleRob_tutorial.md)

## Setup

1. Open the bubbleRob scene from the (bubbleRob tutorial)[../bubbleRob/bubbleRob_tutorial.md] in CoppeliaSim
2. Make sure the Remote API is enabled in your scene (as a child script)
3. Disable the main bubbleRob child script is disabled in the scene
4. Disable the visionSensor child script
5. Make sure you have [OpenCV](https://pypi.org/project/opencv-python/) installed in your python environment

## Replicating the CoppeliaSim Edge Detection in Python with OpenCV

Parts of this tutorial refer back to concepts covered in the previous tutorial. All the code blocks mentioned in the steps are combined in the [Complete Python Script](#complete-python-script) at the end of this tutorial

Remember to refer to the [Python Remote API Documentation](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) as needed.

### About OpenCV

OpenCV (`cv2`) is a robust computer vision Python package (and is also available for other languages).

Take time to learn about OpenCV using the variety of [tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)

### Reading the Input from the Vision sensor

After getting the connection to the remote client, we need to get the handle of the _visionSensor_ similar to how we read the _proximitySensor_ in the previous tutorial.

To get the current image from the vision sensor, use `sim.simxGetVisionSensorImage()`.  This method takes 4 arguments:
* The `ClientID`
* The vision sensor's handle object
* An integer for color (0) or greyscale(1)
* An [operation mode](https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#operationModes) for reading the data

And returns 3 values:
* err_code
* resolution
* image data

The first time you read from a sensor, use `simx_opmode_streaming` mode. For subsequent reads, use `simx_opmode_buffer` mode. The `streaming` read of the vision sensor will return an `err_code` of 1 and an empty `resolution` and `image` array.  This is expected.

```python
# get the handle of the visionSensor
err_code,vision_sensor = sim.simxGetObjectHandle(clientID,"visionSensor", sim.simx_opmode_blocking)

if err_code > 1:
    sys.exit(f'error accessing visionSensor: {err_code}')

# read the current state of the visionSensor
# the numeric option is 0/1 for greyscale/color
err_code,resolution,image = sim.simxGetVisionSensorImage(clientID,vision_sensor,0,sim.simx_opmode_streaming)
if err_code >= 1:
    sys.exit(f'error reading visionSensor: {err_code}')
```

After the configuration and first read of the vision sensor, the `buffer` mode read can be added to the robot processing loop:

```python
err_code,resolution,image = sim.simxGetVisionSensorImage(clientID, vision_sensor,0,sim.simx_opmode_buffer)
```

### Processing & Saving Images

The `process_save_image()` method below will pre-process the image data, apply openCV Canny edge detection, and save the file as a png.

#### Pre-Processing

In order to use and save the image data from the sensor, it needs to be pre-processed.
1. Convert to `numpy` array
2. Resize based on the `resolution` specifications using the `resize()` method of the `img` array
3. Rotate the image 180 (based on how the sensor displays as default) using `cv2.rotate()`
4. By default, the image from the sensor has the color channels ordered as RGB, but OpenCV uses BGR.  Therefore, we need to reorder the image channels to BGR using `cv2.cvtColor()`

#### Apply Canny Edge Detection

We will replicate the CoppeliaSim edge detection in Python using OpenCV Canny Edge Detection.

Canny Edge Detection uses a combination of Gaussian noise filtering and pixel gradient calculations to identify edges based on specified thresholds.  Please review the OpenCV [tutorial](https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html) about Canny edge detection.  The 2nd and 3rd arguments to `cv2.Canny()` are the thresholds for the gradient.  Experiment with the values and see how the output changes.

#### Save Image to Directory

For this tutorial, we will be saving the individual images with the edge detection to the subdirectory `imgs` using `cv2.imwrite()`. **WARNING:** `imwrite()` unfortunately does not throw errors if the image fails to save.  Therefore, make sure the filename path where you want to save exists first.

#### process_save_image()

Putting it all together, here is the method to process and save an incoming image.  The `process_save_image()` method can be called after each subsequent buffer `simxGetVisionSensorImage()` call.

```python
def process_save_image(image, resolution, save_dir = 'imgs'):

    # convert image to numpy array
    img = np.array(image, dtype = np.uint8)

    # resize to resolution
    img.resize([resolution[0],resolution[1],3])

    # rotate image so it is right-side up
    img = cv2.rotate(img, cv2.cv2.ROTATE_180)

    # convert to BGR for openCV
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # apply openCV Canny edge detection with thresholds 100, 200 (adjust as desired)
    # make sure that the edge detection script on the visionSensor is disabled in CopeliaSim
    img = cv2.Canny(img, 100, 200)

    # write image with name as integer datetime (as a naive name gen)
    dir_path = f'./{save_dir}'
    if not os.path.isdir(dir_path):
        raise ValueError("No such directory: {}".format(dir_path))

    filename = f'{dir_path}/{int(round(datetime.now().timestamp()))}.png'

    cv2.imwrite(filename, img)
```

## Complete Python Script

Add the new vision sensor processing to the previous _bubbleRob_ Remote API Python file.  

We can use `while (sim.simxGetConnectionId(clientID) != -1)` to run our process as long as the simulation is running in CoppeliaSim and the remote connection port is open (play button pressed).  When you hit the stop button in CoppeliaSim, then the script will finish.  

```python
import sim
import time
import sys
import cv2
from datetime import datetime
import numpy as np
import os

def process_save_image(image, resolution, save_dir = 'imgs'):

    # convert image to numpy array
    img = np.array(image, dtype = np.uint8)

    # resize to resolution
    img.resize([resolution[0],resolution[1],3])

    # rotate image so it is right-side up
    img = cv2.rotate(img, cv2.cv2.ROTATE_180)

    # convert to BGR for openCV
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # apply openCV Canny edge detection with thresholds 100, 200 (adjust as desired)
    # make sure that the edge detection script on the visionSensor is disabled in CopeliaSim
    img = cv2.Canny(img, 100, 200)

    # write image with name as integer datetime (as a naive name gen)
    dir_path = f'./{save_dir}'
    if not os.path.isdir(dir_path):
        raise ValueError("No such directory: {}".format(dir_path))

    filename = f'{dir_path}/{int(round(datetime.now().timestamp()))}.png'

    cv2.imwrite(filename, img)

def main():

    sim.simxFinish(-1) # just in case, close all opened connections

    # Get the current client ID
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)

    if clientID!=-1:
        print ("Connected to remote API server")
    else:
        print("Could not connect to remote API server")

    # get the handle of the leftMotor
    # NOTE: many of the API functions will always return an error code.  This is useful for debugging
    err_code,left_motor = sim.simxGetObjectHandle(clientID,"/leftMotor", sim.simx_opmode_blocking)
    if err_code > 1:
        sys.exit(f'error accessing leftMotor: {err_code}')

    # get the handle of the rightMotor
    err_code,right_motor = sim.simxGetObjectHandle(clientID,"/rightMotor", sim.simx_opmode_blocking)
    if err_code > 1:
        sys.exit(f'error accessing rightMotor: {err_code}')

    # get the handle of the proximitySensor
    err_code,proximity_sensor = sim.simxGetObjectHandle(clientID,"/proximitySensor", sim.simx_opmode_blocking)

    if err_code > 1:
        sys.exit(f'error accessing proximitySensor: {err_code}')

    # get the handle of the visionSensor
    err_code,vision_sensor = sim.simxGetObjectHandle(clientID,"visionSensor", sim.simx_opmode_blocking)

    if err_code > 1:
        sys.exit(f'error accessing visionSensor: {err_code}')

    # read the current state of the proximitySensor
    # NOTE: simxReadProximitySensor() returns a lot of information. This script will only use detectionState, but read up on the rest of the returned objects.
    (err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector
     ) = sim.simxReadProximitySensor(clientID, proximity_sensor, sim.simx_opmode_streaming)
    if err_code > 1:
        sys.exit(f'error reading proximitySensor: {err_code}')

    # read the current state of the visionSensor
    # the numeric option is 0/1 for greyscale/color
    err_code,resolution,image = sim.simxGetVisionSensorImage(clientID,vision_sensor,0,sim.simx_opmode_streaming)
    if err_code >= 1:
        sys.exit(f'error reading visionSensor: {err_code}')

    speed = 2

    err_code = sim.simxSetJointTargetVelocity(clientID,left_motor, speed ,sim.simx_opmode_streaming)
    err_code = sim.simxSetJointTargetVelocity(clientID,right_motor, speed ,sim.simx_opmode_streaming)

    start_time = time.time() #record the initial time

    obs = False # flag to identify if obstacle was reached
    dt = 0
    t = start_time

    while (sim.simxGetConnectionId(clientID) != -1): # run while ClientID open

        # read the vision sensor
        err_code,resolution,image = sim.simxGetVisionSensorImage(clientID, vision_sensor,0,sim.simx_opmode_buffer)
        # Save the current image
        process_save_image(image, resolution)

        # if obstacle detected
        if detectionState:
            obs = True # set the obs flag
            dt = t # set dt (detection time) equal to the current time (t)


        # if detect obstacle, reverse and turn for 4 seconds
        if obs & (t <= dt+4):
            err_code = sim.simxSetJointTargetVelocity(clientID, left_motor,-speed/2, sim.simx_opmode_streaming)
            err_code = sim.simxSetJointTargetVelocity(clientID, right_motor,-speed/8, sim.simx_opmode_streaming)

        # otherwise, go straight
        else:
            obs = False #reset obs flag
            err_code = sim.simxSetJointTargetVelocity(clientID, left_motor, speed ,sim.simx_opmode_streaming)
            err_code = sim.simxSetJointTargetVelocity(clientID, right_motor, speed ,sim.simx_opmode_streaming)

        # wait for a bit before sensing again
        time.sleep(0.2)

        # read the proximity sensor
        (err_code, detectionState,detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector
         ) = sim.simxReadProximitySensor(clientID, proximity_sensor, sim.simx_opmode_buffer)

        # update the time
        t = time.time()

    # stop the simulation when complete (you should see the CoppeliaSim UI reset)
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)

    print("Done")

if __name__ == '__main__':
    main()
```
