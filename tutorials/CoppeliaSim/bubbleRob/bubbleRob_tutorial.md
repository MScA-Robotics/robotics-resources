# BubbleRob tutorial

_This tutorial is an update and enhancement of the [CoppeliaSim BubbleRob Tutorial](https://www.coppeliarobotics.com/helpFiles/en/bubbleRobTutorial.htm)_

This tutorial will introduce many CoppeliaSim functionalities while designing the simple mobile robot _BubbleRob_.  The figure below illustrates the simulation scene that we will design:

![](img/bubbleRob_final.png)

Since this tutorial will fly over many different aspects, make sure to **click on all the links in each of the instructions to learn more about each of the menus and settings**, and also have a look at the [other tutorials.](https://www.coppeliarobotics.com/helpFiles/en/tutorials.htm)

After building our robot and environment, we will control it in two different ways.  The scripts that run natively within CoppeliaSim use the **Lua** programming language. It is not complex.  We will also experiment with controlling our robot remotely using the Python Remote API.  After finishing this tutorial, please [learn more](https://www.lua.org/) about the Lua language, as well as the differences in running [Lua vs Python scripts](https://www.coppeliarobotics.com/helpFiles/en/luaPythonDifferences.htm) in CoppeliaSim.

## Creating bubbleRob

Start with a fresh instance of CoppeliaSim.  Do not try to have multiple windows of CoppeliaSim open at once. The simulator displays a default [scene](https://www.coppeliarobotics.com/helpFiles/en/scenes.htm).  Save the scene as `bubbleRob_lastName_firstName.ttt`.  Don't forget to save your work often.

We will start by building the body of _BubbleRob_.

### Adding a Sphere

1.  Add a primitive sphere of diameter 0.2 to the scene with [Menu bar > Add > Primitive shape > Sphere]. Adjust the **X-size** item to 0.2, then click **OK**. The created sphere will appear in the [visibility layer](https://www.coppeliarobotics.com/helpFiles/en/layerSelectionDialog.htm) 1 by default, and be [dynamic and respondable](https://www.coppeliarobotics.com/helpFiles/en/designingDynamicSimulations.htm#staticAndRespondable) (since we kept the item **Create dynamic and respondable shape** enabled). This means that _BubbleRob's_ body will be falling and able to react to collisions with other respondable shapes (i.e. simulated by the physics engine). We can see this is the [shape dynamics properties](https://www.coppeliarobotics.com/helpFiles/en/shapeDynamicsProperties.htm): items **Body is respondable** and **Body is dynamic** are enabled.
2.  Start the simulation (via the toolbar button, or by pressing <control-space> in the scene window), and copy-and-paste the created sphere (with [Menu bar > Edit > Copy selected objects] then [Menu bar > Edit -> Paste buffer], or with <control-c> then <control-v>): the two spheres will react to collision and roll away. When you stop the simulation, the duplicated sphere will automatically be removed. This default behaviour can be modified in the [simulation dialog](https://www.coppeliarobotics.com/helpFiles/en/simulationPropertiesDialog.htm). (Make sure to stop the simulation before continuing).
3.  We also want the _BubbleRob's_ body to by usable by the other calculation modules (e.g. [distance calculation](https://www.coppeliarobotics.com/helpFiles/en/distanceCalculation.htm)). For that reason, enable [Collidable](https://www.coppeliarobotics.com/helpFiles/en/collidableObjects.htm), [Measurable](https://www.coppeliarobotics.com/helpFiles/en/measurableObjects.htm) and [Detectable](https://www.coppeliarobotics.com/helpFiles/en/detectableObjects.htm) in the [object common properties](https://www.coppeliarobotics.com/helpFiles/en/commonPropertiesDialog.htm) for that shape, if not already enabled.
4.  In the [shape properties](https://www.coppeliarobotics.com/helpFiles/en/shapeProperties.htm), select **Adjust texture** to open the texture menu.  Textures allow you to load an image onto a shape. Click **Load new texture** and select an image of your choice.  You can adjust the scaling of the texture to change the resolution of the texture on the object. 
5.  Open the [position dialog](https://www.coppeliarobotics.com/helpFiles/en/positionDialog.htm) on the **translation** tab, select the sphere representing _BubbleRob's_ body, and enter 0.02 for **Along Z**. Make sure that the **Relative to**\-item is set to **World**, then click **Translate selection**. This translates all selected objects by 2 cm along the absolute Z-axis, and effectively lifted the sphere a little bit.
6.  In the [scene hierarchy](https://www.coppeliarobotics.com/helpFiles/en/userInterface.htm#SceneHierarchy), double-click the sphere's alias to edit it. Rename to _bubbleRob_ and press enter.

### Adding a Proximity Sensor

Next we will add a [proximity sensor](https://www.coppeliarobotics.com/helpFiles/en/proximitySensors.htm) so that _BubbleRob_ knows when it is approaching obstacles.

1.  Select [Menu bar > Add > Proximity sensor > Cone type].
2.  In the [orientation dialog](https://www.coppeliarobotics.com/helpFiles/en/orientationDialog.htm) on the **Rotation** tab, enter 90 for **Around Y** and for **Around Z**, then click **Rotate selection**.
3.  In the [position dialog](https://www.coppeliarobotics.com/helpFiles/en/positionDialog.htm), on the **position** tab, enter 0.1 for **X-coord.** and 0.12 for **Z-coord.** The proximity sensor is now correctly positioned relative to _BubbleRob's_ body.
4.  Double-click the proximity sensor's icon in the [scene hierarchy](https://www.coppeliarobotics.com/helpFiles/en/userInterface.htm#SceneHierarchy) to open [its properties](https://www.coppeliarobotics.com/helpFiles/en/proximitySensorPropertiesDialog.htm) dialog. Click **Show volume parameter** to open the [proximity sensor volume dialog](https://www.coppeliarobotics.com/helpFiles/en/proximitySensorVolumeDialog.htm). Adjust items **Offset** to 0.005, **Angle** to 30, **Range** to 0.15, **Radius** to 0.0050.
5.  In the [proximity sensor properties](https://www.coppeliarobotics.com/helpFiles/en/proximitySensorPropertiesDialog.htm), click **Show detection parameters**. This opens the [proximity sensor detection parameter dialog](https://www.coppeliarobotics.com/helpFiles/en/proximitySensorDetectionParameterDialog.htm). Uncheck item **Don't allow detections if distance smaller than** then close that dialog again (keep the default value).
6.  In the scene hierarchy, double-click the proximity sensor's alias in order to edit it. Rename it to _proximitySensor_ (no capitals) and press enter.
7.  Select _proximitySensor_, then control-select _bubbleRob_, then click [Menu bar > Edit > Make last selected object parent]. This **attaches** the sensor to the body of the robot. We could also have dragged _proximitySensor_ onto _bubbleRob_ in the scene hierarchy.

The sphere with the proximity sensor should look like this (with your own custom texture):

![](img/bubbleRob_proximity.png)

[Proximity sensor attached to _bubbleRob's_ body]

### Add Wheels

Next, add _BubbleRob's_ wheels.

1.  Add a pure primitive cylinder with dimensions (0.08,0.08,0.02). Enable [Collidable](https://www.coppeliarobotics.com/helpFiles/en/collidableObjects.htm), [Measurable](https://www.coppeliarobotics.com/helpFiles/en/measurableObjects.htm) and [Detectable](https://www.coppeliarobotics.com/helpFiles/en/detectableObjects.htm) in the [object common properties](https://www.coppeliarobotics.com/helpFiles/en/commonPropertiesDialog.htm) for that cylinder, if not already enabled.
3.  In the [position dialog](https://www.coppeliarobotics.com/helpFiles/en/positionDialog.htm), on the **position** tab, set the cylinder's absolute position to (0.05,0.1,0.04)
4. In the [orientation dialog](https://www.coppeliarobotics.com/helpFiles/en/orientationDialog.htm), on the **orientation** tab, st its absolute orientation to (-90,0,0).
4.  Change the alias to _leftWheel_.
5.  Copy and paste the wheel in the scene hierarcy, and set the absolute position Y coordinate of the copy to -0.1.
6.  Rename the copy to _rightWheel_.

If you are having trouble visualizing the new objects as you create them, try creating a new scene with [Menu bar > File > New scene], creating/modifying your new objects there, and then copying your newly created objects back into the original scene. It is often very convenient to work across several scenes, in order to visualize and work only on specific elements.  Creating a new scene is optional, but it is recommended while you are still getting familiar with CoppeliaSim.

### Add Joints/Motors to Wheels

We now need to add [joints](https://www.coppeliarobotics.com/helpFiles/en/joints.htm) (or motors) for the wheels.

1.  Click [Menu bar > Add > Joint > Revolute] to add a revolute joint to the scene. Most of the time, when adding a new object to the scene, the object will appear at the origin of the world.
2.  Keep the joint selected, then control-select _leftWheel_. In the [position dialog](https://www.coppeliarobotics.com/helpFiles/en/positionDialog.htm), on the **position** tab, click the **Apply to selection** button: this positioned the joint at the center of the left wheel.
3.  In the [orientation dialog](https://www.coppeliarobotics.com/helpFiles/en/orientationDialog.htm), on the **orientation** tab, do the same to orient the joint in the same way as the left wheel.
4.  Rename the joint to _leftMotor_.
5.  Double-click the joint's icon in the scene hierarchy to open the [joint properties](https://www.coppeliarobotics.com/helpFiles/en/jointProperties.htm) dialog. Then click **Show dynamic properties** to open the [joint dynamics properties](https://www.coppeliarobotics.com/helpFiles/en/jointDynamicsProperties.htm) dialog. **Enable the motor**, and check item **Lock motor when target velocity is zero**.
6.  Repeat the same procedure for the right motor and rename it to _rightMotor_.
7.  Attach the left wheel to the left motor, the right wheel to the right motor, then attach the two motors to _bubbleRob_ (using the same procedure as above when attaching the proximity sensor to the sphere).

**NOTE:** Observe the common proporties of each of the motors.  You should see that **collidable, measurable, and detectable** are greyed out.  This is because the joint/motor itself is not a physical object in the space, but instead acting upon the objects which it is connected to.  Therefore, the ends of the motor "axles" that extend beyond the weels will not collide and interact with surrounding objects when the robot moves.

The robot should now look like this:

![](img/bubbleRob_wheels.png)

[Proximity sensor, motors and wheels]

### Add 3rd Wheel: Slider/Caster

We run the simulation and notice that the robot is falling backwards. We are still missing a third contact point to the floor. This will be a small sliding caster.

1. Add a pure primitive sphere with diameter 0.05 and make the sphere [Collidable](https://www.coppeliarobotics.com/helpFiles/en/collidableObjects.htm), [Measurable](https://www.coppeliarobotics.com/helpFiles/en/measurableObjects.htm) and [Detectable](https://www.coppeliarobotics.com/helpFiles/en/detectableObjects.htm) (if not already enabled), then rename it to _slider_.
2. In [shape dynamics properties](https://www.coppeliarobotics.com/helpFiles/en/shapeDynamicsProperties.htm), select **Edit material** and  select _noFrictionMaterial_ at the top for **Apply predefined settings**.  NOTE: The menu value will revert back to None, but you should see the values of the other settings below change when you apply the default setting.
3. To rigidly link the slider with the rest of the robot,  add a [force sensor object](https://www.coppeliarobotics.com/helpFiles/en/forceSensors.htm) with [Menu bar > Add > Force sensor]. Rename it to _connection_ and shift the position up by 0.05.
4. Attach the slider to the force sensor.
5. Shift the force sensor by -0.07 along the absolute X-axis, then attach it to the robot body.
6. Run the simulation. Notice that the slider is slightly moving in relation to the robot body: this is because both objects (i.e. _slider_ and _bubbleRob_) are colliding with each other. To avoid strange effects during dynamics simulation, we have to inform CoppeliaSim that both objects do not mutually collide. In the [shape dynamics properties](https://www.coppeliarobotics.com/helpFiles/en/shapeDynamicsProperties.htm), for _slider_ set the **local respondable mask** to 00001111, and for _bubbleRob_, set the **local respondable mask** to 11110000. Run the simulation again and notice that both objects do not interfere anymore.

The robot should now look like this:

![](img/bubbleRob_caster.png)

[Proximity sensor, motors, wheels and slider]

### Physics of bubbleRob

Run the simulation again and notice that _BubbleRob_ slightly moves, even with locked motor. If you try to run the simulation with different physics engines, the result will be different. Stability of dynamic simulations is tightly linked to masses and inertias of the involved non-static shapes. For an explanation of this effect, make sure to carefully read [this](https://www.coppeliarobotics.com/helpFiles/en/designingDynamicSimulations.htm#masses) and [that](https://www.coppeliarobotics.com/helpFiles/en/designingDynamicSimulations.htm#inertias) sections.

This has to do with the objects masses: Keep masses similar and not too light. When linking two shapes with a dynamically enabled joint or a dynamically enabled force sensor, make sure the two shape’s masses are not too different ($m_1 < 10 ∗ m_2$ and $m_2 < 10 ∗ m_1$), otherwise the joint or force sensor might be very soft and wobbly and present large positional/orientational errors (this effect can however also be used as a natural damping sometimes).

Additionally, very low mass shapes should be avoided since they won’t be able to exert very large forces onto other shapes (even if propelled by high force actuators!).

Lastly, the intertia has a role to play: Keep principal moments of inertia* relatively large. Try keeping the principal moments of inertia / mass (*refer to the shape dynamics properties dialog) relatively large, otherwise mechanical chains might be difficult to control and/or might behave in a strange way

To correct for the undesired effect, we need to modify the masses and intertias of the objects:

1. Select the two wheels and the slider (CTRL-click to select multiple objects at once) and open shape dynamics dialog.
2. Multiply the masses by 8 by clicking $M=M*2$ (for selection) three times.  
2. With the two weels and slider selected, multiply the inertia by 8 by clicking  $I=I*2$ (for selection) three times
4. Run the simulation again and the stability should have improved.

### Make the Robot move

In the joint dynamics dialog, set the **Target velocity** to 50 deg/s for both motors. Run the simulation again and notice that _BubbleRob_ now moves forward and eventually falls off the floor. Reset the **Target velocity** item to zero for both motors.

If your robot moves backwards, your motors are spinning in the wrong direction and need to be rotated.  If your robot starts moving in a circle, then the velocities of the motors do not match.

### Add Vision sensor

Next we will add a [vision sensor](https://www.coppeliarobotics.com/helpFiles/en/visionSensors.htm), at the same position and orientation as _BubbleRob's_ proximity sensor.
1. Open the model hierarchy again, then click [Menu bar > Add > Vision sensor > Perspective type], then attach the vision sensor to the proximity sensor, and set the position and orientation of the vision sensor to (0,0,0) relative to the **parent frame** (local).
2. Rename the sensor to _visionSensor_.
3. Make sure the vision sensor is not not visible, not part of the model bounding box, and that if clicked, the model will be selected instead. (hint: object common properties,
change its layer/or camera visibility, and set two other properties).
4. In the [properties dialog](https://www.coppeliarobotics.com/helpFiles/en/visionSensorPropertiesDialog.htm), set the **Far clipping plane** to 1, and the **Resolution x** and **Resolution y** to 256 and 256.
5. Right click on the simulation environment and select [Add > Floating View] to add a floating view to the scene.
 6. Select the vision sensor in the hierarcy and then right click on the new floating view. Select [View > Associate view with selected vision sensor] to associate the sensor to the view.
 7. Run the simulation, and you will see the cylinders in the view.

## Building an Environment for bubbleRob to Interact With

### Add Obstacles

Now we want to surround _bubbleRob_ with obstacles.

1. Add a pure primitive cylinder with following dimensions: (0.1, 0.1, 0.2). We want this cylinder to be static (i.e. not influenced by gravity or collisions) but still exerting some collision responses on non-static respondable shapes. Disable **Body is dynamic** in the [shape dynamics properties](https://www.coppeliarobotics.com/helpFiles/en/shapeDynamicsProperties.htm). We also want our cylinder to be [Collidable](https://www.coppeliarobotics.com/helpFiles/en/collidableObjects.htm), [Measurable](https://www.coppeliarobotics.com/helpFiles/en/measurableObjects.htm) and [Detectable](https://www.coppeliarobotics.com/helpFiles/en/detectableObjects.htm) in the [object common properties](https://www.coppeliarobotics.com/helpFiles/en/commonPropertiesDialog.htm).
2. While the cylinder is still selected, click the object translation toolbar button: ![](https://www.coppeliarobotics.com/helpFiles/en/images/objectShiftButton.jpg)
3. Drag any point in the scene: the cylinder will follow the movement while always being constrained to keep the same Z-coordinate.
4. Copy and paste the cylinder 11 times to create 12 total obstacles, and move them to positions in a circle around _BubbleRob_.  They should be far enough apart that the robot has space to move, but close enough together that the robot cannot fit through them.
5. When done, select the camera pan toolbar button again: ![](https://www.coppeliarobotics.com/helpFiles/en/images/cameraShiftButton.jpg)

TIP: It is most convenient to position the obstacles when looking at the scene from the top. When moving objects, holding down the shift key allows to perform smaller shift steps. Holding down the ctrl key allows to move in an orthogonal direction to the _regular_ direction(s).

## Monitoring the Robot with Graphs

### Track the Robot Position

Next we are going to add a [graph object](https://www.coppeliarobotics.com/helpFiles/en/graphs.htm) to _BubbleRob_ in order to display its trajectory and distance to the closest object over time.

1. Click [Menu bar > Add > Graph] to add a new graph, then rename it to _positionGraph_.
2. Attach the graph to _bubbleRob_, and set the graph's absolute coordinates to (0,0,0.005).
3. Click on the graph settings icon to the far right of the _graph_ object in the hierarchy (looks like three columns or vertical sliders).  Uncheck **show time plots**.  Check **Visible while Simulation not running** and **visible while simulation running**
3. Add a non-threaded [child script](https://www.coppeliarobotics.com/helpFiles/en/childScripts.htm) to the _bubbleRob_ object by right-clicking on _bubbleRob_ in the hierarchy and selecting [Add > Associated Child Script > non-threaded > Lua]
4. Click on the script/paper icon that was added next to the _bubbleRob_ object in the hierarcy to open the script.
5. Paste the following Lua callback code to get the current position of the _bubbleRob_ and graph it.

```lua
function sysCall_init()
    -- get handle of the bubbleRob object where this child script is run
    bubbleRobBase=sim.getObject('.')
    -- get handle of positionGraph object
    graph=sim.getObject('./positionGraph')
    -- initialize graph stream for X position
    objectPosX=sim.addGraphStream(graph,'object pos x','m',1)
    -- initialize graph stream for y position
    objectPosY=sim.addGraphStream(graph,'object pos y','m',1)
    -- initialize graph of the curve
    sim.addGraphCurve(graph,'object pos x/y',2,{objectPosX,objectPosY},{0,0},'m by m') --
end

function sysCall_sensing()
    -- get current position of bubbleRob
    local pos=sim.getObjectPosition(bubbleRobBase,-1)
    -- set graph stream value X
    sim.setGraphStreamValue(graph,objectPosX,pos[1])
    -- set graph stream value Y
    sim.setGraphStreamValue(graph,objectPosY,pos[2])
end
```

7. Set one motor **target velocity** to 50, run the simulation.  You should see _BubbleRob_ position displayed in the graph.
8. Stop the simulation and reset the motor target velocity to zero.

####Important information about scripts and callbacks

The `sysCall_init()` initialization function is executed the first time the child script is called (either at the beginning of the simulation run or whenever the script is added to the running simulation).  

The `sysCall_sensing()` callback function is executed in each simulation step, during the sensing phase of the step.  

More details about the different stages of script execution can be found in the documentation as well as in the [Controlling the Robot with UI & Enhanced Child Scripts](controlling-the-robot-with-ui--enhanced-child-scripts) section below.

Read through the script documentation:
- [Simulation Scripts Overview](https://www.coppeliarobotics.com/helpFiles/en/simulationScripts.htm)
- [The Main Script](https://www.coppeliarobotics.com/helpFiles/en/mainScript.htm)
- [Child Scripts](https://www.coppeliarobotics.com/helpFiles/en/childScripts.htm)

For each of the scripts provided in this guide, it is expected that you will:

- Read through the comments to understand what the scripts are doing
- Use the [API documentation](https://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm) to learn what each of the `sim` functions does.  

### Drawing Paths and Additional Graphs

In addition to monitoring the robot using graphs, we can also draw paths on the environment.  In this section, we will draw the path _bubbleRob_ has taken, draw  distance to the closest obstacle, and graph the change in distnace over time.

1. Click [Menu bar > Add > Graph] to add a new graph, then rename it to _distanceGraph_.
2. Attach the graph to _bubbleRob_, and set the graph's absolute coordinates to (0,0,0.005).
3. Click on the graph settings icon to the far right of the _graph_ object in the hierarchy (looks like three columns or vertical sliders).  Uncheck **show X/Y plots**.  Check **Visible while Simulation not running** and **visible while simulation running**

4. Update the _bubbleRob_ child script with the following to graph the distance and draw the paths. Make sure to read through the comments to understand what the script is doing and use the [API documentation](https://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm) to learn what each of these functions does:

```lua
function sysCall_init()

    -- get handle of the bubbleRob object where this child script is run
    bubbleRobBase=sim.getObject('.')
    -- get handle of positionGraph object
    positionGraph=sim.getObject('./positionGraph')
    -- get handle of distanceGraph object
    distanceGraph=sim.getObject('./distanceGraph')

    -- initialize graph stream for X position
    objectPosX=sim.addGraphStream(positionGraph,'object pos x','m',1)
    -- initialize graph stream for y position
    objectPosY=sim.addGraphStream(positionGraph,'object pos y','m',1)
    -- initialize graph of the curve
    sim.addGraphCurve(positionGraph,'object pos x/y',2,{objectPosX,objectPosY},{0,0},'m by m')

    -- initialize robot collection
    robotCollection=sim.createCollection(0)
    -- add the tree of bubbleRobBase to collection
    sim.addItemToCollection(robotCollection,sim.handle_tree,bubbleRobBase,0)

    -- initialie measurement of distance to closest object
    distanceSegment=sim.addDrawingObject(sim.drawing_lines,4,0,-1,1,{0,1,0})
    -- initialize trace of robot position
    robotTrace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0},nil,nil,{1,1,0})
    -- initialize graph stream for distance
    distStream=sim.addGraphStream(distanceGraph,'bubbleRob clearance','m',0,{1,0,0})
end

function sysCall_sensing()

    -- get current position of bubbleRob
    local pos=sim.getObjectPosition(bubbleRobBase,-1)

    -- set graph stream value X
    sim.setGraphStreamValue(positionGraph,objectPosX,pos[1])
    -- set graph stream value Y
    sim.setGraphStreamValue(positionGraph,objectPosY,pos[2])

    -- check current distance from robot to all other objects
    local result,distData=sim.checkDistance(robotCollection,sim.handle_all)
    if result>0 then
        -- empty the line drawing container for the distance to closest object
        sim.addDrawingObjectItem(distanceSegment,nil)
        -- draw the line between the robot and closest object
        sim.addDrawingObjectItem(distanceSegment,distData)
        -- plot the stream of distances on the distanceGraph
        sim.setGraphStreamValue(distanceGraph,distStream,distData[7])
    end

    -- draw the trace of the robot position (will keep up to the most recent 200 points, based on the initialization)
    sim.addDrawingObjectItem(robotTrace,pos)
end
```
5. Set a **target velocity** of 50 for the left motor and run the simulation: the second graph now displlays the distance to the closest obstacle over time, and the distance segment is visible in the scene too. Additionally, the most recent path taken by the robot is traced on the floor.  **NOTE**: the distance is the closest distance to any point on the _bubbleRob_, not the proximity sensor.
6. Stop the simulation and reset the target velocity to zero.

## Computer Vision

CoppeliaSim provides some basic computer vision image processing out of the box using the [simVision API](https://www.coppeliarobotics.com/helpFiles/en/simVision.htm?).

### Filter the vision sensor image

We will use edge detection to only display the edges of the objects seen by the vision sensor.

1. Add a non-threaded child script to the vision sensor with the following Lua code:

```Lua
function sysCall_vision(inData)
    -- copy the vision sensor image to the work image
    simVision.sensorImgToWorkImg(inData.handle)
    -- perform edge detection on the work image.
    -- threshold for detection is 0.2 (2cm)
    simVision.edgeDetectionOnWorkImg(inData.handle,0.2)
    -- copy the work image to the vision sensor image buffer
    simVision.workImgToSensorImg(inData.handle)
end

function sysCall_init()
    -- init is empty
end
```

If you wanted to have a view with both the "regular" vision output and the edge detection, you would add a second vision sensor and a second floating view.

Vision sensors have a unique set of [vision callback functions](https://www.coppeliarobotics.com/helpFiles/en/visionCallbackFunctions.htm) available. When present for a given vision sensor, the system will call the callback function every time a new image was acquired or applied, allowing the user to perform image processing.

**Independent Exercise (optional)**: Set up the Python interpreter and try converting the various Lua child scripts to Python.

## bubbleRob as a Model

### Combining Objects as One Model

We now need to finish **BubbleRob** as a [model](https://www.coppeliarobotics.com/helpFiles/en/models.htm) definition.
1. Open the [object common properties](https://www.coppeliarobotics.com/helpFiles/en/commonPropertiesDialog.htm) for the model base (i.e. object _bubbleRob_) then check **Object is model**.  There should now be a dashed bounding box that encompasses all objects in the model hierarchy.
2. Select the two joints (motors), the proximity sensor, and the two graphs.  Open the Object Common Properties, check **Ignored by model bounding box** and click **Apply to selection**. The model bounding box now ignores the two joints and the proximity sensor.
3. Select the two joints/motors and the force sensor (_connection_) and disable **camera visibility layer** 2, and enable **camera visibility layer** 10 in Object Common Properties, and **click Apply to selection**. This effectively hides the two joints and the force sensor, since layers 9-16 are disabled by default. At any time we can [modify the visibility layers for the whole scene](https://www.coppeliarobotics.com/helpFiles/en/layerSelectionDialog.htm).
4. To finish the model definition,  select the vision sensor, the two wheels, the slider, and the graph, then enable item **Select base of model instead**.

If we now try to select an object in our model in the scene, the whole model will be selected instead, which is a convenient way to handle and manipulate the whole model as a single object. Additionally, this protects the model against inadvertent modification. Individual objects in the model can still be selected in the scene by click-selecting them with control-shift, or normally selecting them in the scene hierarchy. Finally, collapse the model tree in the scene hierarchy.

### Save the Model

You can save the model for future use in other scenes or as a checkpoint state prior to making modifications.

1. Make sure the velocities on your motors are set to 0.  Run the simulation to make sure it doesn't move.
1. Navigate to [File > Save Model as]
2. Click OK for the warning
3. Click No when asked if you want to use the current thumbnail
4. Click OK after selecting your desired thumbnail on the next screen
5. Choose a location and file name for your model (extension is `.ttm`)

The model can then be loaded back into the scene as desired, using [File > Load model]

**NOTE**: When you save your scene, the model/robot will be saved as an embedded part of the scene file (*.ttt) as well.

Try running the simulation again. Play with it and change parameters, like motor speeds, etc. Try to have one motor spin with a certain speed, another at a different one. Perhaps with the exact same speed, but negative.

## UI & Advanced Scripting

### Controlling the Robot Speed with UI

Next, we want to be able to modulate BubbleRob’s velocity (speed) with an OpenGl-based custom UI. Update the _bubbleRob_ child script with the following lua code.  

```lua
function speedChange_callback(ui,id,newVal)
    -- custom function to change robot speed
    speed=minMaxSpeed[1]+(minMaxSpeed[2]-minMaxSpeed[1])*newVal/100
end

function sysCall_init()

    -- Handle of the bubbleRob object where this child script is run
    bubbleRobBase=sim.getObject('.')
    -- Handle of the left motor
    leftMotor=sim.getObject("./leftMotor")
    -- Handle of the right motor
    rightMotor=sim.getObject("./rightMotor")
    -- Handle of the proximity sensor
    proximitySensor=sim.getObject("./proximitySensor")
    -- Min and max speeds for each motor
    minMaxSpeed={50*math.pi/180,300*math.pi/180}
    -- Tells whether bubbleRob is in forward or backward mode
    backUntilTime=-1
    -- get handle of positionGraph object
    positionGraph=sim.getObject('./positionGraph')
    -- get handle of distanceGraph object
    distanceGraph=sim.getObject('./distanceGraph')
    -- initialize graph stream for X position
    objectPosX=sim.addGraphStream(positionGraph,'object pos x','m',1)
    -- initialize graph stream for y position
    objectPosY=sim.addGraphStream(positionGraph,'object pos y','m',1)
    -- initialize graph of the curve
    sim.addGraphCurve(positionGraph,'object pos x/y',2,{objectPosX,objectPosY},{0,0},'m by m')

    -- initialize robot collection
    robotCollection=sim.createCollection(0)
    -- add the tree of bubbleRobBase to collection
    sim.addItemToCollection(robotCollection,sim.handle_tree,bubbleRobBase,0)
    -- initialie measurement of distance to closest object
    distanceSegment=sim.addDrawingObject(sim.drawing_lines,4,0,-1,1,{0,1,0})
    -- initialize trace of robot position
    robotTrace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0},nil,nil,{1,1,0})
    -- initialize graph stream for distance
    distStream=sim.addGraphStream(distanceGraph,'bubbleRob clearance','m',0,{1,0,0})

    -- Create the custom UI:
        xml = '<ui title="'..sim.getObjectAlias(bubbleRobBase,1)..' speed" closeable="false" resizeable="false" activate="false">'..[[
        <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
        <label text="" style="* {margin-left: 300px;}"/>
        </ui>
        ]]
    ui=simUI.create(xml)
    -- Set speed
    speed=(minMaxSpeed[1]+minMaxSpeed[2])*0.5
    simUI.setSliderValue(ui,1,100*(speed-minMaxSpeed[1])/(minMaxSpeed[2]-minMaxSpeed[1]))

end

function sysCall_actuation()

    -- Read the last state of the proximity sensor (does not perform any new sensing)
    result=sim.readProximitySensor(proximitySensor)
    -- If detected something, set the backward mode:
    if (result>0) then backUntilTime=sim.getSimulationTime()+4 end

    if (backUntilTime<sim.getSimulationTime()) then
        -- When in forward mode, simply move forward at the desired speed
        sim.setJointTargetVelocity(leftMotor,speed)
        sim.setJointTargetVelocity(rightMotor,speed)
    else
        -- When in backward mode, simply backup in a curve at reduced speed
        sim.setJointTargetVelocity(leftMotor,-speed/2)
        sim.setJointTargetVelocity(rightMotor,-speed/8)
    end
end

function sysCall_sensing()
    -- get current position of bubbleRob
    local pos=sim.getObjectPosition(bubbleRobBase,-1)
    -- set graph stream value X
    sim.setGraphStreamValue(positionGraph,objectPosX,pos[1])
    -- set graph stream value Y
    sim.setGraphStreamValue(positionGraph,objectPosY,pos[2])
    -- check current distance from robot to all other objects
    local result,distData=sim.checkDistance(robotCollection,sim.handle_all)
    if result>0 then
        -- empty the line drawing container for the distance to closest object
        sim.addDrawingObjectItem(distanceSegment,nil)
        -- draw the line between the robot and closest object
        sim.addDrawingObjectItem(distanceSegment,distData)
        -- plot the stream of distances on the distanceGraph
        sim.setGraphStreamValue(distanceGraph,distStream,distData[7])
    end
    -- draw the trace of the robot position
    --(will keep up to the most recent 200 points, based on the initialization)
    sim.addDrawingObjectItem(robotTrace,pos)
end

function sysCall_cleanup()
    simUI.destroy(ui)
end
```

The custom `speedChange_callback()` function is called when the UI speed slider is modified.

The `sysCall_actuation()` callback is executed in each simulation pass and allows the robot/simulation to actuate or modify the scene content / robot behavior.

The `sysCall_cleanup()` callback function cleans up the UI when the simulation is stopped.

Most of the child script's system callback functions are called from the main script,  which handles the simulation loop.   At each simulation step, the main script calls (a) "actuation" functions that simulate the motion of the system and (b) "sensing" functions that simulate the sensors. This is set up automatically and you should not need to modify the main script.

## Run and Improve the Simulation

Set the motor velocities back to 50 and run the simulation. Your robot and environment should look like this:

![](img/bubbleRob_simulation.png)

_BubbleRob_ now moves forward while trying to avoid obstacles (in a very basic fashion). While the simulation is still running, change _BubbleRob's_ velocity with the slider. Be aware that the minimum distance calculation functionality might be heavily slowing down the simulation, depending on the environment.

**Independent Exercise (required)**: You should notice that the proximity sensor sometimes goes between the obstacles and does not trigger the robot to turn around, even though the robot is too wide to fit through the gap. Experiment with modifying the parameters of the proximity sensor so that it can more reliably detect the obstacles, even when the robot is headed towards a gap.

After your simulation is working, save your scene.  Then, make a copy of the scene and continue to play around with it. Try copy pasting the bubbleRob model object to add multiple in the scene.  Try scaling their size.

## Control CoppeliaSim Via Remote Python Scripts

Now we want to control our robot remotely through Python scripts.

### Setup the project subdirectory

The CoppeliaSim remote API needs certain files to function properly:

1. Create a new subdirectory for this sample project
2. Navigate to the directory where CoppeliaSim was installed.  On windows, this is likely `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu`
3. Navigate to `programming\remoteApiBindings\python\python` and copy the `sim.py` and `simConst.py` modules into your project directory.
4. Then copy the lib file for your appropriate OS from `programming\remoteApiBindings\lib\lib\<yourOS>` into your project directory

### Prepare your scene for remote access

Back in your original scene (not the copy you were playing around with), we want to disable the native Lua script that controls _bubbleRob_ and instead, tell CoppeliaSim to listen for remote API control.

1. Disable the child script on _bubbleRob_ by clicking on the "Scripts" button on the left menu rail (looks like a piece of paper), selecting **Child Script "/bubbleRob"** and checking **Disabled** under **Script Properties**.
2. Add a **threaded** Lua child script to any of the cylinders and the following line to `coroutineMain()` to start the RemoteAPI server service on port 19999:
```lua
simRemoteApi.start(19999)
```

This temporary remote server service will only be up and running when the simulation is running in CoppeliaSim (play button is clicked).

Read more about [Enabling the Remote API - Server Side](https://www.coppeliarobotics.com/helpFiles/en/remoteApiServerSide.htm) and how the [Remote API works](https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm).  For reference, the continuous API server (not tied to a child script) is on port 19997.

### Python Script to Control bubbleRob

The following code uses the CoppeliaSim remote API `sim` package that we copied into the working directory.

Read more about the [Python Remote API functions](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) and the [Remote API constants / error codes](https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm)

The script below recreates the basic object avoidance functionality of the Lua scripts we wrote within CoppeliaSim.  Read the code carefully and cross-reference with the API documentation.

Save the script in your working directory as bubbleRob_demo.py

```python
import sim
import time
import sys

sim.simxFinish(-1) # just in case, close all opened connections

# Get the current client ID
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)

print(clientID)

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

# read the current state of the prximity_sensor
# NOTE: simxReadProximitySensor() returns a lot of information. This script will only use detectionState, but read up on the rest of the returned objects.
(err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector
 ) = sim.simxReadProximitySensor(clientID, proximity_sensor, sim.simx_opmode_streaming)

if err_code > 1:
    sys.exit(f'error reading proximitySensor: {err_code}')

speed = 2

err_code = sim.simxSetJointTargetVelocity(clientID,left_motor, speed ,sim.simx_opmode_streaming)
err_code = sim.simxSetJointTargetVelocity(clientID,right_motor, speed ,sim.simx_opmode_streaming)

start_time = time.time() #record the initial time

obs = False # flag to identify if obstacle was reached
dt = 0
t = start_time

while (t-start_time)<30: #run for 30 seconds

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

    # read the proximity sensor.  Use the buffer mode for subsequent reads
    (err_code, detectionState,detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector
     ) = sim.simxReadProximitySensor(clientID, proximity_sensor, sim.simx_opmode_buffer)

    # update the time
    t = time.time()

# stop the simulation when complete (you should see the CoppeliaSim UI reset)
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)

print("Done")

```

Start the simulation in CoppeliaSim, then run the script in your IDE.  You should see _bubbleRob_ move around and react to the obstacles.

## Submit Your Work

Save your final CoppeliaSim scene file as `bubbleRob_lastName_firstName.ttt` with your name, and upload it to Canvas

## Continue to Explore

This is only a small example of what you can do with the Remote API.  Explore more and try to recreate more of the Lua functionality that we programmed natively in CoppeliaSim. Experiment with capturing the images from the _visionSensor_.  Also, try combining running a non-threaded Lua child script for just the graphs with the remote API control of the robot movement.  If you get stuck, reach out to the instructor or TA with questions.  If there are enough questions, we will cover more demos in later class sessions.
