# Motor Controller Firmware
This library is responsible for processing the controller's data and driving the motor controllers  that spin the wheels. There are two key classes: MotorController and MotorSet. It is important to understand each of their purposes.

#### MotorController
MotorController handles the turning of one individual motor controller. Each instance of MotorController is its own motor controller. In the rover, there are six motor controllers total (one for each wheel).

#### MotorSet
MotorSet handles multiple MotorController instances. There is a left MotorSet and a right MotorSet. Each MotorSet is responsible for telling each of its MotorControllers what to do. Since none of the  wheels on the left side are ever going to spin in different directions (as for the right side), it is easier to tell one MotorSet to spin forward or backward. In addition, when using a MotorSet the user does not need to worry about the number of motors. 

### Data Handling
This library is responsible for processing the data of the controller. The data is received as an array of five floats.
<nbsp>
float dataArr[5]
<nbsp>
dataArr[0] --> Left Trigger
dataArr[1] --> Right Trigger
- A value of 0.99... refers to a resting trigger
- A value of -0.99... refers to a compressed trigger

dataArr[2] --> Left Shoulder
dataArr[3] --> Right Shoulder
- A value of 0 refers to a resting shoulder
- A value of 1 refers to a compressed shoulder

dataArr[4] --> DPad
- A value of 0 refers to a resting DPad
- A value of 1 refers to left pressed on the DPad
- A value of -1 refers to right pressed on the DPad

Each trigger moves the wheels of the corresponding side forward. When the left trigger is pressed, the rover's left wheels move forwards. When the right trigger is pressed, the rover's right wheels move forwards. When both triggers are pressed, all of the rover's wheels move forwards.
<nbsp>
Each bumper moves the wheels of the corresponding side backward. When the left bumper is pressed, the rover's left wheels move backwards. When the right bumber is pressed, the rover's right wheels move backwards. When both triggers are pressed, all the rover's wheels move backwards.
<nbsp>
The DPad is used to spin the rover. When the left side of the DPad is pressed, the rover spins counter clockwise (to the left). When the right side of the DPad is pressed, the rover spins clockwise (to the right).
<nbsp>
There is a priority to the controller inputs. Triggers take priority over all other inputs, and the shoulders take priority over the DPad.
<nbsp>
Priority List (1 is the highest priority)
1. Triggers
2. Shoulders
3. DPad

