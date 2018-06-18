

# Robot Kinematics & Dynamics Toolkit

This project includes utilities that can assist the development of a robot's kinematic model.

## Features

This toolkit is currently capable of performing the multiple functions.
The feature set in this toolkit will continue to expand with time.

### Kinematics Expression Compiler

Given a robot specification using [Denavit-Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters),
a robot's joint positions can be mathematically related to the pose of the robot's end effector.
This toolkit exposes the Denavit-Hartenberg parameter technique using Transforms:

```
Transform(θ, d, a, α, joint_type)
    θ : Angle about previous z, from old x to new x
    d : Offset along previous z to the common normal
    a : Length of the common normal
        Assuming a revolute joint, this is the radius about previous z
    α : Angle about common normal, from old z axis to new z axis

Joint Types:
    Static    : No actuated joint, transform is static
    Prismatic : The transform 'd' parameter is set in accordance to the joint's value
    Revolute  : The transform 'θ' patameter is set in accordance to the joint's value

Note that the right hand rule for specifying axis orientation is always used.
```

A robotic manipulator is specified by creating a list of transforms, thus creating a kinematic chain.

This toolkit provides support for realizing a robot's **forward kinematics** and **differential kinematics**.

### Robot Renderer

A simple renderer is used to show a visual representation of the kinematic chain.


