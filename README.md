

# Robot Kinematics & Dynamics Toolkit

This project includes utilities that can assist the development of a robot's kinematic model.

## Features

This toolkit is currently capable of performing the multiple functions.
The feature set in this toolkit will continue to expand with time.
Current features include:
* Forward & Differential Kinematics expression compilation
* Visual robot rendering
* Kinematic chain support

Future features include:
* Inverse dynamics expression compilation
* Robot dynamics simulation
* Kinematic tree support
* Unit tests

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
See **example.cpp** for a sample of how to create a robot manipulator.

This toolkit provides support for realizing a robot's **forward kinematics** and **differential kinematics**.

#### Compiled Kinematics Performance

In tests on my machine, the compiled forward kinematics calculations execute **359.3x faster** than forward kinematics applied by sequentially multiplying frame transforms for a simple 3-link RRR manipulator.
This test was executed by running 100k forwards kinematics requests using both methods.
The compiled method calculated all 100k requests in 0.06806 seconds, whereas the frame transform approach calculated all requests in 24.4526 seconds.

### Robot Renderer

A simple renderer is used to show a visual representation of the kinematic chain.
This is intended to assist in specifying the transforms that compose the robot geometry.

![](https://github.com/sjsimps/Robotics-Tools/blob/master/example_render.webm)

This shows a sample rendering of a robot with 6 revolute joints, each joint rotating at constant velocity in the same direction.

### Dependencies

[Symbolic C++](http://issc.uj.ac.za/symbolic/symbolic.html) is copied directly into this repository.
I have not modified nor contributed to the library, therefore all credit for Symbolic C++ goes to its developers.
Symbolic C++ is used to symbolically compute the differential and forward kinematics prior to compilation.

[SDL 2.0](https://wiki.libsdl.org/Installation) is required to render the arm model visually.

