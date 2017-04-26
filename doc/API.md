move3d_ros_lib provide an interface between the core of move3d libraries such
as the modification of robot positions, the creation and initialisation of the
world representation and the modules of move3d.

## Installation
See \ref README.md

## API

move3d_ros_lib main functionalities are provided by the \ref move3d::SceneManager class, that deals with initialization of move3d,
and conversion of positions and configurations from ROS datatypes to move3d.

### World representation

Move3d uses a full world representation, that is it assumes it knows the exact positions of every object, robot, human and obstacle.
It needs to be fed with the geometric representation of the environment (walls, ceiling,...), movable objects, robots, and humans. This is done with
a file in a specific format: p3d. Move3d libraries comes with some [p3d files] you can take as example while we prepare a nice documentation on how to create them.

Individual objects, robots and human can be represented as specific move3d files (p3d .macro files) or as standard URDF files. Refer to the [move3d documentation] for more
information on models used and needed.

[p3d files]:  https://www.openrobots.org/wiki/move3d#P3D_Files
[move3d documentation]: https://redmine.laas.fr/projects/move3d/wiki


### Humans

One of the strengths of move3d is that it takes humans into account for lots of computations. The problem is that according to the systems (hardware and software) used,
the data we get about human position and gesture cannot be normalized. We may for example have only a face recognition algorithm that give the position of the noose, but for
navigation we would like to now the posture of the human, so we need to infer that. Or on tabletop scenarios, we may have only face and hands positions, and we will infer the
elbow position to compute collisions with the full arms of the human.
The solution we propose is to use plugins for that, so each system developer can create its
code that manages the humans in its own way. Meanwhile we propose some plugins that can be used in some simple cases.

