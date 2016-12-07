# move3d_ros_lib

move3d_ros_lib provide an interface between the core of move3d libraries such
as the modification of robot positions, the creation and initialisation of the
world representation and the modules of move3d.

## building

It requires to have a recent versions of move3d libraries installed. It will
search for them under the `${ROBOTPKG_BASE}` directory, so that environment
variable must be set properly.

See the documentation of [move3d][move3d-doc] for instructions on the
installation of these libraries (you don't need the move3d-studio executable,
only the libraries: libmove3d, libmove3d-planners, libmove3d-hri; check the
specific installation instructions for ROS).

move3d_ros_lib is a catkin package, so build it using `catkin_make install` in
your catkin workspace.

[move3d-doc]: https://redmine.laas.fr/projects/move3d/wiki/Wiki

## Usage

TODO

You can refer to the documentation in the headers, and to move3d_facts package as an example.
