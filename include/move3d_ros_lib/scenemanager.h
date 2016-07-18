#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H

#include <string>
#include <set>
#include <vector>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class Project;

/**
 * @brief The SceneManager class provide interface between ROS and the move3d scene and project.
 * Use it to start libmove3d and place the robots and objects in the environment.
 */
class SceneManager
{
public:
    SceneManager();
    SceneManager(const std::string &p3d_path);
    ~SceneManager();

    /**
     * @brief add a move3d module to be initialized
     * @param name name of the module
     * @return true on success, false otherwise.
     * The project must be NOT initialized, otherwise does nothing and return false.
     * So you need to call this BEFORE calling createScene()
     */
    bool addModule(const std::string &name);

    /**
     * @brief create the move3d scene and project.
     * @return false on error
     * A path to a p3d file must be given in the constructor or with setP3dPath().
     * To add a module, you need to call addModule() before  createScene().
     * This is required in order to use most of the components of move3d.
     */
    bool createScene();

    /**
     * @brief update the base pose of a robot (any moveable obect).
     * @param name name of the robot/object
     * @param base_pose the pose where to place the robot
     * @return false on error (robot does not exist,...)
     */
    bool updateRobotPose(const std::string &name, const geometry_msgs::Pose &base_pose);
    /**
     * @brief update a robot configuration (any moveable object).
     * @param name name of the robot to place
     * @param base_pose the base pose (pose of the 1st joint)
     * @param dof_values values of the other degrees of freedom (ordered as in ROS)
     * @return false on error
     */
    bool updateRobot(const std::string &name, const geometry_msgs::Pose &base_pose, const std::vector<double> &dof_values);
    /**
     * @brief update a robot configuration (any moveable object).
     * @param name name of the robot to place
     * @param dof_values value of all the degrees of freedom (ordered as in move3d)
     * @return
     * The position of the 1st joint is specified as the first 6 DoFs
     * of the vector: (x,y,z,rx,ry,rz)
     */
    bool updateRobotM3d(const std::string &name, std::vector<double> &dof_values);
    /// as updateRobotPose(string,Pose)
    bool updateObject(const std::string &name, const geometry_msgs::Pose &pose);

    /**
     * @brief set the matching of ROS DoF names and move3d ones.
     * @param robot_name the name of the robot
     * @param ros_to_move3d_map keys are the name of the joints in ROS, values are the corresponding names in move3d.
     */
    void setDofCorresp(const std::string &robot_name, const std::map<std::string,std::string> &ros_to_move3d_map);
    /**
     * @brief set the DoF order as it will be when calling any updateRobot method.
     * @param robot_name the name of the robot
     * @param dof_names the vector of dof names that will be given to updateRobot for that robot
     * @return
     * This allows the updateRobot methods to avoid parsing strings at every call.
     * It is required to call this method after setDofCorresp and BEFORE any call to updateRobot() with the same robot_name.
     */
    bool setDofNameOrdered(const std::string &robot_name,const std::vector<std::string> &dof_names);

    bool fetchDofCorrespParam(const std::string &param_name, const std::string &robot_name);

    /**
     * @brief convert a configuration from ROS to move3d.
     * Only changes joint order, and remove joints that do not exist in Move3d.
     * @param[in] robot_name
     * @param[in] ros_conf
     * @param[out] m3d_conf
     * @return false on error
     */
    bool convertConfRosToM3d(const std::string &robot_name, const std::vector<double> &ros_conf, std::vector<double> &m3d_conf);

    /**
     * @brief get the path to the p3d file.
     * @return the path
     */
    std::string p3dPath() const;
    /**
     * @brief set the path to the p3d file used to describe the environment and its robots and objects.
     * @param p3dPath the path
     * prefer absolute paths
     */
    void setP3dPath(const std::string &p3dPath);

    /**
     * @brief save the scenario, ie. the positions and configurations of all objects and robots
     * @param path the path where to save the scenario
     * @return true on success
     * The created scenario (.sce) is compatible with any move3d instance, as move3d-studio.
     */
    bool saveScenario(const std::string &path);


    Eigen::Affine3d getFrameTransform() const;
    void setFrameTransform(const Eigen::Affine3d &frame_change);

private:
    /// called in the constructor only
    void preInit();
    std::string _p3dPath;

    std::set<std::string> _modules_to_activ;

    Project *_project;

    typedef std::map<std::string,std::string> NameMap_t;
    typedef std::map<uint,uint> UintMap_t;
    typedef  std::map<std::string,NameMap_t> JointCorrespName_t;
    typedef std::map<std::string,UintMap_t> JointCorrespIndex_t;

    JointCorrespName_t _jointCorrespNames;
    JointCorrespIndex_t _jointCorrespIndex;

    Eigen::Affine3d _frame_transform;

};

#endif // SCENEMANAGER_H
