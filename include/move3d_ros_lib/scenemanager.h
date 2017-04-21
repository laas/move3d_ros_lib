#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H

#include <string>
#include <set>
#include <vector>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

/**
 * @namespace move3d
 * @brief The base namespace of the move3d_ros_lib
 */

class Project;
namespace move3d
{
class HumanMgr;

/**
 * @brief The SceneManager class provide interface between ROS and the move3d scene and project.
 * Use it to start libmove3d and place the robots and objects in the environment.
 */
class SceneManager
{
public:
    /**
     * @brief default constructor
     * @param nh
     */
    SceneManager(ros::NodeHandle *nh);
    /**
     * @brief SceneManager
     * @param nh
     * @param p3d_path
     */
    SceneManager(ros::NodeHandle *nh,const std::string &p3d_path);

    ~SceneManager();

    /**
     * @brief add a move3d module to be initialized
     * @param name name of the module
     * @return true on success, false otherwise.
     *
     * The project must be NOT initialized, otherwise does nothing and return false.
     * So you need to call this BEFORE calling createScene()
     */
    bool addModule(const std::string &name);

    /**
     * @brief create the move3d scene and project.
     * @return false on error
     *
     * A path to a p3d file must be given in the constructor or with setP3dPath().
     * To add a module, you need to call addModule() before  createScene().
     * This is required in order to use most of the components of move3d.
     * @see setProject allows to set a already initialized project
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
     * @brief updatea human position
     * @param name name of the human in ROS
     * @param base_pose position of the base joint of the human
     * @param joints other joint positions (not configuration) in an absolute frame
     * @return false if the human is not found in the HumanMgr settings or in the move3d scene, or if the configuration cannot be computed
     * @see HumanMgr
     */
    bool updateHuman(const std::string &name, const geometry_msgs::Pose &base_pose, const std::map<std::string,geometry_msgs::Pose> &joints);

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

    /**
     * @brief retrieve map of joint/dof correspondances
     * @param param_name where the dictionnary is located in the ROS parameter server
     * @param robot_name the (complete) name of the robot in move3d
     * @return false if the project is not correctly initialized. Empty or non-existent map is ok.
     *
     * The dictionnary in ROS parameters have the form:
     * ~~~{.yaml}
     * param_name : {
     *      ros_joint_name: move3d_joint_name,
     * }
     * ~~~
     *
     * So as an example, you could put in a .yaml file to load:
     *
     * ~~~{.yaml}
     *
     * ---
     * move3d:
     * dof_name_corresp: {
     *    PR2_ROBOT: {
     *      torso_lift_link: Torso,
     *      head_pan_link: pan_cam,
     *      head_tilt_link: tilt_cam,
     *      ...
     *    }
     *  }
     * ~~~
     */
    bool fetchDofCorrespParam(const std::string &param_name, const std::string &robot_name);
    /**
     * @brief retrieve map of joint/dof correspondances for all robots
     * @param base_param_name where the dictionnaries are located in the ROS parameter server
     * @return false if the project is not correctly initialized. Empty or non-existent map is ok.
     *
     * For each robot agent in the scenes, it tries to find a dictionnary of joint names correspondances under base_param_name/<robot_type>, and if not found, under base_param_name/<robot_name>.
     *
     * @see SceneManager::fetchDofCorrespParam(const std::string&, const std::string&)
     */
    bool fetchDofCorrespParam(const std::string &base_param_name);

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
     * prefer absolute paths or ROS package:// url
     */
    void setP3dPath(const std::string &p3dPath);

    /** @see setScePath */
    std::string scePath() const;
    /**
     * @brief set the path to a sce file to load on startup
     * @param scePath the path
     * prefer absolute paths or ROS package:// url
     */
    void setScePath(const std::string &scePath);

    /**
     * @brief save the scenario, ie. the positions and configurations of all objects and robots
     * @param path the path where to save the scenario
     * @return true on success
     * The created scenario (.sce) is compatible with any move3d instance, as move3d-studio.
     */
    bool saveScenario(const std::string &path);


    /// @todo not implemented
    Eigen::Affine3d getFrameTransform() const;
    /// @todo not implemented
    void setFrameTransform(const Eigen::Affine3d &frame_change);

    /**
     * @brief updateAcceptBaseOnly indicates the possibility to consider only the base position of an agent.
     * @return
     *
     * When True, calls to updateRobot with empty dof_values will result in the same as calling updateRobotPose.
     * Otherwise the robot is not updated.
     * @see setUpdateAcceptBaseOnly()
     */
    bool updateAcceptBaseOnly() const;
    /**
     * @brief setUpdateAcceptBaseOnly
     * @param updateAcceptBaseOnly
     * @see updateAcceptBaseOnly()
     */
    void setUpdateAcceptBaseOnly(bool updateAcceptBaseOnly);

    //getter / setters

    /**
     * @brief get the move3d project object
     * @return
     */
    Project *project() const;
    /**
     * @brief set a project instead of creating it with createScene
     * @param project a move3d project
     * @see createScene handles the creation of the project
     */
    void setProject(Project *project);

    /// get the node handle
    ros::NodeHandle *nh() const;
    /// set the node handle
    void setNh(ros::NodeHandle *nh);

    typedef std::map<std::string,std::string> NameMap_t;
    typedef std::map<uint,uint> UintMap_t;
    typedef  std::map<std::string,NameMap_t> JointCorrespName_t;
    typedef std::map<std::string,UintMap_t> JointCorrespIndex_t;


private:
    /// called in the constructor only
    void preInit();
    ros::NodeHandle *_nh;
    std::string _p3dPath;
    std::string _scePath;

    std::set<std::string> _modules_to_activ;

    Project *_project;

    JointCorrespName_t _jointCorrespNames;
    JointCorrespIndex_t _jointCorrespIndex;

    HumanMgr *_humanMgr;

    Eigen::Affine3d _frame_transform;

    bool _updateAcceptBaseOnly;

};

}
#endif // SCENEMANAGER_H
