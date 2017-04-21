#ifndef HUMANMGR_H
#define HUMANMGR_H

#include <vector>
#include <string>
#include <map>

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
namespace ros{
class NodeHandle;
}
namespace pluginlib{
template<class T>
class ClassLoader;
}
namespace move3d{

class SceneManager;
class BaseHumanUpdater;
class HumanSettings;
/**
 * @brief HumanSettingsPtr
 */
typedef boost::shared_ptr<HumanSettings> HumanSettingsPtr;

/**
 * @brief The HumanMgr class updates human configuration based on plugins
 *
 * The representation of humans can vary a lot between systems, according to sensors used and information needed.
 * move3d_ros_lib uses a plugin system to manage the conversion from the input data to a full configuration usable inside move3d.
 *
 * The motivation of the HumanMgr is that most often we don't know the configuration of humans from sensors, but in move3d we may need to
 * infer them in order to do some computations (costs, affordances, collisions,...). The HumanMgr allows one to specify how a specific human
 * have to be handled. Plus, the plugin architecture allows a developper to add a different policy fiting its specific needs.
 */
class HumanMgr
{
public:

    typedef std::map<std::string,HumanSettingsPtr>::iterator HumanSettingsIterator;

    /// constructor
    HumanMgr(ros::NodeHandle *nh,SceneManager *sc);
    ~HumanMgr();

    /**
     * @brief read the ROS parameter server to find settings
     * @return true if the parameter could be parsed successfully
     *
     * searches in the nh namespace a dictionary called "human_mgmt". Its key are human names (in ROS, i.e. in the input) and data is another dictionary containing the following fields:
     * * move3d_name: the name of the human in move3d (p3d), it can be different from the ROS name
     * * policy: the name of the plugin to use
     * * data: a dictionary of floats given to the plugin. It contains user set parameters
     */
    bool updateParams();
    /**
     * @brief load the plugins found by the pluginlib ClassLoader
     * @return
     */
    bool loadPlugins();

    /**
     * @brief update a human configuration in move3d
     * @param name input (ROS) human name, the key of the human_mgmt dictionary in ROS parameter server
     * @param base position of the base (1st joint), used only by the corresponding plugin
     * @param joints joint positions, used only by the plugin
     * @return
     */
    bool setHumanPos(const std::string &name, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints);

    /**
     * @brief list of humans defined in the ROS parameter server
     * @return
     * @see updateParams()
     */
    std::vector<std::string> humanList() const;
    /**
     * @brief get the structure representing the settings for a human
     * @param name name of the human (input/ROS name)
     * @return a pointer to a HumanSettings structure
     */
    HumanSettingsPtr getHumanSettings(const std::string &name);

private:
    bool updateHumanList();

    ros::NodeHandle *_nh;
    SceneManager *_scMgr;
    std::vector<std::string> _humans;
    std::map<std::string,HumanSettingsPtr> _humanSettings;
    pluginlib::ClassLoader<BaseHumanUpdater> *_human_updater_loader;
    std::map<std::string,boost::shared_ptr<BaseHumanUpdater> > _updaters;
    typedef std::map<std::string,boost::shared_ptr<BaseHumanUpdater> >::iterator UpdaterIterator;

};
}

#endif // HUMANMGR_H
