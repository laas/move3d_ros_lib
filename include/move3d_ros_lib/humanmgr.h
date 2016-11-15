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
typedef boost::shared_ptr<HumanSettings> HumanSettingsPtr;

class HumanMgr
{
public:

    typedef std::map<std::string,HumanSettingsPtr>::iterator HumanSettingsIterator;

    HumanMgr(ros::NodeHandle *nh,SceneManager *sc);
    ~HumanMgr();

    bool updateParams();
    bool loadPlugins();

    bool setHumanPos(const std::string &name, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints);

    std::vector<std::string> humanList() const;
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
