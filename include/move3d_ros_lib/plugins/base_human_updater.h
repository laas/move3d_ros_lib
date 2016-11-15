#ifndef BASEHUMANUPDATER_H
#define BASEHUMANUPDATER_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <map>

class Robot;
class RobotState;

namespace move3d
{
class BaseHumanUpdater;
/** @brief The HumanSettings struct stores data for each human.
 */
struct HumanSettings{
    HumanSettings():policy(""){}
    HumanSettings(const std::map<std::string, float> &settings, const std::string &policy, const std::string &move3d_name)
    {
        this->policy=policy;
        data=settings;
        this->move3d_name=move3d_name;
    }
    bool getData(std::string key,float &v) const{
        std::map<std::string,float>::const_iterator it=data.find(key);
        if(it!=data.end()){
            v=it->second;
            return true;
        }
        return false;
    }
    std::string policy;
    boost::shared_ptr<BaseHumanUpdater> updater;
    std::map<std::string,float> data;
    std::string move3d_name;

};
typedef boost::shared_ptr<HumanSettings> HumanSettingsPtr;

/**
 * Base class for implementing plugin human updater.
 *
 * BaseHumanUpdater provides the interface for plugin classes that handle the translation of some specific human description to move3d
 *
 * @see http://wiki.ros.org/pluginlib
 */
class BaseHumanUpdater
{
public:
    virtual ~BaseHumanUpdater(){}

    /**
     * @brief update the robot with the given elements
     * @param h
     * @param base
     * @param joints
     * @param settings
     * @return
     */
    virtual bool update(Robot *h,const Eigen::Affine3d &base,const std::map<std::string,Eigen::Affine3d> &joints,const HumanSettings &settings)=0;
    /**
     * @brief computeConf
     * @param[in] h
     * @param[in] base
     * @param[in] joints
     * @param[in] settings
     * @param[out] q
     * @return
     */
    virtual bool computeConf(Robot *h,const Eigen::Affine3d &base,const std::map<std::string,Eigen::Affine3d> &joints,const HumanSettings &settings, RobotState &q)=0;

protected:
    BaseHumanUpdater(){}

};



}

#endif
