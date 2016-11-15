#include "move3d_ros_lib/robots_tf_publisher.h"
#include "move3d_ros_lib/scenemanager.h"

#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/API/scene.hpp>
#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <libmove3d/include/P3d-pkg.h>


#include <ros/ros.h>
#include <tf/tf.h>

#ifndef foreach
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#endif


using namespace std;

namespace move3d {

RobotsTfPublisher::RobotsTfPublisher(SceneManager *scMgr, ros::NodeHandle *nh):
    _scMgr(scMgr), _nh(nh)
{
    _tf_br = new tf::TransformBroadcaster();
}

RobotsTfPublisher::~RobotsTfPublisher()
{
    delete _tf_br;
}

bool RobotsTfPublisher::publishTf(const std::vector<std::string> &names,const ros::Time &date)
{
    Scene *sc = _scMgr->project()->getActiveScene();
    foreach(string n,names){
        Robot *r=sc->getRobotByName(n);
        if(r){
            this->publishTf(r,date);
        }

    }

}

bool RobotsTfPublisher::publishTf(Robot *r, const ros::Time &date)
{
    if(!r) return false;

    tf::Transform t;
    int count(0);
    for(uint i=0;i<r->getNumberOfJoints();++i){
        Joint *jnt=r->getJoint(i);
        Eigen::Affine3d pos=jnt->getMatrixPos();
        t.setFromOpenGLMatrix(pos.data());
        count += r->getRobotStruct()->joints[i]->n_link_jnt+r->getRobotStruct()->joints[i]->n_link_jnt_owned;
        string body_name(r->getRobotStruct()->joints[i+1]->o->name);
        body_name[body_name.find_first_of(".")]='/';
        //_tf_br->sendTransform(tf::StampedTransform(t,date,"move3d_origin",r->getName()+"/"+jnt->getName()));
        _tf_br->sendTransform(tf::StampedTransform(t,date,"move3d_origin",body_name));
    }
}


} // namespace move3d

