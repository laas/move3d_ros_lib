#include "move3d_ros_lib/tools.h"

#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string/find.hpp>
#include <ros/rosout_appender.h>
#include <ros/package.h>

namespace move3d{
namespace Tools {

std::string getPath(const std::string &url )
{
    typedef const boost::iterator_range<std::string::iterator> StringRange;
    boost::filesystem::path path;
    std::string lurl(url),packstr("package://");

    boost::iterator_range<std::string::iterator> range =
            boost::algorithm::ifind_first(StringRange(lurl.begin(),lurl.end()),
                                          StringRange(packstr.begin(),packstr.end()));
    if (range)
    {
        const std::string chopped_url(range.end(),lurl.end());
        std::string package_name = chopped_url.substr(0,chopped_url.find_first_of('/'));
        std::string file_name = chopped_url.substr(chopped_url.find_first_of('/')+1,-1);
        path = ros::package::getPath(package_name);
        if(path.empty()){
            ROS_WARN("package not found for url %s",url.c_str());
        }
        path = path / file_name;
    }
    else
    {
        path = lurl;
    }
    return path.string();
}

}
}
