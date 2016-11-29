#ifndef TOOLS_H
#define TOOLS_H

#include <string>

namespace move3d{
namespace Tools {

/**
 * @brief getPath can take package:// or absolute URL and return absolute path
 * @param url
 * @return
 */
std::string getPath(const std::string &url );

}
}

#endif // TOOLS_H
