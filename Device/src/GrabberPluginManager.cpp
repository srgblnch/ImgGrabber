/*!
 * \file
 * \brief    Definition of GrabberTask class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#include <GrabberPluginManager.h>

namespace GrabAPI
{
  std::pair<yat::IPlugInInfo*, yat::IPlugInFactory*> 
    GrabberPluginManager::load( const std::string &library_file_name )
  {
    return this->plugin_manager.load(library_file_name);
  }

}
