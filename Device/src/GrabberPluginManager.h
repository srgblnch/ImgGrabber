/*!
 * \file
 * \brief    Declaration of GrabberPluginManager class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#ifndef _GRABAPI_GRABBERPLUGINMANAGER_H_
#define _GRABAPI_GRABBERPLUGINMANAGER_H_

#include <yat/Singleton.h>
#include <yat/plugin/PlugInManager.h>

namespace GrabAPI
{

class GrabberPluginManager : public yat::Singleton<GrabberPluginManager>
{
public:

  std::pair<yat::IPlugInInfo*, yat::IPlugInFactory*> load( const std::string &library_file_name );

private:
  yat::PlugInManager plugin_manager;
};

} // namespace

#endif
