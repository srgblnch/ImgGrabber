/*!
 * \file
 * \brief    Definition of IMAQManager class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#include "IMAQManager.h"
#include "IMAQGrabber.h"

namespace GrabAPI
{
  IMAQManager::IMAQManager()
  {
  }

  IMAQManager::~IMAQManager()
  {
  }

  IMAQManager& IMAQManager::instance()
  {
    static IMAQManager unique_instance;
    return unique_instance;
  }

  void IMAQManager::register_grabber( IMAQGrabber* grabber )
  {
    grabber_set_.insert( grabber );
  }

  void IMAQManager::deregister_grabber( IMAQGrabber* grabber )
  {
    grabber_set_.erase( grabber );
  }

  void IMAQManager::enable( IMAQGrabber* grabber )
  {
    if ( grabber_set_.count( grabber ) == 0 )
    {
      THROW_YAT_ERROR("GRABBER_ERROR",
                      "This grabber is not registered",
                      "IMAQManager::enable");
    }

    std::set< IMAQGrabber* >::iterator it;
    std::set< IMAQGrabber* >::iterator end = grabber_set_.end();
    for ( it = grabber_set_.begin(); it != end; ++it )
    {
      if ( *it != grabber )
      {
        (*it)->close();
      }
    }

    //grabber->open();

  }
}
