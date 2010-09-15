/*!
 * \file
 * \brief    Declaration of IMAQManager class
 * \author   Julien Malik - Synchrotron SOLEIL
 */
#ifndef _IMAQMANAGER_H_
#define _IMAQMANAGER_H_

#include <set>

namespace GrabAPI
{
  class IMAQGrabber;

  class IMAQManager
  {
  public:

    static IMAQManager& instance();

    void register_grabber( IMAQGrabber* grabber );
    
    void deregister_grabber( IMAQGrabber* grabber );

    void enable( IMAQGrabber* grabber );


  private:
    IMAQManager();
    IMAQManager( const IMAQManager& );
    ~IMAQManager();

    std::set< IMAQGrabber* > grabber_set_;
  };

} // namespace



#endif
