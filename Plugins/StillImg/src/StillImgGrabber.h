/*!
 * \file
 * \brief    Declaration of StillImgGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#ifndef _STILLIMGGRABBER_H_
#define _STILLIMGGRABBER_H_

// ============================================================================
// DEPENDENCIES
// ============================================================================
#include <IGrabber.h>
#include <yat/threading/Task.h>
#include <isl/Image.h>
#include <isl/Exception.h>

namespace GrabAPI
{
  class GrabThread;

  class StillImgGrabberInfo : public yat::IPlugInInfo
  {
  public: //! IPlugInInfo implementation
    std::string get_plugin_id(void) const
    { return "StillImgGrabber"; }

    virtual std::string get_interface_name(void) const 
    { return "IGrabber"; }

    virtual std::string get_version_number(void) const
    { return "2.0.0"; }
  };

  class StillImgGrabber : public IGrabber
  {
    friend class GrabThread;
  public:
    StillImgGrabber();

    ~StillImgGrabber();

    // ============================================================================
    // IPlugInObjectWithAttr implementation
    // ============================================================================
    virtual void enumerate_attributes( yat::PlugInAttrInfoList& list) const
      throw (yat::Exception);

    virtual void enumerate_properties( yat::PlugInPropInfos& prop_infos ) const
      throw (yat::Exception);
    
    virtual void set_properties( yat::PlugInPropValues& prop_values )
      throw (yat::Exception);

    // ============================================================================
    // IGrabber implementation
    // ============================================================================
    virtual void initialize( void )
      throw (yat::Exception);

    virtual void uninitialize( void )
      throw (yat::Exception);

    virtual void set_image_handler( ImageHandlerCallback callback )
      throw (yat::Exception);

    virtual GrabberState get_state( void ) const;

    virtual bool is_open( void ) const;
    
    virtual bool is_closed( void ) const;

    virtual bool is_running( void ) const;

    virtual void open( void )
      throw (yat::Exception);

    virtual void close( void )
      throw (yat::Exception);

    virtual void start( void )
      throw (yat::Exception);

    virtual void stop( void )
      throw (yat::Exception);

    virtual void snap( void )
      throw (yat::Exception);
    
    virtual void set_roi( ROI roi )
      throw (yat::Exception);

    virtual ROI  get_roi( void )
      throw (yat::Exception);
    
    virtual void reset_roi( void )
      throw (yat::Exception);

    virtual void get_settings( yat::PlugInPropValues& prop_values ) const
      throw (yat::Exception);

    virtual void set_exposure_time( const yat::Any& )
      throw (yat::Exception);

    virtual void get_exposure_time( yat::Any& )
      throw (yat::Exception);

    virtual void set_frame_rate( const yat::Any& )
      throw (yat::Exception);

    virtual void get_frame_rate( yat::Any& )
      throw (yat::Exception);

    virtual void get_bit_depth( yat::Any& )
      throw (yat::Exception);

    virtual void get_sensor_width( yat::Any& )
      throw (yat::Exception);

    virtual void get_sensor_height( yat::Any& )
      throw (yat::Exception);

  private:
    void set_filename( const yat::Any& )
      throw (yat::Exception);
    void get_filename( yat::Any& )
      throw (yat::Exception);

    ImageHandlerCallback image_callback;

    GrabThread     *grab_thread;
    std::string    filename;
    GrabAPI::Image *image;
    double         frame_rate;
    long           bit_depth;
  };


  class GrabThread : public yat::Task
  {
    friend class StillImgGrabber;
  protected:
	  //- handle_message
	  virtual void handle_message (yat::Message& msg)
      throw (yat::Exception);

  private:
    GrabThread(StillImgGrabber& grabber,
               size_t timeout_ms);
    ~GrabThread();

    StillImgGrabber& grabber_;
  };
} // namespace



#endif
