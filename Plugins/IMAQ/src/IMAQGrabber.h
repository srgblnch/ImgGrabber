/*!
 * \file
 * \brief    Declaration of IMAQGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */
#ifndef _IMAQGRABBER_H_
#define _IMAQGRABBER_H_

#include <IGrabber.h>

#include <IMAQConfig.h>

namespace GrabAPI
{


  class IMAQGrabberInfo : public yat::IPlugInInfo
  {
  public: //! IPlugInInfo implementation
    std::string get_plugin_id(void) const
    { return "IMAQGrabber"; }

    virtual std::string get_interface_name(void) const 
    { return "IGrabber"; }

    virtual std::string get_version_number(void) const
    { return "2.0.0"; }
  };

  class IMAQGrabber : public IGrabber
  {
  public:
    IMAQGrabber();

    ~IMAQGrabber();

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

    virtual void set_state( GrabberState mystate) ;

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
    typedef enum
    {
      AcqType_SNAP,
      AcqType_CONTINUOUS
    } AcqType;

    //- write attributes to camera
    void configure()
      throw (yat::Exception);

    //- allocate and queue the buffers
    void setup_buffers( AcqType )
      throw (yat::Exception);
    
    //- de-queue, unregister and deallocate buffers
    void cleanup_buffers()
      throw (yat::Exception);

    //- configure and start the pulses
    void setup_pulses()
      throw (yat::Exception);

    //- stop and release the pulses
    void cleanup_pulses()
      throw (yat::Exception);

    void get_channel(  yat::Any& )
      throw (yat::Exception);

    void set_black_level( const yat::Any& )
      throw (yat::Exception);
    void get_black_level(  yat::Any& )
      throw (yat::Exception);

    void set_white_level( const yat::Any& )
      throw (yat::Exception);
    void get_white_level( yat::Any& )
      throw (yat::Exception);

    static uInt32 grab_callback (SESSION_ID, IMG_ERR, uInt32, void*);

  private:
    ImageHandlerCallback image_callback;
    
    //! Mutex because commands and attributes are used in 2 different threads
    yat::Mutex mutex;

    //! the current configuration
    ChannelConfig config;

    //! the current state
    GrabberState state;

    //! underlying interface ID (NI-specific)
    INTERFACE_ID iid;

    //! session ID (NI-specific)
    SESSION_ID sid;

    //! shutter trigger signal id (NI-specific)
    PULSE_ID shutter_pulse_id;

    //! vsync signal id (NI-specific)
    PULSE_ID vsync_pulse_id;

    //! buffers list ID (NI-specific)
    BUFLIST_ID buflist_id;
    
    //! buffers
    void* buffers[NUM_BUFFERS];

    //! if false, exit callback immediately if when is called
    bool enable_callback;

  };

} // namespace



#endif
