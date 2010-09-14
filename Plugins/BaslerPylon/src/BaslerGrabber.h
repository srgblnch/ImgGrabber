/*!
 * \file
 * \brief    Declaration of BaslerGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */
#ifndef _BASLERGRABBER_H_
#define _BASLERGRABBER_H_

#include <pylon/TlFactory.h>
#include <pylon/gige/BaslerGigECamera.h>

#include <boost/smart_ptr.hpp>
#include <IGrabber.h>

namespace GrabAPI
{
  typedef Pylon::CBaslerGigECamera Camera_t;
  using namespace Basler_GigECameraParams;
  using namespace Basler_GigEStreamParams;
  using namespace Basler_GigETLParams;

  using boost::shared_ptr;
  using boost::scoped_ptr;
  using boost::scoped_array;

  class GrabThread;

  typedef enum Mode
  {
    MODE_INTERNAL_TRIGGER = 0,
    MODE_EXTERNAL_TRIGGER_TIMED,
    MODE_EXTERNAL_TRIGGER_PULSE_WIDTH,
  };

  typedef TriggerActivationEnums TriggerActivation;

  const double DEFAULT_EXPOSURE_TIME = 100.0;
  const double DEFAULT_FRAME_RATE    = 5.0;
  const long   DEFAULT_GAIN          = 320;
  const long   DEFAULT_BLACKLEVEL    = 32;
  const Mode   DEFAULT_MODE          = MODE_INTERNAL_TRIGGER;
  const TriggerSourceEnums DEFAULT_TRIGGER_LINE  = TriggerSource_Line1;
  const TriggerActivation DEFAULT_TRIGGER_ACTIVATION = TriggerActivation_RisingEdge;

  struct BaslerGrabberInitCfg
  {
    BaslerGrabberInitCfg()
      : exposure_time(DEFAULT_EXPOSURE_TIME),
        frame_rate(DEFAULT_FRAME_RATE),
        gain(DEFAULT_GAIN),
        blacklevel(DEFAULT_BLACKLEVEL),
        acq_mode(DEFAULT_MODE),
        trigger_line(DEFAULT_TRIGGER_LINE),
        trigger_activation(DEFAULT_TRIGGER_ACTIVATION)
    {};

    double exposure_time;
    double frame_rate;
    long   gain;
    long   blacklevel;
    Mode   acq_mode;
    TriggerSourceEnums trigger_line;
    TriggerActivation trigger_activation;
    ROI    roi;
  };

  class TransportLayer;
  class Camera;
  class StreamGrabber;
  class ChunkParser;
  class RegisteredBufferList;
  class QueuedBufferList;
  class StreamGrabberRessourceLock;
  class AcqTask;

  class BaslerGrabberInfo : public yat::IPlugInInfo
  {
  public: //! IPlugInInfo implementation
    std::string get_plugin_id(void) const
    { return "BaslerGrabber"; }

    virtual std::string get_interface_name(void) const 
    { return "IGrabber"; }

    virtual std::string get_version_number(void) const
    { return "2.1.0"; }
  };

  class BaslerGrabber : public IGrabber
  {
    friend class AcqTask;
  public:
    BaslerGrabber();

    ~BaslerGrabber();

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
    //- adjust a value to fit a multiple of increment
    static int adjust (int value, const int increment);

    //- write init attributes to camera
    void setup_initialization_attributes()
      throw (yat::Exception);

    void compute_min_max_exposure()
      throw (yat::Exception);

    void set_gain( const yat::Any& )
      throw (yat::Exception);
    void get_gain( yat::Any& )
      throw (yat::Exception);
   
    void set_black_level( const yat::Any& )
      throw (yat::Exception);
    void get_black_level( yat::Any& )
      throw (yat::Exception);

    void set_trigger_mode( const yat::Any& )
      throw (yat::Exception);
    void get_trigger_mode( yat::Any& )
      throw (yat::Exception);

    void set_trigger_activation( const yat::Any& )
      throw (yat::Exception);
    void get_trigger_activation( yat::Any& )
      throw (yat::Exception);

    void set_trigger_line( const yat::Any& )
      throw (yat::Exception);
    void get_trigger_line( yat::Any& )
      throw (yat::Exception);

    void get_basler_frame_rate( yat::Any& )
      throw (yat::Exception);
    
    void get_overruns( yat::Any& )
      throw (yat::Exception);
    
    void set_internal_acquisition_buffers( const yat::Any& )
      throw (yat::Exception);
    void get_internal_acquisition_buffers( yat::Any& )
      throw (yat::Exception);
    
    void set_averaging( const yat::Any& )
      throw (yat::Exception);
    void get_averaging( yat::Any& )
      throw (yat::Exception);


    void do_set_exposure_time( double exposure_time_ms );
    void do_set_roi(ROI roi);
    
    long _raw_get_bit_depth() const;

  private:
    ImageHandlerCallback image_callback;

    shared_ptr<TransportLayer>             transport_layer;
    shared_ptr<Camera>                     camera;
    shared_ptr<StreamGrabber>              stream_grabber;
    shared_ptr<ChunkParser>                chunk_parser;
    shared_ptr<RegisteredBufferList>       reg_buflist;
    shared_ptr<QueuedBufferList>           queued_buflist;
    shared_ptr<StreamGrabberRessourceLock> ressourcelock;
    shared_ptr<AcqTask>                    acq_task;

    //- Properties
    std::string camera_ip;
    BaslerGrabberInitCfg init_cfg;

    //- Mutex because commands and attributes are used in 2 different threads
    yat::Mutex mutex;

    //- the current state
    GrabberState state;

    //- ExposureTime limits (in microseconds)
    double min_exp_time;
    double max_exp_time;

    bool disable_callback;

    // is it a Pilot or a Scout ?
    // Scout model have names beginning with "sc", Pilot have names beginning with "pi"
    GenICam::gcstring model_name;

    typedef enum DeviceModelFamily
    {
      DEVICE_MODEL_SCOUT,
      DEVICE_MODEL_PILOT
    };

    DeviceModelFamily model_family;


    double frame_rate;
    int64_t last_timestamp;

    long overruns;
    long last_framecounter;

    long acquisition_buffer_nb;

  };

} // namespace



#endif
