/*!
 * \file
 * \brief    Declaration of BaslerGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */
#ifndef _BASLERGRABBER_H_
#define _BASLERGRABBER_H_

#include <pylon/PylonIncludes.h>
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

  const double             DEFAULT_EXPOSURE_TIME                = 100.0;
  const double             DEFAULT_FRAME_RATE                   = 5.0;
  const double             DEFAULT_GAIN                         = 50;//%
  const yat_uint16_t       DEFAULT_BLACKLEVEL                   = 32;
  const Mode               DEFAULT_MODE                         = MODE_INTERNAL_TRIGGER;
  const TriggerSourceEnums DEFAULT_TRIGGER_LINE                 = TriggerSource_Line1;
  const TriggerActivation  DEFAULT_TRIGGER_ACTIVATION           = TriggerActivation_RisingEdge;
  const yat_uint16_t       DEFAULT_FRAME_TRANSMISSION_DELAY     = 0;
  const yat_uint16_t       DEFAULT_BINNING                      = 1;
  const yat_uint16_t       DEFAULT_ETHERNET_PAYLOAD_PACKET_SIZE = 1500;
  const bool               DEFAULT_ENABLE_RESEND                = false;
  const yat_uint16_t       DEFAULT_PACKET_TIMEOUT               = 40;
  const yat_uint16_t       DEFAULT_FRAME_RETENTION              = 200;
  const yat_uint16_t       DEFAULT_RECEIVE_WINDOW_SIZE          = 16;
  const yat_uint16_t       DEFAULT_ACQUISITION_FRAME_COUNT      = 1;
  const PixelFormatEnums   DEFAULT_PIXEL_FORMAT                 = PixelFormat_Mono16;

  struct BaslerGrabberInitCfg
  {
    BaslerGrabberInitCfg()
      : exposure_time(DEFAULT_EXPOSURE_TIME),
        frame_rate(DEFAULT_FRAME_RATE),
        gain(DEFAULT_GAIN),
        blacklevel(DEFAULT_BLACKLEVEL),
        acq_mode(DEFAULT_MODE),
        trigger_line(DEFAULT_TRIGGER_LINE),
        trigger_activation(DEFAULT_TRIGGER_ACTIVATION),
        frame_transmission_delay(DEFAULT_FRAME_TRANSMISSION_DELAY),
        binning_vertical(DEFAULT_BINNING),
        binning_horizontal(DEFAULT_BINNING),
        ethernet_payload_packet_size(DEFAULT_ETHERNET_PAYLOAD_PACKET_SIZE),
        ethernet_enable_resend(DEFAULT_ENABLE_RESEND),
        ethernet_packet_timeout(DEFAULT_PACKET_TIMEOUT),
        ethernet_frame_retention(DEFAULT_FRAME_RETENTION),
        ethernet_receive_window_size(DEFAULT_RECEIVE_WINDOW_SIZE),
        acquisition_frame_count(DEFAULT_ACQUISITION_FRAME_COUNT),
        pixel_format(DEFAULT_PIXEL_FORMAT)
    {};

    double exposure_time;
    double frame_rate;
    double gain;
    yat_uint16_t blacklevel;
    Mode acq_mode;
    TriggerSourceEnums trigger_line;
    TriggerActivation trigger_activation;
    ROI roi;
    yat_uint16_t frame_transmission_delay;
    yat_uint16_t binning_vertical;
    yat_uint16_t binning_horizontal;
    yat_uint16_t ethernet_payload_packet_size;
    bool ethernet_enable_resend;
    yat_uint16_t ethernet_packet_timeout;
    yat_uint16_t ethernet_frame_retention;
    yat_uint16_t ethernet_receive_window_size;
    yat_uint16_t acquisition_frame_count;
    PixelFormatEnums pixel_format;
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
    { return "2.2.0"; }
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
    void set_state( GrabberState );

    virtual bool is_open( void ) const;

    virtual bool is_closed( void ) const;

    virtual bool is_running( void ) const;

    virtual bool is_camera_present( void ) const;

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

    virtual void reset_camera( void )
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

    virtual void register_camera_removal_cb ( void );
    virtual void removal_callback_method( Pylon::IPylonDevice* );

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

    void set_frame_transmission_delay( const yat::Any& )
      throw (yat::Exception);
    void get_frame_transmission_delay( yat::Any& )
      throw (yat::Exception);

    void set_packet_transmission_delay( const yat::Any& )
      throw (yat::Exception);
    void get_packet_transmission_delay( yat::Any& )
      throw (yat::Exception);

    void set_binning_vertical( const yat::Any& )
      throw (yat::Exception);
    void get_binning_vertical( yat::Any& )
      throw (yat::Exception);

    void set_binning_horizontal( const yat::Any& )
      throw (yat::Exception);
    void get_binning_horizontal( yat::Any& )
      throw (yat::Exception);

    void set_ethernet_payload_packet_size( const yat::Any& )
      throw (yat::Exception);
    void get_ethernet_payload_packet_size( yat::Any& )
      throw (yat::Exception);

    void get_ethernet_payload_image_size( yat::Any& )
      throw (yat::Exception);

    void set_ethernet_enable_resend( const yat::Any& )
      throw (yat::Exception);
    void get_ethernet_enable_resend( yat::Any& )
      throw (yat::Exception);

    void set_ethernet_packet_timeout( const yat::Any& )
      throw (yat::Exception);
    void get_ethernet_packet_timeout( yat::Any& )
      throw (yat::Exception);

    void set_ethernet_frame_retention( const yat::Any& )
      throw (yat::Exception);
    void get_ethernet_frame_retention( yat::Any& )
      throw (yat::Exception);

    void set_ethernet_receive_window_size( const yat::Any& )
      throw (yat::Exception);
    void get_ethernet_receive_window_size( yat::Any& )
      throw (yat::Exception);

    void get_ethernet_bandwidth_use( yat::Any& )
      throw (yat::Exception);

    void get_firmware_version( yat::Any& )
      throw (yat::Exception);
    void get_device_version( yat::Any& )
      throw (yat::Exception);
    void get_device_model( yat::Any& )
      throw (yat::Exception);
    void get_serial_number( yat::Any& )
      throw (yat::Exception);

    void set_acquisition_frame_count( const yat::Any& )
      throw (yat::Exception);
    void get_acquisition_frame_count( yat::Any& )
      throw (yat::Exception);

    void get_sensor_board_temperature( yat::Any& )
      throw (yat::Exception);

    void set_pixel_format( const yat::Any& )
      throw (yat::Exception);
    void get_pixel_format( yat::Any& )
      throw (yat::Exception);

    void get_last_timestamp (yat::Any& )
      throw (yat::Exception);

    void do_set_exposure_time( double exposure_time_ms );
    virtual void do_set_roi( ROI roi );
    
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
    bool camera_present;

    //- ExposureTime limits (in microseconds)
    double min_exp_time;
    double max_exp_time;

    bool disable_callback;

    // is it a Pilot or a Scout ?
    // Scout model have names beginning with "sc",
    // Pilot have names beginning with "pi",
    // Ace have names beginning with "ac"
    GenICam::gcstring model_name;

    typedef enum DeviceModelFamily
    {
      DEVICE_MODEL_SCOUT,
      DEVICE_MODEL_PILOT,
      DEVICE_MODEL_ACE
    };

    DeviceModelFamily model_family;


    double frame_rate;
    yat_uint64_t last_timestamp;

    yat_uint32_t overruns;
    yat_int32_t last_framecounter;

    yat_uint32_t acquisition_buffer_nb;
    yat_uint16_t acquisition_frame_count;

    std::string pixel_format;

  };

} // namespace



#endif
