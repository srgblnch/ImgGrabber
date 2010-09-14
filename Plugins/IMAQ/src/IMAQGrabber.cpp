/*!
 * \file
 * \brief    Definition of IMAQGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#include "IMAQGrabber.h"

#include <yat/plugin/PlugInSymbols.h>
#include <algorithm>
#include <iostream>
#include "IMAQManager.h"

EXPORT_SINGLECLASS_PLUGIN(GrabAPI::IMAQGrabber, GrabAPI::IMAQGrabberInfo);

namespace GrabAPI
{

# define IMAQ_CALL( Statement )                                                \
  {                                                                            \
    IMG_ERR error_code = Statement ;                                           \
    if (error_code != IMG_ERR_GOOD)                                            \
    {                                                                          \
      char error_text [256];                                                   \
      ::memset(error_text, 0, sizeof(error_text));                             \
      ::imgShowError(error_code, error_text);                                  \
      yat::OSStream origin;                                                    \
      origin << __FILE__;                                                      \
      origin << " (line ";                                                     \
      origin << __LINE__;                                                      \
      origin << " )";                                                          \
      origin << std::ends;                                                     \
      THROW_YAT_ERROR( "IMAQ_ERROR",                                           \
                       error_text,                                             \
                       origin.str() );                                         \
    }                                                                          \
  }

# ifndef MIN
#   define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#   define MAX(x,y) ( (x) < (y) ? (y) : (x) )
# endif

  // ============================================================================
  // IMAQGrabber::IMAQGrabber
  // ============================================================================
  IMAQGrabber::IMAQGrabber()
    : state(CLOSE),
      iid(0),
      sid(0),
      shutter_pulse_id(0),
      vsync_pulse_id(0),
      buflist_id(0)
  {
    ::memset(this->buffers, 0, NUM_BUFFERS * sizeof(this->buffers[0]));
  }

  // ============================================================================
  // IMAQGrabber::~IMAQGrabber
  // ============================================================================
  IMAQGrabber::~IMAQGrabber()
  {
    if (this->is_running())
    {
      try
      {
        this->stop();
      }
      catch(...) {}
    }

    if (this->is_open())
    {
      try
      {
        this->close();
      }
      catch(...) {}
    }
  }

  // ============================================================================
  // IMAQGrabber::enumerate_attributes
  // ============================================================================
  void IMAQGrabber::enumerate_attributes( yat::PlugInAttrInfoList& list ) const
    throw (yat::Exception)
  {
    yat::PlugInAttrInfo attr_info;
    attr_info.name   = "Channel";
    attr_info.label  = "Channel";
    attr_info.desc   = "Channel";
    attr_info.unit   = " ";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<IMAQGrabber&>(*this), &IMAQGrabber::get_channel );
    list.push_back(attr_info);

    attr_info.name   = "BlackLevel";
    attr_info.label  = "BlackLevel";
    attr_info.desc   = "Black Level";
    attr_info.unit   = " ";
    attr_info.display_format = "%3.2f";
    attr_info.data_type = yat::PlugInDataType::DOUBLE;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<IMAQGrabber&>(*this), &IMAQGrabber::set_black_level );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<IMAQGrabber&>(*this), &IMAQGrabber::get_black_level );
    list.push_back(attr_info);

    attr_info.name   = "WhiteLevel";
    attr_info.label  = "WhiteLevel";
    attr_info.desc   = "WhiteLevel";
    attr_info.unit   = " ";
    attr_info.display_format = "%3.2f";
    attr_info.data_type = yat::PlugInDataType::DOUBLE;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<IMAQGrabber&>(*this), &IMAQGrabber::set_white_level );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<IMAQGrabber&>(*this), &IMAQGrabber::get_white_level );
    list.push_back(attr_info);
  }

  // ============================================================================
  // IMAQGrabber::enumerate_attributes
  // ============================================================================
  void IMAQGrabber::enumerate_properties( yat::PlugInPropInfos& prop_infos ) const
    throw (yat::Exception)
  {
    prop_infos["InterfaceName"] = yat::PlugInPropType::STRING;
    prop_infos["Channel"]       = yat::PlugInPropType::INT32;
    prop_infos["Mode"]          = yat::PlugInPropType::INT32;
    prop_infos["RefLevelBlack"] = yat::PlugInPropType::DOUBLE;
    prop_infos["RefLevelWhite"] = yat::PlugInPropType::DOUBLE;
    prop_infos["ShutterDelay"]  = yat::PlugInPropType::DOUBLE;
    prop_infos["ExposureTime"]  = yat::PlugInPropType::DOUBLE;
    prop_infos["FrameRate"]     = yat::PlugInPropType::DOUBLE;
    prop_infos["ROI"]           = yat::PlugInPropType::INT32_VECTOR;
  }

  // ============================================================================
  // IMAQGrabber::set_properties
  // ============================================================================
  void IMAQGrabber::set_properties( yat::PlugInPropValues& prop_values )
    throw (yat::Exception)
  {
    if ( prop_values["InterfaceName"].empty() )
    {
      THROW_YAT_ERROR("PROPERTY_ERROR",
                      "Critical property missing or empty [InterfaceName]",
                      "IMAQGrabber::set_properties");
    }
    if ( prop_values["Channel"].empty() )
    {
      THROW_YAT_ERROR("PROPERTY_ERROR",
                      "Critical property missing or empty [Channel]",
                      "IMAQGrabber::set_properties");
    }

    this->config.interface_name = yat::any_cast<std::string>(prop_values["InterfaceName"]);


#   define REGISTER_PROP( prop_name, type, type_ref )             \
    {                                                             \
      yat::Any& prop_value = prop_values[prop_name];              \
      if (!prop_value.empty())                                    \
        type_ref = yat::any_cast<type>(prop_value);               \
    }

#   define REGISTER_MODE_PROP( prop_name, type_ref )                     \
    {                                                                    \
      yat::Any& prop_value = prop_values[prop_name];                     \
      if (!prop_value.empty())                                           \
        type_ref = static_cast<Mode>(yat::any_cast<long>(prop_value));   \
    }

    REGISTER_PROP("Channel",        long, this->config.channel);
    REGISTER_MODE_PROP("Mode",      this->config.mode);
    REGISTER_PROP("RefLevelBlack",  double, this->config.black_ref_level);
    REGISTER_PROP("RefLevelWhite",  double, this->config.white_ref_level);
    REGISTER_PROP("ShutterDelay",   double, this->config.shutter_delay_ms);
    REGISTER_PROP("ExposureTime",   double, this->config.exposure_time_ms);
    REGISTER_PROP("FrameRate",      double, this->config.frame_rate);
    std::vector<long> roi0;
    REGISTER_PROP("ROI", std::vector<long>, roi0);
    if (roi0.size() == 4)
      this->config.roi = ROI(roi0[0], roi0[1], roi0[2], roi0[3]);
  }

  // ============================================================================
  // IMAQGrabber::initialize
  // ============================================================================
  void IMAQGrabber::initialize()
    throw (yat::Exception)
  {
    IMAQManager::instance().register_grabber( this );
  }
  
  // ============================================================================
  // IMAQGrabber::uninitialize
  // ============================================================================
  void IMAQGrabber::uninitialize()
    throw (yat::Exception)
  {
    IMAQManager::instance().deregister_grabber( this );
  }

  // ============================================================================
  // IMAQGrabber::set_image_handler
  // ============================================================================
  void IMAQGrabber::set_image_handler(ImageHandlerCallback callback)
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    this->image_callback = callback;
  }

  // ============================================================================
  // IMAQGrabber::get_state
  // ============================================================================
  GrabberState IMAQGrabber::get_state( void ) const
    throw (yat::Exception)
  {
    return state;
  }

  // ============================================================================
  // IMAQGrabber::is_open
  // ============================================================================
  bool IMAQGrabber::is_open( void ) const
  {
    return state == OPEN || state == RUNNING;
  }

  // ============================================================================
  // IMAQGrabber::is_closed
  // ============================================================================
  bool IMAQGrabber::is_closed( void ) const
  {
    return state == CLOSE;
  }

  // ============================================================================
  // IMAQGrabber::is_running
  // ============================================================================
  bool IMAQGrabber::is_running( void ) const
  {
    return state == RUNNING;
  }

  // ============================================================================
  // IMAQGrabber::open
  // ============================================================================
  void IMAQGrabber::open()
    throw (yat::Exception)
  {
    if ( this->is_open() )
      return;

    try
    {
      IMAQManager::instance().enable( this );

      char* iname = const_cast<char*>( this->config.interface_name.c_str() );

      yat::AutoMutex<>(this->mutex);

      IMAQ_CALL( ::imgInterfaceOpen(iname, &this->iid) );

      IMAQ_CALL( ::imgInterfaceReset(this->iid) );

      IMAQ_CALL( ::imgSessionOpen(this->iid, &this->sid) );
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      try
      {
        this->close();
      }
      catch(...) {};

      throw;
    }
    catch(...)
    {
      this->state = FAULT;
      try
      {
        this->close();
      }
      catch(...) {};

      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unknwon exception when opening access to acquisition device",
                      "IMAQGrabber::open");
    }
    this->state = OPEN;
  }

  // ============================================================================
  // IMAQGrabber::close
  // ============================================================================
  void IMAQGrabber::close()
    throw (yat::Exception)
  {
    if ( this->is_closed() )
      return;

    try
    {
      yat::AutoMutex<>(this->mutex);

      IMAQ_CALL( ::imgClose(this->sid, TRUE) );

      IMAQ_CALL( ::imgClose(this->iid, TRUE) );

      this->state = CLOSE;
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
    catch(...)
    {
      this->state = FAULT;
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unknwon exception when closing access to acquisition device",
                      "IMAQGrabber::open");
    }
  }

  // ============================================================================
  // IMAQGrabber::start
  // ============================================================================
  void IMAQGrabber::start()
    throw (yat::Exception)
  {
    if ( this->is_running() )
      return;

    try
    {
      yat::AutoMutex<>(this->mutex);

      this->configure();

      this->setup_buffers(AcqType_CONTINUOUS);

      this->setup_pulses();

      IMAQ_CALL( ::imgSetAttribute (this->sid, 
                                    IMG_ATTR_FRAMEWAIT_MSEC, 
                                    IMG_FRAMETIME_1SECOND) );

      IMAQ_CALL( ::imgSessionWaitSignalAsync (this->sid, 
                                              IMG_BUF_COMPLETE, 
                                              IMG_TRIG_POLAR_ACTIVEH,
                                              IMAQGrabber::grab_callback,
                                              static_cast<void*>(this)) );

      this->enable_callback = true;

      IMAQ_CALL( ::imgSessionAcquire(this->sid,
                                     1,    //- indicates an asynchronous acquisition
                                     0) ); //- no callback given

      this->state = RUNNING;
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      try
      {
        this->stop();
      }
      catch(...) {};

      throw;
    }
    catch(...)
    {
      this->state = FAULT;
      try
      {
        this->stop();
      }
      catch(...) {};

      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unknwon exception when starting acquisition",
                      "IMAQGrabber::start");
    }
  }

  // ============================================================================
  // IMAQGrabber::stop
  // ============================================================================
  void IMAQGrabber::stop()
    throw (yat::Exception)
  {
    if ( !this->is_running() )
      return;

    try
    {
      yat::AutoMutex<>(this->mutex);

      this->enable_callback = false;

      uInt32 last_valid_buf_id;
      IMAQ_CALL( ::imgSessionAbort(this->sid, &last_valid_buf_id) );
      
      this->cleanup_pulses();

      this->cleanup_buffers();
      
      this->state = OPEN;
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
    catch(...)
    {
      this->state = FAULT;
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unknwon exception when stoping acquisition",
                      "IMAQGrabber::stop");
    }
  }

  // ============================================================================
  // IMAQGrabber::grab_callback
  // ============================================================================
  uInt32 IMAQGrabber::grab_callback (SESSION_ID _sid, IMG_ERR _err, uInt32, void* _user_data)
  {
    IMAQGrabber* _this = reinterpret_cast<IMAQGrabber*>(_user_data);

    yat::AutoMutex<>(_this->mutex);

    if ( !_this->enable_callback )
      return 0;
   
    bool exception_caught = false;
    try
    {
      //- get last valid frame
      unsigned long lvf = 0;
      IMAQ_CALL( ::imgGetAttribute(_this->sid, IMG_ATTR_LAST_VALID_FRAME, &lvf) );

      //- fight against a vision bug (or bad feature?): lvf may contains 
      //- 0xFFFFFFFF if no frame has been acquired yet - in this case
      //- we ask the very first frame - the timeout will do its job in case
      //- the frame takes to long to be acquired...
      if (lvf == 0xFFFFFFFF)
      {
        std::cout << "BUG ??????" << std::endl;
        lvf = 0;
      }

      size_t width  = _this->config.roi.width;
      size_t height = _this->config.roi.height;

      std::auto_ptr<Image> new_image(new Image(width, height));

      uInt32 current_buffer, buffer_addr, buffer_size;
      IMAQ_CALL( ::imgSessionGetBufferSize(_this->sid, &buffer_size) );
      IMAQ_CALL( ::imgSessionExamineBuffer(_this->sid, lvf, &current_buffer, &buffer_addr) );

      ::memcpy( new_image->base(), reinterpret_cast<void*>(buffer_addr), buffer_size );

      IMAQ_CALL( ::imgSessionReleaseBuffer(_this->sid) );

      _this->image_callback(new_image.release());
    }
    catch(...)
    {
      exception_caught = true;
    }

    return static_cast<uInt32>(!exception_caught);
  }


  // ============================================================================
  // IMAQGrabber::snap
  // ============================================================================
  void IMAQGrabber::snap()
    throw (yat::Exception)
  {
    try
    {
      yat::AutoMutex<>(this->mutex);

      this->configure();

      this->setup_buffers(AcqType_SNAP);

      this->setup_pulses();

      IMAQ_CALL( ::imgSetAttribute (this->sid, 
                                    IMG_ATTR_FRAMEWAIT_MSEC, 
                                    IMG_FRAMETIME_1SECOND) );

      IMAQ_CALL( ::imgSessionWaitSignalAsync (this->sid, 
                                              IMG_BUF_COMPLETE, 
                                              IMG_TRIG_POLAR_ACTIVEH,
                                              IMAQGrabber::grab_callback,
                                              static_cast<void*>(this)) );

      this->enable_callback = true;

      IMAQ_CALL( ::imgSessionAcquire(this->sid,
                                     0,    //- indicates an synchronous acquisition
                                     0) ); //- no callback given
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
    catch(...)
    {
      this->state = FAULT;
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unknwon exception when starting acquisition",
                      "IMAQGrabber::start");
    }
  }

  // ============================================================================
  // IMAQGrabber::set_roi
  // ============================================================================
  void IMAQGrabber::set_roi( ROI roi )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);

    bool need_restart = false;
    if ( this->is_running() )
    {
      this->stop();

      need_restart = true;
    }

    this->config.roi = roi;

    if ( need_restart )
    {
      this->start();
    }
  }

  // ============================================================================
  // IMAQGrabber::get_roi
  // ============================================================================
  ROI IMAQGrabber::get_roi()
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    return this->config.roi;
  }

  // ============================================================================
  // IMAQGrabber::reset_roi
  // ============================================================================
  void IMAQGrabber::reset_roi()
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    bool need_restart = false;
    if ( this->is_running() )
    {
      this->stop();
      this->close();

      need_restart = true;
    }

    this->config.roi = ROI();

    if ( need_restart )
    {
      this->open();
      this->start();
    }
  }

  // ============================================================================
  // IMAQGrabber::get_settings
  // ============================================================================
  void IMAQGrabber::get_settings( yat::PlugInPropValues& prop_values ) const
    throw (yat::Exception)
  {

    prop_values["Channel"]         = this->config.channel;

    prop_values["Mode"]          = static_cast<long>(this->config.mode);
    prop_values["RefLevelBlack"] = this->config.black_ref_level;
    prop_values["RefLevelWhite"] = this->config.white_ref_level;
    prop_values["ShutterDelay"]  = this->config.shutter_delay_ms;
    prop_values["ExposureTime"]  = this->config.exposure_time_ms;
    prop_values["FrameRate"]     = this->config.frame_rate;

    std::vector<long> roi_vect(4);
    roi_vect[0] = this->config.roi.x;
    roi_vect[1] = this->config.roi.y;
    roi_vect[2] = this->config.roi.width;
    roi_vect[3] = this->config.roi.height;
    prop_values["ROI"]           = roi_vect;
  }

  // ============================================================================
  // IMAQGrabber::set_exposure_time
  // ============================================================================
  void IMAQGrabber::set_exposure_time( const yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);

    if (!this->is_open())
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Device must be OPEN to set the exposure time",
                      "IMAQGrabber::set_exposure_time");
    }

    double contained = yat::any_cast<double>(container);

    if ( !is_in_range( contained, MIN_EXPOSURE_TIME_MS, MAX_EXPOSURE_TIME_MS ) )
    {
      yat::OSStream oss;
      oss << "Exposure Time must be in the range [ "
          << MIN_EXPOSURE_TIME_MS
          << " , "
          << MAX_EXPOSURE_TIME_MS
          << " ]";

      THROW_YAT_ERROR("OUT_OF_RANGE",
                      oss.str(),
                      "IMAQGrabber::set_exposure_time");
    }

    bool need_restart = false;
    if ( this->is_running() )
    {
      this->stop();
      this->close();

      need_restart = true;
    }

    this->config.exposure_time_ms = contained;

    if ( need_restart )
    {
      this->open();
      this->start();
    }
  }

  // ============================================================================
  // IMAQGrabber::get_exposure_time
  // ============================================================================
  void IMAQGrabber::get_exposure_time( yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    
    container = this->config.exposure_time_ms;
  }

  // ============================================================================
  // IMAQGrabber::set_frame_rate
  // ============================================================================
  void IMAQGrabber::set_frame_rate( const yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    
    if (!this->is_open())
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Device must be OPEN to set the frame rate",
                      "IMAQGrabber::set_frame_rate");
    }

    double contained = yat::any_cast<double>(container);
    if ( !is_in_range( contained, MIN_FRAME_RATE, MAX_FRAME_RATE ) )
    {
      yat::OSStream oss;
      oss << "Frame rate must be in the range [ "
          << MIN_FRAME_RATE
          << " , "
          << MAX_FRAME_RATE
          << " ]";

      THROW_YAT_ERROR("OUT_OF_RANGE",
                      oss.str(),
                      "IMAQGrabber::set_frame_rate");
    }

    bool need_restart = false;
    if ( this->is_running() )
    {
      this->stop();
      this->close();

      need_restart = true;
    }

    this->config.frame_rate = contained;

    if ( need_restart )
    {
      this->open();
      this->start();
    }
  }

  // ============================================================================
  // IMAQGrabber::get_frame_rate
  // ============================================================================
  void IMAQGrabber::get_frame_rate( yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    
    container = this->config.frame_rate;
  }


  // ============================================================================
  // IMAQGrabber::get_bit_depth
  // ============================================================================
  void IMAQGrabber::get_bit_depth( yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Device must be OPEN to get the bit depth",
                      "IMAQGrabber::get_bit_depth");
    }

    uInt32 value;
    IMAQ_CALL( ::imgGetAttribute(this->iid,
                                 IMG_ATTR_PIXDEPTH,
                                 &value) );

    container = static_cast<long>(value);
  }

  // ============================================================================
  // IMAQGrabber::get_sensor_width
  // ============================================================================
  void IMAQGrabber::get_sensor_width( yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Device must be OPEN to get the sensor width",
                      "IMAQGrabber::get_sensor_width");
    }
    uInt32 value;
    IMAQ_CALL( ::imgGetAttribute(this->sid,
                                 IMG_ATTR_ACQWINDOW_WIDTH,
                                 &value) );

    container = static_cast<long>(value);
  }

  // ============================================================================
  // IMAQGrabber::get_sensor_height
  // ============================================================================
  void IMAQGrabber::get_sensor_height( yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Device must be OPEN to get the sensor height",
                      "IMAQGrabber::get_sensor_height");
    }

    uInt32 pix_depth;
    IMAQ_CALL( ::imgGetAttribute(this->sid,
                                 IMG_ATTR_ACQWINDOW_HEIGHT,
                                 &pix_depth) );

    container = static_cast<long>(pix_depth);
  }

  // ============================================================================
  // IMAQGrabber::get_channel
  // ============================================================================
  void IMAQGrabber::get_channel(  yat::Any& container )
    throw (yat::Exception)
  {
    container = this->config.channel;
  }

  // ============================================================================
  // IMAQGrabber::set_blacklevel
  // ============================================================================
  void IMAQGrabber::set_black_level( const yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    
    if (!this->is_open())
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Device must be OPEN to set the black level",
                      "IMAQGrabber::set_black_level");
    }
    
    double contained = yat::any_cast<double>(container);
    double max_black_ref_level = MIN(MAX_BLACK_REF_LEVEL, this->config.white_ref_level);

    if ( !is_in_range( contained, 
                       MIN_BLACK_REF_LEVEL,
                       max_black_ref_level) )
    {
      yat::OSStream oss;
      oss << "Black Ref Level must be in the range [ "
          << MIN_BLACK_REF_LEVEL
          << " , "
          << max_black_ref_level
          << " ]";

      THROW_YAT_ERROR("OUT_OF_RANGE",
                      oss.str(),
                      "IMAQGrabber::set_black_level");
    }

    bool need_restart = false;
    if ( this->is_running() )
    {
      this->stop();
      this->close();

      need_restart = true;
    }

    this->config.black_ref_level = contained;

    if ( need_restart )
    {
      this->open();
      this->start();
    }
  }

  // ============================================================================
  // IMAQGrabber::get_black_level
  // ============================================================================
  void IMAQGrabber::get_black_level( yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    container = this->config.black_ref_level;
  }

  // ============================================================================
  // IMAQGrabber::set_white_level
  // ============================================================================
  void IMAQGrabber::set_white_level( const yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    
    if (!this->is_open())
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Device must be OPEN to set the white level",
                      "IMAQGrabber::set_white_level");
    }

    double contained = yat::any_cast<double>(container);
    
    double min_white_ref_level = MAX(this->config.black_ref_level, MIN_WHITE_REF_LEVEL);
    
    if ( !is_in_range( contained, 
                       min_white_ref_level,
                       MAX_WHITE_REF_LEVEL) )
    {
      yat::OSStream oss;
      oss << "Black Ref Level must be in the range [ "
          << min_white_ref_level
          << " , "
          << MAX_WHITE_REF_LEVEL
          << " ]";

      THROW_YAT_ERROR("OUT_OF_RANGE",
                      oss.str(),
                      "IMAQGrabber::set_white_level");
    }

    bool need_restart = false;
    if ( this->is_running() )
    {
      this->stop();
      this->close();

      need_restart = true;
    }

    this->config.white_ref_level = contained;

    if ( need_restart )
    {
      this->open();
      this->start();
    }
  }

  // ============================================================================
  // IMAQGrabber::get_white_level
  // ============================================================================
  void IMAQGrabber::get_white_level( yat::Any& container )
    throw (yat::Exception)
  {
    yat::AutoMutex<>(this->mutex);
    container = this->config.white_ref_level;
  }

  // ============================================================================
  // IMAQGrabber::configure
  // ============================================================================
  void IMAQGrabber::configure()
    throw (yat::Exception)
  {
    IMAQ_CALL( ::imgSessionTriggerClear(this->sid) );

    IMAQ_CALL( ::imgSetAttribute(this->sid, IMG_ATTR_CHANNEL, this->config.channel) );

    IMAQ_CALL( ::imgSetAttribute(this->sid,
                                 IMG_ATTR_BLACK_REF_VOLT,
                                 (uInt32)(&this->config.black_ref_level)) );

    IMAQ_CALL( ::imgSetAttribute(this->sid,
                                 IMG_ATTR_WHITE_REF_VOLT,
                                 (uInt32)(&this->config.white_ref_level)) );


    if ( this->config.roi.is_empty() )
    {
      //-  set ROI to full image
      uInt32 acq_win_width, acq_win_height;
      IMAQ_CALL( ::imgGetAttribute(this->sid,
                                   IMG_ATTR_ACQWINDOW_WIDTH,
                                   &acq_win_width) );

      IMAQ_CALL( ::imgGetAttribute(this->sid,
                                   IMG_ATTR_ACQWINDOW_HEIGHT,
                                   &acq_win_height) );

      this->config.roi.x = 0;
      this->config.roi.y = 0;
      this->config.roi.width = acq_win_width;
      this->config.roi.height = acq_win_height;
    }

    IMAQ_CALL( ::imgSessionSetROI(this->sid,
                                  this->config.roi.y,
                                  this->config.roi.x,
                                  this->config.roi.height,
                                  this->config.roi.width) );

    //- if ever it is not what we requested
    //- (note : we are sure values are unsigned so reinterpret_cast is safe)
    IMAQ_CALL( ::imgSessionGetROI(this->sid,
                                  reinterpret_cast<uInt32*>(&this->config.roi.y),
                                  reinterpret_cast<uInt32*>(&this->config.roi.x),
                                  reinterpret_cast<uInt32*>(&this->config.roi.height),
                                  reinterpret_cast<uInt32*>(&this->config.roi.width)) );
  }

  // ============================================================================
  // IMAQGrabber::setup_buffers
  // ============================================================================
  void IMAQGrabber::setup_buffers( AcqType type )
    throw (yat::Exception)
  {
    size_t num_buffers = (type == AcqType_SNAP ? 1 : NUM_BUFFERS);

    //- create the buffer list
    IMAQ_CALL( ::imgCreateBufList(NUM_BUFFERS, &this->buflist_id) );

    //- get the buffer sizes
    uInt32 roi_height, row_bytes;
    IMAQ_CALL( ::imgGetAttribute(this->sid, IMG_ATTR_ROI_HEIGHT, &roi_height) );
    IMAQ_CALL( ::imgGetAttribute(this->sid, IMG_ATTR_ROWBYTES, &row_bytes) );

    uInt32 buffer_size; // = roi_height * row_bytes;
    IMAQ_CALL( ::imgSessionGetBufferSize(this->sid, &buffer_size) );

    //- create ring buffers
    for (uInt32 i = 0; i < num_buffers; i++)
    {
      IMAQ_CALL( ::imgCreateBuffer(this->sid, IMG_HOST_FRAME,
                                   buffer_size, &(this->buffers[i])) );

      IMAQ_CALL( ::imgSetBufferElement(this->buflist_id,
                                       i,
                                       IMG_BUFF_ADDRESS,
                                       reinterpret_cast<uInt32>(this->buffers[i])) );

      IMAQ_CALL( ::imgSetBufferElement(this->buflist_id,
                                       i,
                                       IMG_BUFF_SIZE,
                                       buffer_size ) );

      uInt32 buff_command;
      if (type == AcqType_SNAP)
      {
        buff_command = IMG_CMD_STOP;
      }
      else
      {
        buff_command = ( i == (NUM_BUFFERS - 1) ? IMG_CMD_LOOP : IMG_CMD_NEXT );
      }

      IMAQ_CALL( ::imgSetBufferElement(this->buflist_id,
                                       i,
                                       IMG_BUFF_COMMAND,
                                       buff_command) );
    }

    IMAQ_CALL( ::imgMemLock(this->buflist_id) );

    IMAQ_CALL( ::imgSessionConfigure(this->sid, this->buflist_id) );
  }

  // ============================================================================
  // IMAQGrabber::cleanup_buffers
  // ============================================================================
  void IMAQGrabber::cleanup_buffers()
    throw (yat::Exception)
  {
    IMAQ_CALL( ::imgMemUnlock(this->buflist_id) );

    for (uInt32 i = 0; i < NUM_BUFFERS; i++)
    {
      if (this->buffers[i] != 0)
      {
        IMAQ_CALL( ::imgDisposeBuffer(this->buffers[i]) );
        this->buffers[i] = 0;
      }
    }

    if (this->buflist_id)
    {
      IMAQ_CALL( ::imgDisposeBufList(this->buflist_id, 0) );
      this->buflist_id = 0;
    }
  }

  // ============================================================================
  // IMAQGrabber::setup_pulses
  // ============================================================================
  void IMAQGrabber::setup_pulses()
    throw (yat::Exception)
  {
    PulseConfig shutter_pulse, vsync_pulse;
    uInt32 timebase;

    switch (this->config.mode)
    {
    case MODE_CONTINUOUS:
      //- nothing to do : exit the function
      return;
    case MODE_HSYNC_TRIGGER:
      {
        IMAQ_CALL( ::imgSessionTriggerDrive( this->sid,
                                              IMG_EXT_TRIG1,
                                              IMG_TRIG_POLAR_ACTIVEH,
                                              IMG_TRIG_DRIVE_HSYNC ) );

        shutter_pulse.input_line      = IMG_EXT_TRIG1;
        shutter_pulse.input_polarity  = IMG_TRIG_POLAR_ACTIVEH;
        shutter_pulse.output_line     = SHUTTER_LINEOUT;
        shutter_pulse.output_polarity = SHUTTER_POLARITY;
        shutter_pulse.mode            = PULSE_MODE_SINGLE_REARM;

        //- specific for the XC56 camera : ExposureTime = TriggerWidth + 8µs
        double width_s = 1E-3 * this->config.exposure_time_ms - 8E-6;
        double delay_s = 1 / this->config.frame_rate - width_s;
        delay_s = MAX(delay_s, 1E-3 * MIN_DEAD_TIME_MS);
        IMAQ_CALL( ::imgPulseRate( delay_s, 
                                   width_s, 
                                   &shutter_pulse.delay, 
                                   &shutter_pulse.width,
                                   &timebase ) );

        vsync_pulse.input_line     = shutter_pulse.output_line;
        vsync_pulse.input_polarity = shutter_pulse.output_polarity == IMG_PULSE_POLAR_ACTIVEL
                                      ? IMG_PULSE_POLAR_ACTIVEH
                                      : IMG_PULSE_POLAR_ACTIVEL;
        vsync_pulse.output_line     = VSYNC_LINEOUT;
        vsync_pulse.output_polarity = VSYNC_POLARITY;
        vsync_pulse.mode = PULSE_MODE_SINGLE_REARM;
        IMAQ_CALL( ::imgPulseRate( 1E-3 * VSYNC_DELAY_MS, 
                                   1E-3 * VSYNC_WIDTH_MS, 
                                   &vsync_pulse.delay, 
                                   &vsync_pulse.width,
                                   &timebase ) );
      }
      break;
   case MODE_EXTERNAL_TRIGGER:
      {
        shutter_pulse.input_line      = EXTERNAL_TRIGGER_LINEIN;
        shutter_pulse.input_polarity  = EXTERNAL_TRIGGER_POLARITY;
        shutter_pulse.output_line     = SHUTTER_LINEOUT;
        shutter_pulse.output_polarity = SHUTTER_POLARITY;
        shutter_pulse.mode            = PULSE_MODE_SINGLE_REARM;

        vsync_pulse.input_line      = shutter_pulse.output_line;
        vsync_pulse.input_polarity  = shutter_pulse.output_polarity;
        vsync_pulse.output_line     = VSYNC_LINEOUT;
        vsync_pulse.output_polarity = VSYNC_POLARITY;
        vsync_pulse.mode            = PULSE_MODE_SINGLE_REARM;

#ifdef USE_OLD_CODE

# define kUS_PER_CLOCK_TICK 0.02
# define TO_CLK_TICK(p) static_cast<Int32>( (p)  / kUS_PER_CLOCK_TICK )
# define kDEFAULT_VIDEO_TRANSFER_DELAY 10000
# define kDEFAULT_VSYNC_WIDTH 10000

        shutter_pulse.delay = TO_CLK_TICK(1.E3 * this->config.shutter_delay_ms);

        shutter_pulse.width = TO_CLK_TICK(1.E3 * this->config.exposure_time_ms); 

        vsync_pulse.delay = TO_CLK_TICK(1.E3 * (this->config.shutter_delay_ms + this->config.exposure_time_ms))
                          + kDEFAULT_VIDEO_TRANSFER_DELAY;

        vsync_pulse.width = kDEFAULT_VSYNC_WIDTH;
  
        timebase = PULSE_TIMEBASE_50MHZ;

# undef kUS_PER_CLOCK_TICK
# undef TO_CLK_TICK

#else
        IMAQ_CALL( ::imgPulseRate( 1E-3 * this->config.shutter_delay_ms, 
                                   1E-3 * this->config.exposure_time_ms - 8E-6, 
                                   &shutter_pulse.delay, 
                                   &shutter_pulse.width,
                                   &timebase ) );

        IMAQ_CALL( ::imgPulseRate( 1E-3 * (this->config.shutter_delay_ms + this->config.exposure_time_ms + 0.2), 
                                   1E-3 * VSYNC_WIDTH_MS, 
                                   &vsync_pulse.delay, 
                                   &vsync_pulse.width,
                                   &timebase ) );
#endif
      }
      break;

    case MODE_SOFTWARE_TRIGGER:
      {
        shutter_pulse.input_line = IMG_IMMEDIATE;
        shutter_pulse.input_polarity = 0; //- not used in IMG_IMMEDIATE case
        shutter_pulse.output_line = SHUTTER_LINEOUT;
        shutter_pulse.output_polarity = SHUTTER_POLARITY;
        shutter_pulse.mode = PULSE_MODE_TRAIN;

        //- specific for the XC56 camera : ExposureTime = TriggerWidth + 8µs
        double width_s = 1E-3 * this->config.exposure_time_ms - 8E-6;
        double delay_s = 1 / this->config.frame_rate - width_s;
        delay_s = MAX(delay_s, 1E-3 * MIN_DEAD_TIME_MS);
        IMAQ_CALL( ::imgPulseRate( delay_s, 
                                   width_s, 
                                   &shutter_pulse.delay, 
                                   &shutter_pulse.width,
                                   &timebase ) );

        vsync_pulse.input_line = shutter_pulse.output_line;
        vsync_pulse.input_polarity = shutter_pulse.output_polarity == IMG_PULSE_POLAR_ACTIVEL
                                      ? IMG_PULSE_POLAR_ACTIVEH
                                      : IMG_PULSE_POLAR_ACTIVEL;
        vsync_pulse.output_line = VSYNC_LINEOUT;
        vsync_pulse.output_polarity = VSYNC_POLARITY;
        vsync_pulse.mode = PULSE_MODE_SINGLE_REARM;
        IMAQ_CALL( ::imgPulseRate( 1E-3 * VSYNC_DELAY_MS, 
                                   1E-3 * VSYNC_WIDTH_MS, 
                                   &vsync_pulse.delay, 
                                   &vsync_pulse.width,
                                   &timebase ) );

      }
      break;
    default:
      break;
    }

    IMAQ_CALL( ::imgPulseCreate(timebase, //- PULSE_TIMEBASE_50MHZ,
                                shutter_pulse.delay, shutter_pulse.width,
                                shutter_pulse.input_line, shutter_pulse.input_polarity,
                                shutter_pulse.output_line, shutter_pulse.output_polarity,
                                shutter_pulse.mode,
                                &this->shutter_pulse_id) );
 
    IMAQ_CALL( ::imgPulseCreate(timebase, //- PULSE_TIMEBASE_50MHZ,
                                vsync_pulse.delay, vsync_pulse.width,
                                vsync_pulse.input_line, vsync_pulse.input_polarity,
                                vsync_pulse.output_line, vsync_pulse.output_polarity,
                                vsync_pulse.mode,
                                &this->vsync_pulse_id) );

    IMAQ_CALL( ::imgPulseStart(this->shutter_pulse_id,this->sid) );
    IMAQ_CALL( ::imgPulseStart(this->vsync_pulse_id,  this->sid) );
  }

  // ============================================================================
  // IMAQGrabber::cleanup_buffers
  // ============================================================================
  void IMAQGrabber::cleanup_pulses()
    throw (yat::Exception)
  {
    if (this->config.mode == MODE_CONTINUOUS)
      return;

    IMAQ_CALL( ::imgPulseStop(this->shutter_pulse_id) );
    IMAQ_CALL( ::imgPulseStop(this->vsync_pulse_id)   );

    IMAQ_CALL( ::imgPulseDispose(this->shutter_pulse_id) );
    this->shutter_pulse_id = 0;
    IMAQ_CALL( ::imgPulseDispose(this->vsync_pulse_id) );
    this->vsync_pulse_id   = 0;

    IMAQ_CALL( ::imgSessionTriggerClear(this->sid) );
  }

}
