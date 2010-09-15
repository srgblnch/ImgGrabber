/*!
 * \file
 * \brief    Definition of GrabberTask class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#include <tango.h>
#include <yat4tango/ExceptionHelper.h>
#include <GrabberTask.h>

#include <iostream>
#include <sstream>

#include <isl/ErrorHandler.h>
#include <isl/Exception.h>
#include <isl/Image.h>
#include <isl/movie/MovieWriter.h>

#include <memory>

#if !defined (YAT_INLINE_IMPL)
# include "GrabberTask.i"
#endif


namespace GrabAPI
{

  const std::string STATUS_MSG_CLOSE    = "Grabber closed\n"
                                          "Call 'Open' to access underlying hardware";

  const std::string STATUS_MSG_OPEN     = "Grabber opened\n"
                                          "Call 'Start' or 'Snap' to acquire image";

  const std::string STATUS_MSG_RUNNING  = "Grabber is currently grabbing images\n"
                                          "Call 'Stop' to stop acquisition or 'Close' to release hardware access";
  
  const std::string STATUS_MSG_SAVING   = "Grabber is currently grabbing images and saving them to hard-disk\n"
                                          "Call 'StopSaveMovie' to stop saving film to disk\n"
                                          "Call 'Stop' to stop acquisition or 'Close' to release hardware access";

  const std::string STATUS_MSG_FAULT    = "Grabber encountered a fatal error\n"
                                          "Call 'Init' to re-init the device";

  const std::string STATUS_MSG_UNKNOWN  = "Grabber is in an unknown state";


  // ============================================================================
  // GrabberTask::GrabberTask
  // ============================================================================
  GrabberTask::GrabberTask(SignalType & sig)
    : Task( Config( false, // timeout msg
                    0,
                    false, // no periodic msg
                    0,
                    true,  // use the lock during msg handling
                    kDEFAULT_LO_WATER_MARK,
                    kDEFAULT_HI_WATER_MARK,
                    true,
                    0 ) ),
      grabber(0),
      last_image(0),
      movie_writer(0),
      image_available_observer_sig(sig)
  {
  }

  // ============================================================================
  // GrabberTask::~GrabberTask
  // ============================================================================
  GrabberTask::~GrabberTask()
  {
    SAFE_DELETE_PTR(this->movie_writer);
    SAFE_RELEASE(last_image);
    this->release_grabber();
  }

  // ============================================================================
  // GrabberTask::release_grabber
  // ============================================================================
  void GrabberTask::release_grabber( void )
  {
    try
    {
      //- ensure acquisition is stopped
      if (this->grabber && this->grabber->is_running())
      {
        try
        {
          this->on_stop();
        }
        catch(...) {}
      }
      //- ensure hardware access is released
      if (this->grabber && this->grabber->is_open())
      {
        try
        {
          this->on_close();
        }
        catch(...) {}
      }
      //- ensure driver is uninitialized
      if (this->grabber)
      {
        this->grabber->uninitialize();
      }
      //- release grabber object
      SAFE_DELETE_PTR( this->grabber );
    }
    catch(...) {}
  }

  // ============================================================================
  // GrabberTask::handle_message
  // ============================================================================
  void GrabberTask::handle_message(yat::Message& _msg)
    throw (yat::Exception)
  {
    switch (_msg.type())
    {
    case yat::TASK_INIT:         this->on_init(_msg);            break;
    case yat::TASK_EXIT:         this->on_exit();                break;
    case yat::TASK_TIMEOUT:                                      break;
    case kMSG_OPEN:              this->on_open();                break;
    case kMSG_CLOSE:             this->on_close();               break;
    case kMSG_START:             this->on_start();               break;
    case kMSG_STOP:              this->on_stop();                break;
    case kMSG_SNAP:              this->on_snap();                break;
    case kMSG_START_RECORDING:   this->on_start_recording(_msg); break;
    case kMSG_STOP_RECORDING:    this->on_stop_recording();      break;
    case kMSG_SET_ROI:           this->on_set_roi(_msg);         break;
    case kMSG_RESET_ROI:         this->on_reset_roi();           break;
    case kMSG_SAVE_SETTING:      this->on_save_settings();       break;
    }
  }

  //! INIT msg handler
  void GrabberTask::on_init(yat::Message& _msg)
    throw (yat::Exception)
  {
    //- extract the Grabber object from the msg
    
    std::auto_ptr<GrabberTaskInit> init_cfg;
    try
    {
      GrabberTaskInit* init_cfg_p;
      _msg.detach_data(init_cfg_p);
      init_cfg.reset(init_cfg_p);

      this->grabber = init_cfg->grabber;
      this->device  = init_cfg->device;
    }
    catch(yat::Exception& ex)
    {
      RETHROW_YAT_ERROR(ex,
                        "SOFTWARE_FAILURE",
                        "Unable to dettach data from a yat::THREAD_INIT message",
                        "GrabberTask::on_init");
    }
    catch(...)
    {
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unable to dettach data from a yat::THREAD_INIT message",
                      "GrabberTask::on_init");
    }

    //- initialize the grabber
    try
    {
      this->grabber->initialize();
      this->grabber->set_image_handler( ImageHandlerCallback::instanciate( *this, &GrabberTask::image_callback ) );
    }
    catch(yat::Exception& ex)
    {
      RETHROW_YAT_ERROR(ex,
                        "INITIALIZATION_ERROR",
                        "error during INIT",
                        "GrabberTask::on_init");
    }
    catch(...)
    {
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during INIT",
                      "GrabberTask::on_init");
    }

    try
    {
      if (init_cfg->auto_start) 
      {
        this->on_start();
      }
    }
    catch(yat::Exception& ex)
    {
      RETHROW_YAT_ERROR(ex,
                        "INITIALIZATION_ERROR",
                        "error during INIT",
                        "GrabberTask::on_init");
    }
    catch(...)
    {
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during INIT",
                      "GrabberTask::on_init");
    }
  }

  //! EXIT msg handler
  void GrabberTask::on_exit()
    throw (yat::Exception)
  {
    this->release_grabber();
  }

  //! OPEN msg handler
  void GrabberTask::on_open()
    throw (yat::Exception)
  {
    if ( this->grabber->is_open() )
    {
      //- already done : exit silently
      return;
    }

    if ( !this->grabber->is_closed() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call OPEN if the device is not in CLOSE state",
                      "GrabberTask::on_open");
    }

    //- launch the open command on the grabber
    try
    {
      this->grabber->open();
    }
    catch(yat::Exception& ex)
    {
      this->last_error_desc = "[ OPEN failed : ";
      this->last_error_desc += ex.errors[0].desc;
      this->last_error_desc += " ]";
      RETHROW_YAT_ERROR(ex,
                        "HARDWARE_ERROR",
                        "error during OPEN",
                        "GrabberTask::on_open");
    }
    catch(...)
    {
      this->last_error_desc = "[ OPEN failed : unknwon error ]";
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during OPEN",
                      "GrabberTask::on_open");
    }
  }

  //! CLOSE msg handler
  void GrabberTask::on_close()
    throw (yat::Exception)
  {
    if ( this->grabber->is_closed() )
    {
      //- already done : exit silently
      return;
    }

    if ( !this->grabber->is_open() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call CLOSE if the device is not OPEN or RUNNING",
                      "GrabberTask::on_close");
    }

    try
    {
      //- launch the open command on the grabber
      if ( this->grabber->is_running() )
      {
        //this->enable_timeout_msg(false);
        try
        {
          this->grabber->stop();
        }
        catch(...)
        {
          //- ignore
        }
      }

      this->grabber->close();
      SAFE_RELEASE( this->last_image );
    }
    catch(yat::Exception& ex)
    {
      this->last_error_desc = "[ CLOSE failed : ";
      this->last_error_desc += ex.errors[0].desc;
      this->last_error_desc += " ]";
      RETHROW_YAT_ERROR(ex,
                        "HARDWARE_ERROR",
                        "error during CLOSE",
                        "GrabberTask::on_close");
    }
    catch(...)
    {
      this->last_error_desc = "[ CLOSE failed : unknwon error ]";
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during CLOSE",
                      "GrabberTask::on_close");
    }
  }

  //! START msg handler
  void GrabberTask::on_start()
    throw (yat::Exception)
  {
    if ( this->grabber->is_running() )
    {
      //- already done : exit silently
      return;
    }

    if ( !this->grabber->is_open() && !this->grabber->is_closed() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call START if the device is not OPEN or CLOSE",
                      "GrabberTask::on_start");
    }

    try
    {
      this->fps_computer.reset();
      if (this->grabber->is_closed())
      {
        this->grabber->open();
      }

      this->grabber->start();

      //this->set_timeout_msg_period(1000);
      //this->enable_timeout_msg(true);
    }
    catch(yat::Exception& ex)
    {
      this->last_error_desc = "[ START failed : ";
      this->last_error_desc += ex.errors[0].desc;
      this->last_error_desc += " ]";
      RETHROW_YAT_ERROR(ex,
                        "HARDWARE_ERROR",
                        "error during START",
                        "GrabberTask::on_start");
    }
    catch(...)
    {
      this->last_error_desc = "[ START failed : unknwon error ]";
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during START",
                      "GrabberTask::on_start");
    }
  }

  //! STOP msg handler
  void GrabberTask::on_stop()
    throw (yat::Exception)
  {
    if ( this->grabber->get_state() == OPEN )
    {
      //- already done : exit silently
      return;
    }

    if ( !this->grabber->is_running() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call STOP if the device is not RUNNING",
                      "GrabberTask::on_stop");
    }

    try
    {
      //this->enable_timeout_msg(false);

      
      // Now we are gonna say 'stop()' to this->grabber
      // This grabber object will probably be running a thread,
      // and after stopping it we want to wait until it has
      // finished. However, that thread has a is calling 
      // image_callback(). So, if between the start of handling
      // the message, when we acquired the lock, and now a new
      // image is available, then it will be waiting for the lock.
      // To avoid this deadlock, we must release it here and reaquire
      // once it is done. But then someone could be acquiring the lock
      // to send more messages, I don't know if it is a great idea...

      /// @todo: Instead of a callback, maybe passing imatges as messages
      /// is worth it to get rid of this trick. What if several threads
      /// are calling image_callback? Probably some kind of mess? Probably
      /// not exactly a mess, because we KNOW how it works we KNOW that it's
      /// just one thread accessing here so there's no problem here.
      /// As it has better performance, I am leaving it as it is for now...
      this->m_lock.release();
      try {
        this->grabber->stop();
      } catch(...) {
        this->m_lock.lock();
        throw;
      }
      this->m_lock.lock();

      this->fps_computer.reset();
    }
    catch(yat::Exception& ex)
    {
      this->last_error_desc = "[ STOP failed : ";
      this->last_error_desc += ex.errors[0].desc;
      this->last_error_desc += " ]";
      RETHROW_YAT_ERROR(ex,
                        "HARDWARE_ERROR",
                        "error during STOP",
                        "GrabberTask::on_stop");
    }
    catch(...)
    {
      this->last_error_desc = "[ STOP failed : unknwon error ]";
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during STOP",
                      "GrabberTask::on_stop");
    }
  }

  //! SNAP msg handler
  void GrabberTask::on_snap()
    throw (yat::Exception)
  {
    if ( !this->grabber->is_open() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call SNAP if the device is not in OPEN state",
                      "GrabberTask::on_snap");
    }
 
    try
    {
      this->grabber->snap();
    }
    catch(yat::Exception& ex)
    {
      this->last_error_desc = "[ SNAP failed : ";
      this->last_error_desc += ex.errors[0].desc;
      this->last_error_desc += " ]";
      RETHROW_YAT_ERROR(ex,
                        "HARDWARE_ERROR",
                        "error during SNAP",
                        "GrabberTask::on_snap");
    }
    catch(...)
    {
      this->last_error_desc = "[ SNAP failed : unknwon error ]";
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during SNAP",
                      "GrabberTask::on_snap");
    }

  }

  //! SET_ROI msg handler
  void GrabberTask::on_set_roi(yat::Message& _msg)
    throw (yat::Exception)
  {
    if ( !this->grabber->is_open() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call SET_ROI if the device is not in OPEN state",
                      "GrabberTask::on_set_roi");
    }

    //- extract the ROI object from the msg
    ROI* new_roi = 0;
    try
    {
      _msg.detach_data(new_roi);
    }
    catch(yat::Exception& ex)
    {
      RETHROW_YAT_ERROR(ex,
                        "SOFTWARE_FAILURE",
                        "Unable to dettach data from a SET_ROI message",
                        "GrabberTask::on_set_roi");
    }
    catch(...)
    {
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unable to dettach data from a SET_ROI message",
                      "GrabberTask::on_set_roi");
    }


    try
    {
      this->grabber->set_roi( *new_roi );
    }
    catch(yat::Exception& ex)
    {
      this->last_error_desc = "[ SET_ROI failed : ";
      this->last_error_desc += ex.errors[0].desc;
      this->last_error_desc += " ]";
      RETHROW_YAT_ERROR(ex,
                        "HARDWARE_ERROR",
                        "error during SET_ROI",
                        "GrabberTask::on_set_roi");
    }
    catch(...)
    {
      this->last_error_desc = "[ SET_ROI failed : unknwon error ]";
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during SET_ROI",
                      "GrabberTask::on_set_roi");
    }
  }

  //! RESET_ROI msg handler
  void GrabberTask::on_reset_roi()
    throw (yat::Exception)
  {
    if ( !this->grabber->is_open() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call RESET_ROI if the device is not in OPEN state",
                      "GrabberTask::on_reset_roi");
    }

    try
    {
      this->grabber->reset_roi(  );
    }
    catch(yat::Exception& ex)
    {
      this->last_error_desc = "[ RESET_ROI failed : ";
      this->last_error_desc += ex.errors[0].desc;
      this->last_error_desc += " ]";
      RETHROW_YAT_ERROR(ex,
                        "HARDWARE_ERROR",
                        "error during RESET_ROI",
                        "GrabberTask::on_reset_roi");
    }
    catch(...)
    {
      this->last_error_desc = "[ RESET_ROI failed : unknown error ]";
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "error during RESET_ROI",
                      "GrabberTask::on_reset_roi");
    }
  }

  //! SAVE_SETTINGS msg handler
  void GrabberTask::on_save_settings()
    throw (yat::Exception)
  {
    if ( !this->grabber->is_open() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "You cannot call SaveSettings if the device is not in OPEN state",
                      "GrabberTask::on_save_settings");
    }

    yat::PlugInPropValues settings;
    this->grabber->get_settings( settings );

    yat::PlugInPropInfos prop_infos;
    this->grabber->enumerate_properties( prop_infos );

    yat::PlugInPropValues::iterator settings_it;
    yat::PlugInPropValues::iterator settings_end = settings.end();
    Tango::DbData data;

    for ( settings_it = settings.begin(); settings_it != settings_end; settings_it++ )
    {
      std::string name  = (*settings_it).first;
      yat::Any    value = (*settings_it).second;

      int type = prop_infos[ name ];

#define HANDLE_TYPE( TypeDesc, Type )        \
      case TypeDesc:                         \
      {                                      \
        Tango::DbDatum datum(name);          \
        datum << yat::any_cast<Type>(value); \
        data.push_back( datum );             \
        break;                               \
      }

      switch (type)
      {
        HANDLE_TYPE( yat::PlugInPropType::BOOLEAN, bool );
        HANDLE_TYPE( yat::PlugInPropType::UINT8, yat_uint8_t );
        HANDLE_TYPE( yat::PlugInPropType::INT16, yat_int16_t );
        HANDLE_TYPE( yat::PlugInPropType::UINT16, yat_uint16_t );
        HANDLE_TYPE( yat::PlugInPropType::INT32, yat_int32_t );
        HANDLE_TYPE( yat::PlugInPropType::UINT32, yat_uint32_t );
        HANDLE_TYPE( yat::PlugInPropType::FLOAT, float );
        HANDLE_TYPE( yat::PlugInPropType::DOUBLE, double );
        HANDLE_TYPE( yat::PlugInPropType::STRING, std::string );
        HANDLE_TYPE( yat::PlugInPropType::STRING_VECTOR, std::vector<std::string> );
        HANDLE_TYPE( yat::PlugInPropType::INT16_VECTOR, std::vector<yat_int16_t> );
        HANDLE_TYPE( yat::PlugInPropType::UINT16_VECTOR, std::vector<yat_uint16_t> );
        HANDLE_TYPE( yat::PlugInPropType::INT32_VECTOR, std::vector<yat_int32_t> );
        HANDLE_TYPE( yat::PlugInPropType::UINT32_VECTOR, std::vector<yat_uint32_t> );
        HANDLE_TYPE( yat::PlugInPropType::FLOAT_VECTOR, std::vector<float> );
        HANDLE_TYPE( yat::PlugInPropType::DOUBLE_VECTOR, std::vector<double> );
      }
    }

    try
    {
      this->device->get_db_device()->put_property( data );
    }
    catch( Tango::DevFailed& df )
    {
      yat4tango::TangoYATException ex( df );
      RETHROW_YAT_ERROR(ex,
                        "DATABASE_FAILURE",
                        "Unable to put property",
                        "GrabberTask::on_save_settings");
    }
  }

  //! START_RECORDING msg handler
  void GrabberTask::on_start_recording(yat::Message& _msg)
    throw (yat::Exception)
  {
    if ( this->is_saving_movie() )
    {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "A movie is already being saved. Stop the current movie first",
                      "GrabberTask::on_start_recording");
    }


    //- check if it is possible to save a video
    //- there are problem in the movie when the width of an image is odd
    
    ROI r = this->get_roi();
    if( (r.width & 1) != 0 )
    {
      THROW_YAT_ERROR("DIMENSION_ERROR",
                      "The width of the image must have an even number of pixels\n"
                      "Use the 'SetROI' command to define a ROI suitable for saving a movie",
                      "GrabberTask::on_start_recording");
    }

    //- extract the RecordMovieConfig object from the msg
    try
    {
      RecordMovieConfig* recmovie_config = 0;
      _msg.detach_data(recmovie_config);
      this->movie_config = *recmovie_config;
      SAFE_DELETE( recmovie_config );
    }
    catch(yat::Exception& ex)
    {
      RETHROW_YAT_ERROR(ex,
                        "SOFTWARE_FAILURE",
                        "Unable to detach RecordMovieConfig from a message",
                        "GrabberTask::on_start_recording");
    }
    catch(...)
    {
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unable to detach RecordMovieConfig from a message",
                      "GrabberTask::on_start_recording");
    }

    try
    {
      //- get the bit depth from the grabber object
      yat::Any container;
      this->grabber->get_bit_depth( container );
      yat_int32_t bit_depth = yat::any_cast<yat_int32_t>(container);

      isl::MovieConfig isl_conf;
      isl_conf.format = this->movie_config.format;
      isl_conf.file_basename = this->movie_config.file_basename;
      isl_conf.bit_depth = bit_depth;
      isl_conf.frame_rate = this->fps_computer.get_frame_rate();

      this->movie_writer = isl::MovieWriterFactory::create( isl_conf );

      _GET_TIME(this->movie_start);
      this->movie_end = this->movie_start;
      this->movie_end.tv_sec += static_cast<int>(this->movie_config.duration_s);

      YAT_LOG( "Ready to write images to disk" );
    }
    catch(isl::Exception& isl_ex)
    {
      ISL2YATException ex(isl_ex);
      isl::ErrorHandler::reset();
      RETHROW_YAT_ERROR(ex,
                        "ISL_ERROR",
                        "Unable to instantiate MovieWriter",
                        "GrabberTask::on_start_recording");
    }
    catch(...)
    {
      THROW_YAT_ERROR("UNKNOWN_ERROR",
                      "Unable to instantiate MovieWriter",
                      "GrabberTask::on_start_recording");
    }
  }

  //! STOP_RECORDING msg handler
  void GrabberTask::on_stop_recording()
    throw (yat::Exception)
  {
    std::cout << "Finished saving movie" << std::endl;
    SAFE_DELETE_PTR(this->movie_writer);
  }

  // ============================================================================
  // GrabberTask::get_roi
  // ============================================================================
  ROI GrabberTask::get_roi( void )
    throw (yat::Exception)
  {
    ROI r;
    try
    {
      r = this->grabber->get_roi();
    }
    catch(yat::Exception& ex)
    {
      RETHROW_YAT_ERROR(ex,
                        "SOFTWARE_FAILURE",
                        "Error when getting ROI",
                        "GrabberTask::get_roi");
    }
    catch(...)
    {
      THROW_YAT_ERROR("SOFTWARE_FAILURE",
                      "Error when getting ROI",
                      "GrabberTask::get_roi");
    }
    return r;
  }

  // ============================================================================
  // GrabberTask::image_callback
  // ============================================================================
  void GrabberTask::image_callback(Image* new_image)
  {
    SharedImage* shared_image = 0;
    {
      //- Lock the msg handling mutex so that we don't 
      //- process a msg while the image callback is called
      yat::AutoMutex<> guard(this->m_lock);

      try
      {
        if (this->movie_writer != 0)
        {
          isl::Image record_image( static_cast<int>(new_image->width()), static_cast<int>(new_image->height()), isl::ISL_STORAGE_USHORT );
          record_image.unserialize(static_cast<void*>(new_image->base()));
          
          try
          {
            this->movie_writer->write_frame(record_image);
            yat::Timestamp ts;
            _GET_TIME( ts );
            if ( _ELAPSED_SEC( this->movie_start, ts ) > this->movie_config.duration_s )
            {
              this->on_stop_recording();
            }
          }
          catch( isl::Exception& ex )
          {
            isl::ErrorHandler::reset();
            std::cout << "on_stop_recording" << std::endl;
            this->on_stop_recording();
            std::cout << "on_stop_recording ->" << std::endl;
            throw ex;
          }

        }

        SharedImage* new_shared_image = new SharedImage(new_image);
        SharedImage* old_shared_image = this->last_image;
        this->last_image = new_shared_image;
        SAFE_RELEASE( old_shared_image );

        /// @todo Should I rewrite fps_computer just as any another client of image_available_observer_sig?
        /// Think about it...
        this->fps_computer.notify_new_image();

        // Now we are done, unless somebody is interested in the image...
        if (!this->image_available_observer_sig.connected())
          return;
        shared_image = new_shared_image->duplicate();
      }
      catch(isl::Exception& isl_ex)
      {
        std::cout << isl_ex;
        ISL2YATException ex(isl_ex);
        isl::ErrorHandler::reset();
        
        /*
        RETHROW_YAT_ERROR(ex,
                          "ISL_ERROR",
                          "ISL exception caught while processing image",
                          "GrabberTask::image_callback");
        */
        return;
      }
      catch(...)
      {
        /*
        THROW_YAT_ERROR("UNKNOWN_ERROR",
                        "Unknown exception caught while processing image",
                        "GrabberTask::image_callback");
        */
        return;
      }
    }

    try {
      this->image_available_observer_sig.run(shared_image);
    } catch (...) {
      shared_image->release();
      return;
    }
    shared_image->release();
  }

  // ============================================================================
  // GrabberTask::get_last_image
  // ============================================================================
  SharedImage* GrabberTask::get_last_image( void )
  {
    yat::AutoMutex<> guard(this->m_lock);
    if (this->last_image)
      return this->last_image->duplicate();
    else
      return 0;
  }

  // ============================================================================
  // GrabberTask::get_resulting_frame_rate
  // ============================================================================
  void GrabberTask::get_resulting_frame_rate( yat::Any& container )
    throw (yat::Exception)
  {
    container = this->fps_computer.get_frame_rate();
  }

  // ============================================================================
  // GrabberTask::get_image_counter
  // ============================================================================
  void GrabberTask::get_image_counter( yat::Any& container )
    throw (yat::Exception)
  {
    container = this->fps_computer.get_image_counter();
  }

  // ============================================================================
  // GrabberTask::get_state_status
  // ============================================================================
  void GrabberTask::get_state_status(GrabberState& _state, std::string& _status)
  {
    yat::AutoMutex<> guard(this->state_status_mutex);
    _state = this->grabber ? this->grabber->get_state() : UNKNOWN;

    switch( _state )
    {
    case OPEN:
      _status = STATUS_MSG_OPEN;
      break;
    case CLOSE:
      _status = STATUS_MSG_CLOSE;
      break;
    case RUNNING:
      if (this->is_saving_movie())
        _status = STATUS_MSG_SAVING;
      else
        _status = STATUS_MSG_RUNNING;
      break;
    case FAULT:
      _status =  STATUS_MSG_FAULT;
      _status += "\n";
      _status += this->last_error_desc;
      break;
    default:
      _status =  STATUS_MSG_UNKNOWN;
      break;
    }
  }

  // ============================================================================
  // GrabberTask::set_state_status
  // ============================================================================
  void GrabberTask::set_state_status(GrabberState state)
  {
    yat::AutoMutex<> guard(this->state_status_mutex);
    if(this->grabber)
      this->grabber->set_state(state);

  }
  
  
  bool GrabberTask::is_saving_movie( void )
  {
    return this->movie_writer != 0;
  }

  std::string GrabberTask::movie_remaining_time( void )
  {
    double elapsed;
    if ( this->is_saving_movie() )
    {
      yat::Timestamp now;
      _GET_TIME( now );
      elapsed = _ELAPSED_SEC( now, this->movie_end );
    }
    else
      elapsed = 0;

    yat::OSStream oss;
    oss << elapsed << " s" << std::ends;
    return oss.str();
  }


  SharedImage::SharedImage( GrabAPI::Image* _image )
    : image(_image)
  {}

  SharedImage::~SharedImage( void )
  {
    SAFE_DELETE_PTR(image);
  }


  static const double BUFFER_MAX_DURATION = 5.0;

  FrameRateComputer::FrameRateComputer()
    : frame_rate(0),image_counter(0)
  {
    _GET_TIME(this->init_tv);
  }

  void FrameRateComputer::notify_new_image( void )
  {
    yat::Timestamp t;
    _GET_TIME(t);
    double timestamp = _ELAPSED_SEC(init_tv, t);

    rep.push_back( timestamp );

    while (rep.size() > 2 && (*rep.rbegin() - *rep.begin()) > BUFFER_MAX_DURATION)
      rep.pop_front();
    
    if (rep.size() > 1)
    {
      double mean_period = 0;
      std::deque<double>::const_iterator it1, it2;
      it1 = it2 = rep.begin();
      ++it2;
      for (; it2 != rep.end(); ++it1, ++it2)
      {
        mean_period += *it2 - *it1;
      }
      mean_period /= (rep.size() - 1);
      this->frame_rate = 1 / mean_period;
    }
	if (this->image_counter < USHRT_MAX){
		this->image_counter++;
	}else{
		this->image_counter = 0;
	}
  }

  double FrameRateComputer::get_frame_rate( void )
  {
    return this->frame_rate;
  }

  yat_uint32_t FrameRateComputer::get_image_counter( void )
  {
	  return this->image_counter;
  }

  void FrameRateComputer::reset( void )
  {
    this->frame_rate = 0;
    this->rep.clear();
    _GET_TIME(this->init_tv);
	this->image_counter = 0;
  }



  // ============================================================================
  // ISL2YATException::ISL2YATException
  // ============================================================================
  ISL2YATException::ISL2YATException(const isl::Exception& ex)
  {
    const isl::ErrorList& isl_errors = ex.errors;
    for (size_t i = 0; i < isl_errors.size(); i++)
    {
      this->push_error( isl_errors[i].reason,
                        isl_errors[i].description,
                        isl_errors[i].origin);
    }
  }


}
