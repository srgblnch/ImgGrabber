/*!
 * \file
 * \brief    Definition of IMAQGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#include "BaslerGrabber.h"

#include <yat/plugin/PlugInSymbols.h>
#include <yat/Singleton.h>
#include <yat/threading/Task.h>
#include <yat/network/Address.h>
#include <cmath>

EXPORT_SINGLECLASS_PLUGIN(GrabAPI::BaslerGrabber, GrabAPI::BaslerGrabberInfo);

#define GRABLOG( p ) std::cout << p  << std::endl


namespace GrabAPI
{
  // Number of buffers used for grabbing
  const yat_uint32_t kNUM_BUFFERS = 5;

  void throw_genicam_exception( const GenICam::GenericException &e, const char* description, const char* location, const char* filename, int line_number )
    throw (yat::Exception)
  {
    yat::Exception ex;

    std::stringstream genicam_origin;
    genicam_origin << e.GetSourceFileName()
                   << "["
                   << e.GetSourceLine()
                   << "]";
    ex.push_error( "GENICAM_ERROR", e.GetDescription(), genicam_origin.str().c_str() );

    std::stringstream baslergrabber_origin;
    baslergrabber_origin << location
                         << " ("
                         << filename
                         << "["
                         << line_number
                         << "])";
    ex.push_error( "BASLER_ERROR", description, baslergrabber_origin.str().c_str() );

    throw ex;
  }

  void throw_yat_exception( const char* reason, const char* description, const char* location, const char* filename, int line_number )
  {
    std::stringstream origin;
    origin << location
           << " ("
           << filename
           << "["
           << line_number
           << "])";

    throw yat::Exception( reason, description, origin.str().c_str() );
  }

#define THROW_GENICAM( genicam_exception, description ) throw_genicam_exception( (genicam_exception), (description), (__FUNCTIONNAME__), __FILE__, __LINE__ )

#define THROW_YAT( reason, description )        throw_yat_exception( (reason), (description), (__FUNCTIONNAME__), __FILE__, __LINE__ )

#define FUNCTION_NAME( function_name ) static const char* __FUNCTIONNAME__ = function_name

  class PylonGuard : public yat::Singleton<PylonGuard>
  {
    public:
      PylonGuard();
      ~PylonGuard();

      void init();

    private:
      bool initialized_;
  };



  PylonGuard::PylonGuard()
    : initialized_(false)
  {
  }

  PylonGuard::~PylonGuard()
  {
    GRABLOG( "Pylon::PylonTerminate" );
    Pylon::PylonTerminate( );
  }

  void PylonGuard::init()
  {
    if ( !initialized_ )
    {
      GRABLOG( "Pylon::PylonInitialize" );
      Pylon::PylonInitialize( );
      initialized_ = true;
    }
  }


  // RAII for Transport Layer
  class TransportLayer
  {
  public:
    TransportLayer()
      : tl_(0)
    {
      GRABLOG( "Pylon::CTlFactory::GetInstance().CreateTl( Camera_t::DeviceClass() )" );
      tl_ = Pylon::CTlFactory::GetInstance().CreateTl( Camera_t::DeviceClass() );
    }

    ~TransportLayer()
    {
      if (tl_)
      {
        GRABLOG( "Pylon::CTlFactory::GetInstance().ReleaseTl" );
        Pylon::CTlFactory::GetInstance().ReleaseTl(tl_);      
      }
    }

    Pylon::ITransportLayer& get()
    {
      return *tl_;
    }

  private:
    Pylon::ITransportLayer* tl_;
  };

  // RAII for camera
  class Camera
  {
  public:
    Camera(shared_ptr<TransportLayer> tl, const std::string& camera_ip)
    {
      FUNCTION_NAME( "Camera::Camera" );

      // Get all attached cameras and exit if no camera is found
      Pylon::DeviceInfoList_t devices;
      const size_t max_retry = 5;
      size_t retry_count = max_retry;
      int nb_camera_found = 0;
      
      while ( 0 < retry_count-- )
      { 
        GRABLOG( "TransportLayer::EnumerateDevices" );
        nb_camera_found = tl->get().EnumerateDevices( devices );
        if ( 0 != nb_camera_found )
          break;
        else
          yat::ThreadingUtilities::sleep( 0, 100 * 1000 * 1000 ); // wait 100 msec
      }

      if ( 0 == nb_camera_found )
      {
        yat::OSStream oss;
        oss << "Unable to find any camera on the network after "
            << max_retry
            << " tries.\n"
            << "Please check connection to camera "
            << camera_ip;

        THROW_YAT("BASLER_DRIVER_ERROR", oss.str().c_str());
      }

      // Find the device with an IP corresponding to the one given in property
      Pylon::DeviceInfoList_t::const_iterator it;

      // camera_ip is not really necessarily an IP, it may also be a DNS name
      // pylon_camera_ip IS an IP
      Pylon::String_t pylon_camera_ip(
                    yat::Address(camera_ip, 0).get_ip_address().c_str());

      for (it = devices.begin(); it != devices.end(); it++)
      {
        const Camera_t::DeviceInfo_t& gige_device_info = static_cast<const Camera_t::DeviceInfo_t&>(*it);
        Pylon::String_t current_ip = gige_device_info.GetIpAddress();
        if (current_ip == pylon_camera_ip)
          break;
      }

      if (it == devices.end())
      {
        yat::OSStream oss;
        oss << "Camera "
            << camera_ip
            << " not found";
        THROW_YAT("BASLER_DRIVER_ERROR", oss.str().c_str());
      }

      
      // Create the camera object of the first available camera
      // The camera object is used to set and get all available
      // camera features.
      GRABLOG( "TransportLayer::CreateDevice" );
      scoped_ptr<Camera_t> camera( new Camera_t(tl->get().CreateDevice( *it )) );

      if( !camera->GetDevice() )
      {
        THROW_YAT("BASLER_CAM_NOT_AVAILABLE", "Unable to get the camera from transport_layer.");
      }


      GRABLOG( "Camera::Open" );
      camera->Open();

      // camera object created successfully (no exception) : 
      // keep a reference on the TL since it needs to be destroyed after the camera
      tl_ = tl;
      camera_.swap( camera );
    }

    ~Camera()
    {
      GRABLOG( "Camera::Close" );
      camera_->Close();

      // release camera_, THEN tl_ 
      camera_.reset();
      tl_.reset();
    }

    Camera_t& get()
    {
      return *camera_;
    }

  private:
    scoped_ptr<Camera_t> camera_;
    shared_ptr<TransportLayer> tl_; // TL must be deleted AFTER the camera object
  };

  class StreamGrabber
  {
  public:
    StreamGrabber( shared_ptr<Camera> camera )
    {
      // Get the first stream grabber object of the selected camera
      scoped_ptr<Camera_t::StreamGrabber_t> stream_grabber;
      GRABLOG( "new Camera_t::StreamGrabber_t( camera.GetStreamGrabber(0) )" );
      stream_grabber.reset( new Camera_t::StreamGrabber_t(camera->get().GetStreamGrabber( 0 )) );

      GRABLOG( "StreamGrabber::Open" );
      stream_grabber->Open();

      // stream grabber opened successfully : keep the reference active to properly close it
      stream_grabber_.swap(stream_grabber);
      // stream grabber depends on camera so it must be active during the lifetime of the stream grabber
      camera_ = camera;
    }

    ~StreamGrabber()
    {
      GRABLOG( "StreamGrabber::Close" );
      stream_grabber_->Close();

      // release stream_grabber_, THEN camera_ 
      stream_grabber_.reset();
      camera_.reset();
    }

    Camera_t::StreamGrabber_t& get()
    {
      return *stream_grabber_;
    }

  private:
    scoped_ptr<Camera_t::StreamGrabber_t> stream_grabber_;
    shared_ptr<Camera> camera_;
  };


  class ChunkParser
  {
  public:
    ChunkParser( shared_ptr<Camera> camera )
    {
      GRABLOG( "Camera::CreateChunkParser" );
      chunk_parser_ = camera->get().CreateChunkParser();
      camera_ = camera;
   }

    ~ChunkParser()
    {
      GRABLOG( "Camera::DestroyChunkParser" );
      camera_->get().DestroyChunkParser( chunk_parser_ );
      camera_.reset();
    }

    Pylon::IChunkParser& get()
    {
      return *chunk_parser_;
    }

  private:
    Pylon::IChunkParser* chunk_parser_;
    shared_ptr<Camera> camera_;
  };


  class RegisteredBuffer
  {
  public:
    RegisteredBuffer( shared_ptr<StreamGrabber> stream_grabber,
                      size_t image_size )
      : stream_grabber_(stream_grabber)
    {
      GRABLOG( "new Buffer"  );
      data_.reset( new yat_uint8_t[image_size] );
      GRABLOG( "StreamGrabber::RegisterBuffer "  << std::hex << reinterpret_cast<uintptr_t>(data_.get()) );
      handle_ = stream_grabber_->get().RegisterBuffer( data_.get(), image_size );
      GRABLOG( "Successfully registered buffer. handle = "  << std::hex << handle_ );
    }
    
    ~RegisteredBuffer()
    {
      GRABLOG( "DeregisterBuffer : " << std::hex << handle_  );
      stream_grabber_->get().DeregisterBuffer( handle_ );
      
      // data must be released after deregistering buffer
      GRABLOG( "delete " << std::hex << reinterpret_cast<uintptr_t>(data_.get())  );
      data_.reset();

      // stream grabber must be released after everything
      stream_grabber_.reset();
    }

    Pylon::StreamBufferHandle handle() const
    {
      return handle_;
    }


  private:
    scoped_array<yat_uint8_t> data_;
    Pylon::StreamBufferHandle handle_;
    shared_ptr<StreamGrabber> stream_grabber_;
  };


  class RegisteredBufferList
  {
    friend class QueuedBufferList;
  public:
    typedef shared_ptr<RegisteredBuffer> RegBufferP;

    RegisteredBufferList( shared_ptr<Camera> camera,
                          shared_ptr<StreamGrabber> stream_grabber,
                          shared_ptr<StreamGrabberRessourceLock> ressourcelock,
                          long acquisition_buffer_nb )
    {
      const size_t payload = static_cast<size_t>( camera->get().PayloadSize.GetValue() );
      buf_list_.reserve( acquisition_buffer_nb );
      for (long i = 0; i < acquisition_buffer_nb; ++i)
      {
        RegBufferP reg_buf( new RegisteredBuffer(stream_grabber, payload) );
        buf_list_.push_back( reg_buf );
      }
    }

    /*
    // std::vector destructor will do the job
    // each RegBufferP will be deleted, so each handle will be DeRegistered
    ~RegisteredBufferList()
    {
    }
    */

  private:
    std::vector< RegBufferP > buf_list_;
    shared_ptr<StreamGrabberRessourceLock> ressourcelock_;
  };


  // when all buffer are properly registered, they can be queued
  class QueuedBufferList
  {
  public:
    typedef shared_ptr<RegisteredBufferList> RegBufferListP;

    QueuedBufferList( RegBufferListP reg_buffer_list,
                      shared_ptr<StreamGrabber> stream_grabber )
      : reg_buffer_list_(reg_buffer_list),
        stream_grabber_( stream_grabber )
    {
      // queue them
      for (size_t i = 0; i < reg_buffer_list->buf_list_.size() ; ++i)
      {
        RegisteredBuffer& reg_buffer = *(reg_buffer_list->buf_list_[i]);
        GRABLOG( "StreamGrabber::QueueBuffer with handle " << std::hex << reg_buffer.handle()  );
        stream_grabber_->get().QueueBuffer( reg_buffer.handle() );
      }
    }

    ~QueuedBufferList()
    {
      //- put all pending buffers to the stream_grabber's output queue
      GRABLOG( "StreamGrabber::CancelGrab"  );
      stream_grabber_->get().CancelGrab();

      //- retrieve them to flush the output queue
      GRABLOG( "StreamGrabber::flush queue"  );
      Pylon::GrabResult r;
      while( stream_grabber_->get().RetrieveResult( r ) )
      {
        // empty loop : it's normal
        //
        // RetrieveResult returns false
        // as soon as there is no buffer in the queue
      }

      // unregister all buffers
      GRABLOG( "delete RegisteredBufferList"  );
      reg_buffer_list_.reset();
    }

  private:
    RegBufferListP reg_buffer_list_;
    shared_ptr<StreamGrabber> stream_grabber_;
  };


  // buffers must remain queued as long as the grab is active
  class StreamGrabberRessourceLock
  {
  public:
    StreamGrabberRessourceLock( shared_ptr<StreamGrabber> stream_grabber )
      : stream_grabber_( stream_grabber )
    {
      GRABLOG( "StreamGrabber::PrepareGrab"  );
      stream_grabber_->get().PrepareGrab();
    }

    ~StreamGrabberRessourceLock()
    {
      GRABLOG( "StreamGrabber::FinishGrab"  );
      stream_grabber_->get().FinishGrab();
    }

  private:
    shared_ptr<StreamGrabber> stream_grabber_;
  };

  class AcqTask : public yat::Task
  {
  public:
    AcqTask( BaslerGrabber& grabber_object, long nb_image )
      : grabber_( grabber_object ),
        nb_image_(nb_image),
        size_x_(0),
        size_y_(0),
        first_image_received_( false )
    {
#ifndef YAT_WIN32
      termination_event_ = Pylon::WaitObjectEx::Create();
#endif
      wait_objects_.Add( grabber_.stream_grabber->get().GetWaitObject() );  // getting informed about buffers
      wait_objects_.Add( termination_event_ ); // getting informed about termination request

      this->go();
    }

    ~AcqTask()
    {
    }

    void start()
    {
      this->wait_msg_handled( yat::Message::allocate(MSG_ID_START, DEFAULT_MSG_PRIORITY, true ) );
    }

    void stop()
    {
      // signal the termination event to cancel the blocking call where we wait on a new image
      GRABLOG( "termination_event_.Signal()" );
      termination_event_.Signal();

      this->wait_msg_handled( yat::Message::allocate(MSG_ID_STOP, DEFAULT_MSG_PRIORITY, true ) );
    }

  protected:

    //- handle_message
    virtual void handle_message (yat::Message& msg)
      throw (yat::Exception)
    {
      FUNCTION_NAME( "AcqTask::handle_message" );
      switch( msg.type() )
      {
      case MSG_ID_START :
        {
          try
          {
            current_nb_image_ = 0;
            size_x_ = static_cast<size_t>(grabber_.camera->get().Width.GetValue());
            size_y_ = static_cast<size_t>(grabber_.camera->get().Height.GetValue());
            first_image_received_ = false;

            grabber_.camera->get().AcquisitionStart.Execute();

            this->post( yat::Message::allocate(MSG_ID_GET_IMAGE) ); 

            grabber_.state = RUNNING;
          }
          catch( GenICam::GenericException & e )
          {
            // transfer the exception to the caller (this message is 'waited')
            THROW_GENICAM( e, "Error when starting acquisition" );

          }
        }
        break;
      case MSG_ID_GET_IMAGE :
        {
          try
          {
            unsigned int index;  // index of the wait object that has been signalled
            Pylon::GrabResult result;   // grab result

            // block until an image arrive
            const unsigned int kWAIT_TIMEOUT = 11000;
            if ( !wait_objects_.WaitForAny( kWAIT_TIMEOUT, &index ) )
            {
              // timeout error
              GRABLOG( "Timeout occurred" );
              grabber_.camera->get().AcquisitionStop.Execute();
              grabber_.state = FAULT;
              return;
            }

            // index == 0 -> the stream grabber signaled the wait object
            // index == 1 means that AcqTask::stop has been called
            if ( index == 0 && grabber_.stream_grabber->get().RetrieveResult( result ) )
            {
              switch( result.Status() )
              {
              case Pylon::Idle:
                //GRABLOG( "Pylon::Idle result received" );
                break;
              case Pylon::Queued:
                //GRABLOG( "Pylon::Queued result received" );
                break;
              case Pylon::Grabbed:
                {
                  if ( grabber_.disable_callback == false )
                  {
                    //- make a deep copy and call the callback
                    /// @todo Delete new_image pointer on error...
                    current_nb_image_++;
                    size_t bit_depth = grabber_._raw_get_bit_depth();
                    Image* new_image = new Image(size_x_,
                                                 size_y_,
                                                 static_cast<unsigned short*>(result.Buffer()));
                    
                    new_image->bit_depth = bit_depth;

                    if ( !first_image_received_ )
                    {
                      grabber_.chunk_parser->get().AttachBuffer( result.Buffer(), result.GetPayloadSize() );
                      first_image_received_ = true;
                    }
                    else
                    {
                      grabber_.chunk_parser->get().UpdateBuffer( result.Buffer() );
                    }

                    int64_t framecounter = grabber_.camera->get().ChunkFramecounter.GetValue();
                    if ( framecounter != 0 && grabber_.last_framecounter != framecounter - 1 )
                      grabber_.overruns++;

                    grabber_.last_framecounter = static_cast<yat_int32_t>(framecounter);
                    int64_t timestamp = grabber_.camera->get().ChunkTimestamp.GetValue();
                    // each clock tick is 8 ns for a Pilot camera
                    grabber_.frame_rate = 125000000. / (double(timestamp) - double(grabber_.last_timestamp));
                    grabber_.last_timestamp = timestamp;

                    grabber_.image_callback(new_image);
                  }
                }
              case Pylon::Canceled:
                //GRABLOG( "Pylon::Canceled result received" );
                break;
              case Pylon::Failed:
                //GRABLOG( "Pylon::Failed result received" );
                break;
              default:
                //GRABLOG( "Unknown Pylon result received" );
                break;
              }

              // reuse the buffer for grabbing 
              grabber_.stream_grabber->get().QueueBuffer(result.Handle(), result.Context() );

              if ( current_nb_image_ == nb_image_ )
                this->post( yat::Message::allocate(MSG_ID_STOP) ); 
              else
                this->post( yat::Message::allocate(MSG_ID_GET_IMAGE) ); 
            }

            
          }
          catch( GenICam::GenericException & e )
          {
            try
            {
              GRABLOG( "Camera.AcquisitionStop.Execute()" );
              grabber_.camera->get().AcquisitionStop.Execute();
            }
            catch( ... ) {}

            grabber_.state = FAULT;

            THROW_GENICAM( e, "Error during acquisition" );
          }
        }
        break;
      case MSG_ID_STOP :
        {

          try
          {
            GRABLOG( "Camera.AcquisitionStop.Execute()" );
            grabber_.camera->get().AcquisitionStop.Execute();
            grabber_.state = OPEN;
          }
          catch( GenICam::GenericException & e )
          {
            // transfer the exception to the caller (this message is 'waited')
            THROW_GENICAM( e, "Error when stoping acquisition" );

          }
        }
        break;
      }
    }


  private:
    enum 
    {
      MSG_ID_START = yat::FIRST_USER_MSG,
      MSG_ID_GET_IMAGE,
      MSG_ID_STOP,
    };

    BaslerGrabber& grabber_;
    
#ifdef YAT_WIN32
    Pylon::AlertableWaitObject termination_event_;
#else
    Pylon::WaitObjectEx  termination_event_;
#endif
    Pylon::WaitObjects wait_objects_;

    size_t nb_image_; // nb of images to acquire (0 = continuous mode)
    size_t current_nb_image_; // the current number of images acquired
    size_t size_x_, size_y_;
    bool first_image_received_;
  };


  struct TaskExiter
  {
    void operator()( yat::Task* task )
    {
      try
      {
        task->exit();
      }
      catch(...) {}
    }
  };


  // ============================================================================
  // BaslerGrabber::BaslerGrabber
  // ============================================================================
  BaslerGrabber::BaslerGrabber()
    : state(CLOSE),
      disable_callback(true),
      frame_rate(0),
      last_timestamp(0),
      overruns(0),
      last_framecounter(0),
      acquisition_buffer_nb(kNUM_BUFFERS)
  {
  }

  // ============================================================================
  // BaslerGrabber::~BaslerGrabber
  // ============================================================================
  BaslerGrabber::~BaslerGrabber()
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
  // BaslerGrabber::enumerate_attributes
  // ============================================================================
  void BaslerGrabber::enumerate_attributes( yat::PlugInAttrInfoList& list ) const
    throw (yat::Exception)
  {
    yat::PlugInAttrInfo attr_info;

    attr_info.name   = "BlackLevel";
    attr_info.label  = "BlackLevel";
    attr_info.desc   = "Black Level";
    attr_info.unit   = " ";
    attr_info.display_format = "%3d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_black_level );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_black_level );
    list.push_back(attr_info);

    attr_info.name   = "Gain";
    attr_info.label  = "Gain";
    attr_info.desc   = "Gain";
    attr_info.unit   = " ";
    attr_info.display_format = "%4d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_gain );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_gain );
    list.push_back(attr_info);

    attr_info.name   = "TriggerMode";
    attr_info.label  = "Trigger Mode";
    attr_info.desc   = "Trigger Mode (0 : internal, 1 : external - timed, 2 : external - pulse width)";
    attr_info.unit   = " ";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_trigger_mode );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_trigger_mode );
    list.push_back(attr_info);

    attr_info.name   = "TriggerLine";
    attr_info.label  = "Trigger Line";
    attr_info.desc   = "Trigger Line (0 : Line1, 1 : Line2)";
    attr_info.unit   = " ";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_trigger_line );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_trigger_line );
    list.push_back(attr_info);

    attr_info.name   = "TriggerActivation";
    attr_info.label  = "Trigger Activation";
    attr_info.desc   = "Trigger Activation (0 : rising edge, 1 : falling edge)";
    attr_info.unit   = " ";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_trigger_activation );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_trigger_activation );
    list.push_back(attr_info);

    attr_info.name   = "InternalFrameRate";
    attr_info.label  = "InternalFrameRate";
    attr_info.desc   = "";
    attr_info.unit   = "fps";
    attr_info.display_format = "%3.1f";
    attr_info.data_type = yat::PlugInDataType::DOUBLE;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_basler_frame_rate );
    list.push_back(attr_info);

    attr_info.name   = "Overruns";
    attr_info.label  = "Overruns";
    attr_info.desc   = "number of overruns since beginning of acquisition";
    attr_info.unit   = " ";
    attr_info.display_format = "%6d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_overruns );
    list.push_back(attr_info);

    attr_info.name   = "InternalAcquisitionBuffers";
    attr_info.label  = "Internal Acquisition Buffers";
    attr_info.desc   = "";
    attr_info.unit   = " ";
    attr_info.display_format = "%6d";
    attr_info.data_type = yat::PlugInDataType::INT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_internal_acquisition_buffers );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_internal_acquisition_buffers );
    list.push_back(attr_info);
/*
Averaging is supported only for Pilot camera. Should I do it myself to make it available for all ?
    attr_info.name   = "Averaging";
    attr_info.label  = "Averaging";
    attr_info.desc   = "";
    attr_info.unit   = " ";
    attr_info.display_format = "%6d";
    attr_info.data_type = yat::PlugInDataType::INT2;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_averaging );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_averaging );
    list.push_back(attr_info);
*/
  }

  // ============================================================================
  // BaslerGrabber::enumerate_attributes
  // ============================================================================
  void BaslerGrabber::enumerate_properties( yat::PlugInPropInfos& prop_infos ) const
    throw (yat::Exception)
  {
    prop_infos["CameraIP"]          = yat::PlugInPropType::STRING;
    prop_infos["ExposureTime"]      = yat::PlugInPropType::DOUBLE;
    prop_infos["FrameRate"]         = yat::PlugInPropType::DOUBLE;
    prop_infos["Gain"]              = yat::PlugInPropType::INT32;
    prop_infos["BlackLevel"]        = yat::PlugInPropType::INT32;
    prop_infos["TriggerMode"]       = yat::PlugInPropType::INT32;
    prop_infos["TriggerLine"]       = yat::PlugInPropType::INT32;
    prop_infos["TriggerActivation"] = yat::PlugInPropType::INT32;
    prop_infos["ROI"]               = yat::PlugInPropType::INT32_VECTOR;
  }

  // ============================================================================
  // BaslerGrabber::set_properties
  // ============================================================================
  void BaslerGrabber::set_properties( yat::PlugInPropValues& prop_values )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_properties" );

    if (prop_values["CameraIP"].empty())
    {
      THROW_YAT("PROPERTY_ERROR", "Critical property missing or empty [CameraIP]");
    }
    this->camera_ip = yat::any_cast<std::string>(prop_values["CameraIP"]);


#   define REGISTER_PROP( prop_name, type, type_ref )             \
    {                                                             \
      yat::Any& prop_value = prop_values[prop_name];              \
      if (!prop_value.empty())                                    \
        type_ref = yat::any_cast<type>(prop_value);               \
    }

#   define REGISTER_ENUM( prop_name, type, type_ref )                    \
    {                                                                    \
      yat::Any& prop_value = prop_values[prop_name];                     \
      if (!prop_value.empty())                                           \
        type_ref = static_cast<type>(yat::any_cast<yat_int32_t>(prop_value));   \
    }

    REGISTER_PROP("ExposureTime",       double,             init_cfg.exposure_time);
    REGISTER_PROP("FrameRate",          double,             init_cfg.frame_rate);
    REGISTER_PROP("Gain",               yat_int32_t,               init_cfg.gain);
    REGISTER_PROP("BlackLevel",         yat_int32_t,               init_cfg.blacklevel);
    REGISTER_ENUM("TriggerMode",        Mode,               init_cfg.acq_mode);
    REGISTER_ENUM("TriggerLine",        TriggerSourceEnums, init_cfg.trigger_line);
    REGISTER_ENUM("TriggerActivation",  TriggerActivation,  init_cfg.trigger_activation);

    std::vector<yat_int32_t> roi;
    REGISTER_PROP("ROI", std::vector<yat_int32_t>, roi);
    if ( roi.size() == 4 )
      init_cfg.roi = ROI(roi[0], roi[1], roi[2], roi[3]);

    if ( init_cfg.acq_mode != MODE_INTERNAL_TRIGGER
         && init_cfg.acq_mode != MODE_EXTERNAL_TRIGGER_TIMED
         && init_cfg.acq_mode != MODE_EXTERNAL_TRIGGER_PULSE_WIDTH )
    {
      THROW_YAT("BASLER_DRIVER_ERROR", "'TriggerMode' property must be 0 (INTERNAL), 1 (EXTERNAL, TIMED) or 2 (EXTERNAL, PULSE WIDTH)");
    }

    if (init_cfg.trigger_line != TriggerSource_Line1 && init_cfg.trigger_line != TriggerSource_Line2)
    {
      THROW_YAT("BASLER_DRIVER_ERROR", "'TriggerLine' property must be 0 or 1");
    }

    if (init_cfg.trigger_activation != TriggerActivation_RisingEdge
        && init_cfg.trigger_activation != TriggerActivation_FallingEdge)
    {
      THROW_YAT("BASLER_DRIVER_ERROR", "'TriggerActivation' property must be 0 (rising edge) or 1 (falling edge)");
    }
  }

  // ============================================================================
  // BaslerGrabber::initialize
  // ============================================================================
  void BaslerGrabber::initialize()
    throw (yat::Exception)
  {
    PylonGuard::instance().init();
  }
  
  // ============================================================================
  // BaslerGrabber::uninitialize
  // ============================================================================
  void BaslerGrabber::uninitialize()
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // BaslerGrabber::set_image_handler
  // ============================================================================
  void BaslerGrabber::set_image_handler(ImageHandlerCallback callback)
    throw (yat::Exception)
  {
    yat::MutexLock guard(this->mutex);
    this->image_callback = callback;
  }

  // ============================================================================
  // BaslerGrabber::get_state
  // ============================================================================
  GrabberState BaslerGrabber::get_state( void ) const
  {
    return state;
  }

  // ============================================================================
  // BaslerGrabber::is_open
  // ============================================================================
  bool BaslerGrabber::is_open( void ) const
  {
    return state == OPEN || state == RUNNING;
  }

  // ============================================================================
  // BaslerGrabber::is_closed
  // ============================================================================
  bool BaslerGrabber::is_closed( void ) const
  {
    return state == CLOSE;
  }

  // ============================================================================
  // BaslerGrabber::is_running
  // ============================================================================
  bool BaslerGrabber::is_running( void ) const
  {
    return state == RUNNING;
  }

  // ============================================================================
  // BaslerGrabber::open
  // ============================================================================
  void BaslerGrabber::open()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::open" );
  
    yat::MutexLock guard(this->mutex);
    try
    {
      transport_layer.reset( new TransportLayer() );
      camera.reset( new Camera(transport_layer, this->camera_ip) );
      stream_grabber.reset( new StreamGrabber(camera) );
      chunk_parser.reset( new ChunkParser(camera) );

      this->setup_initialization_attributes();

      this->state = OPEN;
    }
    catch( GenICam::GenericException &e )
    {
      this->state = FAULT;
      THROW_GENICAM( e, "Error when opening hardware access" );
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
  }

  // ============================================================================
  // BaslerGrabber::close
  // ============================================================================
  void BaslerGrabber::close()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::close" );
    yat::MutexLock guard(this->mutex);
    try
    {
      chunk_parser.reset();
      stream_grabber.reset();
      camera.reset();
      transport_layer.reset();
      this->state = CLOSE;
    }
    catch( GenICam::GenericException &e )
    {
      this->state = FAULT;
      THROW_GENICAM( e, "Error when closing hardware access" );
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
  }

  // ============================================================================
  // BaslerGrabber::start
  // ============================================================================
  void BaslerGrabber::start()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::start" );
    yat::MutexLock guard(this->mutex);
    try
    {
      if (this->acq_task)
      {
        disable_callback = true;
        this->acq_task->stop();
        this->acq_task.reset();

        // release everything. order don't really matter since each object
        // already keeps a reference to the objects they depend on
        this->reg_buflist.reset();
        this->queued_buflist.reset();
        this->ressourcelock.reset();
      }

      this->frame_rate = 0;
      this->last_timestamp = 0;
      this->overruns = 0;
      this->last_framecounter = 0;

      this->camera->get().AcquisitionMode.SetValue( AcquisitionMode_Continuous );

      // We won't use image buffers greater than PayLoadSize of the maximal image size
      this->stream_grabber->get().MaxBufferSize.SetValue( this->camera->get().PayloadSize.GetValue() );
      // We won't queue more than 'acquisition_buffer_nb' image buffer at a time
      this->stream_grabber->get().MaxNumBuffer.SetValue( this->acquisition_buffer_nb );

      this->ressourcelock.reset( new StreamGrabberRessourceLock(this->stream_grabber) );
      this->reg_buflist.reset( new RegisteredBufferList(this->camera, this->stream_grabber, this->ressourcelock, acquisition_buffer_nb) );
      this->queued_buflist.reset( new QueuedBufferList(this->reg_buflist, this->stream_grabber) );

      this->acq_task.reset( new AcqTask(*this, 0), TaskExiter() );
      disable_callback = false;
      this->acq_task->start();
    }
    catch( GenICam::GenericException &e )
    {
      this->state = FAULT;
      THROW_GENICAM( e, "Error when starting acqusisition" );
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
  }

  // ============================================================================
  // BaslerGrabber::stop
  // ============================================================================
  void BaslerGrabber::stop()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::stop" );
    yat::MutexLock guard(this->mutex);
    try
    {
      disable_callback = true;
      this->acq_task->stop();
      this->acq_task.reset();
      this->ressourcelock.reset();
      this->queued_buflist.reset();
      this->reg_buflist.reset();
    }
    catch( GenICam::GenericException &e )
    {
      this->state = FAULT;
      THROW_GENICAM( e, "Error when stoping acqusisition" );
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
  }

  // ============================================================================
  // BaslerGrabber::snap
  // ============================================================================
  void BaslerGrabber::snap()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::snap" );
    yat::MutexLock guard(this->mutex);
    try
    {
      if (this->acq_task)
      {
        disable_callback = true;
        this->acq_task->stop();
        this->acq_task.reset();

        // release everything. order don't really matter since each object
        // already keeps a reference to the objects they depend on
        this->reg_buflist.reset();
        this->queued_buflist.reset();
        this->ressourcelock.reset();
      }

      this->frame_rate = 0;
      this->last_timestamp = 0;
      this->overruns = 0;
      this->last_framecounter = 0;

      this->camera->get().AcquisitionMode.SetValue( AcquisitionMode_Continuous );

      // We won't use image buffers greater than PayLoadSize of the maximal image size
      this->stream_grabber->get().MaxBufferSize.SetValue( this->camera->get().PayloadSize.GetValue() );
      // We won't queue more than 'acquisition_buffer_nb' image buffer at a time
      this->stream_grabber->get().MaxNumBuffer.SetValue( this->acquisition_buffer_nb );

      this->ressourcelock.reset( new StreamGrabberRessourceLock(this->stream_grabber) );
      this->reg_buflist.reset( new RegisteredBufferList(this->camera, this->stream_grabber, this->ressourcelock, acquisition_buffer_nb) );
      this->queued_buflist.reset( new QueuedBufferList(this->reg_buflist, this->stream_grabber) );

      this->acq_task.reset( new AcqTask(*this, 1), TaskExiter() );
      disable_callback = false;
      this->acq_task->start();
    }
    catch( GenICam::GenericException &e )
    {
      this->state = FAULT;
      THROW_GENICAM( e, "Error when starting acqusisition" );
    }
    catch( yat::Exception & )
    {
      this->state = FAULT;
      throw;
    }
  }

  // ============================================================================
  // BaslerGrabber::set_roi
  // ============================================================================
  void BaslerGrabber::set_roi( ROI roi )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_roi" );
    yat::MutexLock guard(this->mutex);

    if (!this->is_open())
    {
      THROW_YAT( "SEQUENCE_ERROR", "The device must be OPEN to configure the ROI" );
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    this->do_set_roi( roi );


    if (was_running)
    {
      this->start();
    }

  }

  // ============================================================================
  // BaslerGrabber::do_set_roi
  // ============================================================================
  void BaslerGrabber::do_set_roi(ROI roi)
  {
    FUNCTION_NAME( "BaslerGrabber::do_set_roi" );
    if ( roi.x > this->camera->get().Width.GetMax()     || roi.x < 0
         || roi.y > this->camera->get().Height.GetMax() || roi.y < 0
         || roi.width > this->camera->get().Width.GetMax() - roi.x  || roi.width < 0
         || roi.height > this->camera->get().Height.GetMax() - roi.y  || roi.height < 0)
    {
      THROW_YAT("INVALID_ARGUMENT", "Wrong ROI specified");
    }


    ROI adjusted_roi;
    /*
    adjusted_roi.x        = BaslerGrabber::adjust( roi.x ,      static_cast<int>(this->camera->OffsetX.GetInc()) );
    adjusted_roi.y        = BaslerGrabber::adjust( roi.y ,      static_cast<int>(this->camera->OffsetY.GetInc()) );
    adjusted_roi.width    = BaslerGrabber::adjust( roi.width ,  static_cast<int>(this->camera->Width.GetInc()  ) );
    adjusted_roi.height   = BaslerGrabber::adjust( roi.height , static_cast<int>(this->camera->Height.GetInc() ) );
    */
    adjusted_roi = roi;

    //- first reset the ROI
    this->camera->get().OffsetX.SetValue(this->camera->get().OffsetX.GetMin());
    this->camera->get().OffsetY.SetValue(this->camera->get().OffsetY.GetMin());
    this->camera->get().Width.SetValue(this->camera->get().Width.GetMax());
    this->camera->get().Height.SetValue(this->camera->get().Height.GetMax());

    //- then send the values (if new width or heigth is incompatible with the previous ROI,
    //- the order is important)

    //- width and height before offsets otherwise it fails (offset is blocked since width is at max)
    this->camera->get().Width   = adjusted_roi.width;
    this->camera->get().Height  = adjusted_roi.height;
    this->camera->get().OffsetX = adjusted_roi.x;
    this->camera->get().OffsetY = adjusted_roi.y;
  }

  // ============================================================================
  // BaslerGrabber::get_roi
  // ============================================================================
  ROI BaslerGrabber::get_roi()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_roi" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Attribute not available if device is not OPEN");
    }

    ROI r;
    r.x      = static_cast<int>(this->camera->get().OffsetX());
    r.y      = static_cast<int>(this->camera->get().OffsetY());
    r.width  = static_cast<int>(this->camera->get().Width());
    r.height = static_cast<int>(this->camera->get().Height());

    return r;
  }

  // ============================================================================
  // BaslerGrabber::reset_roi
  // ============================================================================
  void BaslerGrabber::reset_roi()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::reset_roi" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("SEQUENCE_ERROR", "The device must be OPEN to configure the ROI");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    this->camera->get().OffsetX.SetValue(this->camera->get().OffsetX.GetMin());
    this->camera->get().OffsetY.SetValue(this->camera->get().OffsetY.GetMin());
    this->camera->get().Width.SetValue(this->camera->get().Width.GetMax());
    this->camera->get().Height.SetValue(this->camera->get().Height.GetMax());

    if (was_running)
    {
      this->start();
    }

  }

  // ============================================================================
  // BaslerGrabber::get_settings
  // ============================================================================
  void BaslerGrabber::get_settings( yat::PlugInPropValues& prop_values ) const
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_settings" );
    BaslerGrabber& self = const_cast<BaslerGrabber&>(*this);

    self.get_exposure_time( prop_values["ExposureTime"] );
    self.get_frame_rate( prop_values["FrameRate"] );
    self.get_gain( prop_values["Gain"] );
    self.get_black_level( prop_values["BlackLevel"] );

    self.get_trigger_mode( prop_values["TriggerMode"] );
    self.get_trigger_activation( prop_values["TriggerActivation"] );
    self.get_trigger_line( prop_values["TriggerLine"] );

    ROI roi = self.get_roi();
    std::vector<yat_int32_t> roi_vect(4);
    roi_vect[0] = roi.x;
    roi_vect[1] = roi.y;
    roi_vect[2] = roi.width;
    roi_vect[3] = roi.height;

    prop_values["ROI"] = roi_vect;
  }

  // ============================================================================
  // BaslerGrabber::set_exposure_time
  // ============================================================================
  void BaslerGrabber::set_exposure_time( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_exposure_time" );
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the exposure time");
    }

    yat::MutexLock guard(this->mutex);

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    //- Exposure Time (in ms)
    double value = yat::any_cast<double>(container);
    this->do_set_exposure_time( value );

    if (was_running)
    {
      this->start();
    }

  }

  // ============================================================================
  // BaslerGrabber::get_exposure_time
  // ============================================================================
  void BaslerGrabber::get_exposure_time( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_exposure_time" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the exposure time");
    }

    try
    {
      double value = 1.0E-3 * static_cast<double>(this->camera->get().ExposureTimeAbs.GetValue());
      container = value;
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting exposure time");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting exposure time");
    }
  }

  // ============================================================================
  // BaslerGrabber::set_frame_rate
  // ============================================================================
  void BaslerGrabber::set_frame_rate( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_frame_rate" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the frame rate");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }


    try
    {
      this->camera->get().AcquisitionFrameRateAbs.SetValue( yat::any_cast<double>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting frame rate");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting frame rate");
    }

    if (was_running)
    {
      this->start();
    }

  }

  // ============================================================================
  // BaslerGrabber::get_frame_rate
  // ============================================================================
  void BaslerGrabber::get_frame_rate( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_frame_rate" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the frame rate");
    }

    try
    {
      double value = static_cast<double>(this->camera->get().ResultingFrameRateAbs.GetValue());
      container = value;
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM( e, "Error when getting frame rate");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting frame rate");
    }
  }
  
  long BaslerGrabber::_raw_get_bit_depth() const
  {
    PixelSizeEnums ps = this->camera->get().PixelSize.GetValue();
    switch( ps )
    {
      case PixelSize_Bpp8:
        return 8;
      case PixelSize_Bpp12:
      case PixelSize_Bpp16: //- this is in fact 12 bpp inside a 16bpp image
          return 12;
      default:
        return 0;
    }
  }

  // ============================================================================
  // BaslerGrabber::get_bit_depth
  // ============================================================================
  void BaslerGrabber::get_bit_depth( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_bit_depth" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the bit depth");
    }

    try
    {
      long value = this->_raw_get_bit_depth();
      if (value == 0)
        THROW_YAT("UNKNOWN_VALUE", "Bit depth value is not supported");
      container = static_cast<yat_int32_t>(value);
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM( e, "Error when getting bit depth");
    }
    catch( yat::Exception &)
    {
      throw;
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting bit depth");
    }
  }

  // ============================================================================
  // BaslerGrabber::get_sensor_width
  // ============================================================================
  void BaslerGrabber::get_sensor_width( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_sensor_width" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the sensor width");
    }

    try
    {
      yat_int32_t value = static_cast<yat_int32_t>( this->camera->get().SensorWidth.GetValue() );
      container = value;
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting sensor width");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting sensor width");
    }
  }

  // ============================================================================
  // BaslerGrabber::get_sensor_height
  // ============================================================================
  void BaslerGrabber::get_sensor_height( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_sensor_height" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the sensor height");
    }

    try
    {
      yat_int32_t value = static_cast<yat_int32_t>( this->camera->get().SensorHeight.GetValue() );
      container = value;
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting sensor height");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting sensor height");
    }
  }

  // ============================================================================
  // BaslerGrabber::set_trigger_mode
  // ============================================================================
  void BaslerGrabber::set_trigger_mode( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_trigger_mode" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set TriggerMode");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }


    try
    {
      yat_int32_t mode = yat::any_cast<yat_int32_t>(container);
      if ( mode == 0 )
      {
        //- INTERNAL
        this->camera->get().TriggerMode.SetValue( TriggerMode_Off );
        this->camera->get().AcquisitionFrameRateEnable.SetValue( true );
      }
      else if ( mode == 1 )
      {
        //- EXTERNAL - TIMED
        this->camera->get().TriggerMode.SetValue( TriggerMode_On );
        this->camera->get().AcquisitionFrameRateEnable.SetValue( false );
        this->camera->get().ExposureMode.SetValue( ExposureMode_Timed );
      }
      else if ( mode == 2 )
      {
        //- EXTERNAL - TRIGGER WIDTH
        this->camera->get().TriggerMode.SetValue( TriggerMode_On );
        this->camera->get().AcquisitionFrameRateEnable.SetValue( false );
        this->camera->get().ExposureMode.SetValue( ExposureMode_TriggerWidth );
      }
      else
      {
        THROW_YAT("BAD_ARGUMENT", "'TriggerMode' must be 0 (internal), 1 (external - timed), or 2 (external - pulse width)");
      }
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM( e, "Error when setting TriggerMode" );
    }
    catch( yat::Exception& )
    {
      throw;
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting TriggerMode");
    }

    if ( was_running )
      this->start();
  }

  // ============================================================================
  // BaslerGrabber::get_trigger_mode
  // ============================================================================
  void BaslerGrabber::get_trigger_mode( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_trigger_mode" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get TriggerMode");
    }

    try
    {
      if (this->camera->get().TriggerMode.GetValue() == TriggerMode_Off)
        container = yat_int32_t(0);
      else if (this->camera->get().ExposureMode.GetValue() != ExposureMode_TriggerWidth)
        container = yat_int32_t(1);
      else
        container = yat_int32_t(2);
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting TriggerMode");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting TriggerMode");
    }
  }

  // ============================================================================
  // BaslerGrabber::set_trigger_activation
  // ============================================================================
  void BaslerGrabber::set_trigger_activation( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_trigger_activation" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set TriggerActivation");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      TriggerActivation trigger_activation = static_cast<TriggerActivation>(yat::any_cast<yat_int32_t>(container));
      if ( trigger_activation !=  TriggerActivation_RisingEdge
           && trigger_activation !=  TriggerActivation_FallingEdge)
      {
        THROW_YAT("BAD_ARGUMENT", "'TriggerActivation' must be 0 (rising edge) or 1 (falling edge)");
      }

      this->camera->get().TriggerActivation.SetValue( trigger_activation );

    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting TriggerActivation");
    }
    catch( yat::Exception& )
    {
      throw;
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting TriggerActivation");
    }

    if ( was_running )
      this->start();
  }

  // ============================================================================
  // BaslerGrabber::get_trigger_activation
  // ============================================================================
  void BaslerGrabber::get_trigger_activation( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_trigger_activation" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get TriggerActivation");
    }

    try
    {
      container = static_cast<yat_int32_t>(this->camera->get().TriggerActivation.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting TriggerActivation");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting TriggerActivation");
    }
  }

  // ============================================================================
  // BaslerGrabber::set_trigger_line
  // ============================================================================
  void BaslerGrabber::set_trigger_line( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_trigger_line" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set TriggerLine");
    }

    
    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }
    
    try
    {
      TriggerSourceEnums trigger_line = static_cast<TriggerSourceEnums>(yat::any_cast<yat_int32_t>(container));
      
      if (trigger_line !=  TriggerSource_Line1
          && trigger_line !=  TriggerSource_Line2)
      {
        THROW_YAT("BAD_ARGUMENT", "'TriggerLine' must be 0 (line 1) or 1 (line 2)");
      }

      this->camera->get().TriggerSource.SetValue( trigger_line );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM( e, "Error when setting TriggerLine");
    }
    catch( yat::Exception& )
    {
      throw;
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting TriggerLine");
    }

    
    if ( was_running )
      this->start();
    
  }

  // ============================================================================
  // BaslerGrabber::get_trigger_line
  // ============================================================================
  void BaslerGrabber::get_trigger_line( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_trigger_line" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get TriggerLine");
    }

    try
    {
      container = static_cast<yat_int32_t>(this->camera->get().TriggerSource.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM( e, "Error when getting TriggerLine");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting TriggerLine");
    }
  }

  // ============================================================================
  // BaslerGrabber::adjust
  // ============================================================================
  int BaslerGrabber::adjust( int v, const int i )
  {
    FUNCTION_NAME( "BaslerGrabber::adjust" );
    try
    {
      if (i==1)
      {
        return v;
      }
      else if (i>1)
      {
        int r = (v+(i-1))/i;
        return r*i;
      }
      else
      {
        throw LOGICAL_ERROR_EXCEPTION( "Unexpected increment %d", i );
      }
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM( e, "Error when rounding value");
    }
  }

  // ============================================================================
  // BaslerGrabber::do_set_exposure_time
  // ============================================================================
  void BaslerGrabber::do_set_exposure_time( double exposure_time_ms )
  {
    FUNCTION_NAME( "BaslerGrabber::do_set_exposure_time" );
    // For Basler camera, there are 3 GenICam attributes that can be set
    // for exposure time setup :
    //    + ExposureTimeAbs      (float, microseconds)
    //    + ExposureTimeBaseAbs  (float, microsseconds)
    //    + ExposureTimeRaw      (integer, no unit)
    //
    // with : ExposureTimeAbs = ExposureTimeBaseAbs * ExposureTimeRaw
    //
    // All these parameters are writable, but with the following conditions :
    //   + when writing ExposureTimeAbs,     ExposureTimeRaw is adjusted
    //   + when writing ExposureTimeBaseAbs, ExposureTimeAbs is adjusted
    //   + when writing ExposureTimeRaw,     ExposureTimeAbs is adjusted
    //
    // ExposureTimeRaw is an integer with values until 4095 (min can be > 0 in limit cases)
    // so we must guess the best values for ExposureTimeBaseAbs and ExposureTimeRaw
    // such that ExposureTimeAbs will be as close as possible than the desired value.
    //
    // ExposureTimeBaseAbs gives the number of microseconds represented by one unit of
    // ExposureTimeRaw.
    // To be as precise as possible, ExposureTimeBaseAbs must be minimized

    try
    {

      if (exposure_time_ms * 1E3 < min_exp_time || exposure_time_ms * 1E3 > max_exp_time)
      {
        yat::OSStream oss;
        oss << "ExposureTime must be in the range [ "
            << min_exp_time * 1E-3
            << " , "
            << max_exp_time * 1E-3
            << " ]"
            << std::ends;

        THROW_YAT("OUT_OF_RANGE", oss.str().c_str());
      }

      this->disable_callback = true;

      this->camera->get().ExposureTimeBaseAbs = 100.0; //- to be sure we can set the Raw setting on the full range (1 .. 4095)
      double raw = ::ceil( exposure_time_ms / 50 );
      this->camera->get().ExposureTimeRaw = static_cast<yat_int32_t>(raw);
      raw = static_cast<double>(this->camera->get().ExposureTimeRaw());
      double base = 1E3 * exposure_time_ms / raw;
      this->camera->get().ExposureTimeBaseAbs = 1E3 * exposure_time_ms / this->camera->get().ExposureTimeRaw();
      
      this->disable_callback = false;

    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "BaslerGrabber::do_set_exposure_time");
    }
    catch( yat::Exception& )
    {
      throw;
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting exposure time");
    }

  }

  // ============================================================================
  // BaslerGrabber::set_gain
  // ============================================================================
  void BaslerGrabber::set_gain( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_gain" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the gain");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().GainRaw.SetValue( yat::any_cast<yat_int32_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting gain");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting gain");
    }

    if ( was_running )
      this->start();

  }

  // ============================================================================
  // BaslerGrabber::get_gain
  // ============================================================================
  void BaslerGrabber::get_gain( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_gain" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the gain");
    }

    try
    {
      container = static_cast<yat_int32_t>(this->camera->get().GainRaw.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting gain");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting gain");
    }
  }

  // ============================================================================
  // BaslerGrabber::set_blacklevel
  // ============================================================================
  void BaslerGrabber::set_black_level( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_black_level" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the black level");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().BlackLevelRaw.SetValue( yat::any_cast<yat_int32_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting black level");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting black level");
    }

    if ( was_running )
      this->start();

  }

  // ============================================================================
  // BaslerGrabber::get_black_level
  // ============================================================================
  void BaslerGrabber::get_black_level( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_black_level" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the black level");
    }

    try
    {
      container = static_cast<yat_int32_t>(this->camera->get().BlackLevelRaw.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting black level");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting black level");
    }
  }

  // ============================================================================
  // BaslerGrabber::get_basler_frame_rate
  // ============================================================================
  void BaslerGrabber::get_basler_frame_rate( yat::Any& container )
    throw (yat::Exception)
  {
    container = frame_rate;
  }


  // ============================================================================
  // BaslerGrabber::get_overruns
  // ============================================================================
  void BaslerGrabber::get_overruns( yat::Any& container )
    throw (yat::Exception)
  {
    container = overruns;
  }

  // ============================================================================
  // BaslerGrabber::set_internal_acquisition_buffers
  // ============================================================================
  void BaslerGrabber::set_internal_acquisition_buffers( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_internal_acquisition_buffers" );
    yat::MutexLock guard(this->mutex);
    if ( this->is_running() )
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Acquisition must be stopped before changing the internal acquisition buffer number");
    }

    acquisition_buffer_nb = yat::any_cast<yat_int32_t>(container);
  }

  // ============================================================================
  // BaslerGrabber::get_internal_acquisition_buffers
  // ============================================================================
  void BaslerGrabber::get_internal_acquisition_buffers( yat::Any& container )
    throw (yat::Exception)
  {
    container = acquisition_buffer_nb;
  }

  // ============================================================================
  // BaslerGrabber::set_averaging
  // ============================================================================
  void BaslerGrabber::set_averaging( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_averaging" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set Averaging");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().AveragingNumberOfFrames.SetValue( yat::any_cast<yat_int32_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting Averaging");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting Averaging");
    }

    if ( was_running )
      this->start();
  }

  // ============================================================================
  // BaslerGrabber::get_averaging
  // ============================================================================
  void BaslerGrabber::get_averaging( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_averaging" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get Averaging");
    }

    try
    {
      container = static_cast<yat_int32_t>(this->camera->get().AveragingNumberOfFrames.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting Averaging");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting Averaging");
    }
  }


  // ============================================================================
  // BaslerGrabber::setup_initialization_attributes
  // ============================================================================
  void BaslerGrabber::setup_initialization_attributes()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::setup_initialization_attributes" );
    try
    {
      model_name = this->camera->get().DeviceModelName.GetValue( );
      std::string model_name_start;
      model_name_start += model_name[0];
      model_name_start += model_name[1];

      if ( model_name_start == "sc" )
      {
        model_family = DEVICE_MODEL_SCOUT;
      }
      else if ( model_name_start == "pi" )
      {
        model_family = DEVICE_MODEL_PILOT;
      }
      else
      {
        THROW_YAT("UNKNOWN_CAMERA_MODEL", "Unsupported camera model. Currently supported models are 'Scout' and 'Pilot'");
      }


      this->camera->get().PixelFormat.SetValue( PixelFormat_Mono16 );
      this->camera->get().OffsetX.SetValue( 0 );
      this->camera->get().OffsetY.SetValue( 0 );
      this->camera->get().Width.SetValue( this->camera->get().Width.GetMax() );
      this->camera->get().Height.SetValue( this->camera->get().Height.GetMax() );
      

      // set_roi must be called after MaxBufferSize.SetValue
      if ( !init_cfg.roi.is_empty() )
        this->do_set_roi( init_cfg.roi );

      //- default mode is continuous
      this->camera->get().AcquisitionMode.SetValue( AcquisitionMode_Continuous );
      
      this->camera->get().TriggerSelector.SetValue( TriggerSelector_AcquisitionStart );
      this->camera->get().TriggerSource.SetValue( init_cfg.trigger_line );
      this->camera->get().TriggerActivation.SetValue( init_cfg.trigger_activation );

      if (init_cfg.acq_mode == MODE_INTERNAL_TRIGGER)
      {
        this->camera->get().TriggerMode.SetValue( TriggerMode_Off );
        this->camera->get().AcquisitionFrameRateEnable.SetValue( true );
        this->camera->get().AcquisitionFrameRateAbs.SetValue( init_cfg.frame_rate );
        this->camera->get().ExposureMode.SetValue( ExposureMode_Timed );
      }
      else if (init_cfg.acq_mode == MODE_EXTERNAL_TRIGGER_TIMED
               || init_cfg.acq_mode == MODE_EXTERNAL_TRIGGER_PULSE_WIDTH)
      {
        this->camera->get().TriggerMode.SetValue( TriggerMode_On );
        this->camera->get().AcquisitionFrameRateEnable.SetValue( false );

        if (init_cfg.acq_mode == MODE_EXTERNAL_TRIGGER_TIMED)
          this->camera->get().ExposureMode.SetValue( ExposureMode_Timed );
        else
          this->camera->get().ExposureMode.SetValue( ExposureMode_TriggerWidth );

      }
      this->compute_min_max_exposure();
      this->do_set_exposure_time(init_cfg.exposure_time);
      this->camera->get().GainRaw.SetValue( init_cfg.gain );
      this->camera->get().BlackLevelRaw.SetValue( init_cfg.blacklevel );


      this->camera->get().ChunkModeActive.SetValue( true );

      this->camera->get().ChunkSelector.SetValue( ChunkSelector_Framecounter );
      this->camera->get().ChunkEnable.SetValue( true );

      this->camera->get().ChunkSelector.SetValue( ChunkSelector_Timestamp );
      this->camera->get().ChunkEnable.SetValue( true );

    }
    catch( GenICam::GenericException &e )
    {
      this->state = FAULT;
      THROW_GENICAM(e, "Error when configuring camera");
    }
    catch(yat::Exception&)
    {
      this->state = FAULT;
      throw;
    }
    catch(...)
    {
      this->state = FAULT;
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when configuring camera");
    }
  }


  // ============================================================================
  // BaslerGrabber::compute_min_max_exposure
  // ============================================================================
  void BaslerGrabber::compute_min_max_exposure() throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::compute_min_max_exposure" );
    //- set to default situation
    this->camera->get().ExposureTimeRaw = 100;
    this->camera->get().ExposureTimeBaseAbs = this->camera->get().ExposureTimeBaseAbs.GetMax();
    this->camera->get().ExposureTimeRaw = this->camera->get().ExposureTimeRaw.GetMax();
    this->max_exp_time = this->camera->get().ExposureTimeBaseAbs() * this->camera->get().ExposureTimeRaw();

    this->camera->get().ExposureTimeRaw = 100;
    this->camera->get().ExposureTimeBaseAbs = this->camera->get().ExposureTimeBaseAbs.GetMin();
    this->camera->get().ExposureTimeRaw = this->camera->get().ExposureTimeRaw.GetMin();
    this->min_exp_time = this->camera->get().ExposureTimeBaseAbs() * this->camera->get().ExposureTimeRaw();
  }

}
