/*!
 * \file
 * \brief    Definition of IMAQGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#include "BaslerGrabber.h"

#include <yat/plugin/PlugInSymbols.h>
#include <yat/utils/Singleton.h>
#include <yat/threading/Task.h>
#include <yat/network/Address.h>
#include <cmath>

EXPORT_SINGLECLASS_PLUGIN(GrabAPI::BaslerGrabber, GrabAPI::BaslerGrabberInfo);

#define GRABLOG( p ) std::cout << p  << std::endl

#include <sys/syscall.h>   /* For SYS_xxx definitions */
#include <sys/resource.h>

namespace GrabAPI
{
  // internal constants
  const yat_uint32_t kNUM_BUFFERS = 5;
  const yat_uint32_t kNUM_EVENTBUFFERS = 20;
  const int TLREAD_TIMEOUT = 250;//ms
  const int TLWRITE_TIMEOUT = 250;//ms
  const int TLHEARTBEAT_TIMEOUT = 5000;//ms
  const unsigned int kWAIT_TIMEOUT = 11000;//ms

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
      GRABLOG("TransportLayer::~TransportLayer()");
      if (tl_)
      {
        GRABLOG("TransportLayer::~TransportLayer() Pylon::CTlFactory::GetInstance().ReleaseTl");
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
      GRABLOG("Camera::Camera()");

      // Get all attached cameras and exit if no camera is found
      Pylon::DeviceInfoList_t devices;
      const size_t max_retry = 5;
      size_t retry_count = max_retry;
      int nb_camera_found = 0;
      
      while ( 0 < retry_count-- )
      { 
        GRABLOG( "Camera::Camera() TransportLayer::EnumerateDevices" );
        nb_camera_found = tl->get().EnumerateDevices( devices );
        if ( 0 != nb_camera_found )
          break;
        else
          yat::ThreadingUtilities::sleep( 0, 100 * 1000 * 1000 ); // wait 100 msec
      }

      GRABLOG("Camera::Camera() "<<nb_camera_found<<" cameras found");
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
        GRABLOG("Camera::Camera() Found a "<<gige_device_info.GetModelName()<<" with ip "<<current_ip);
        if (current_ip == pylon_camera_ip)
        {
          GRABLOG("Camera::Camera() This is the camera "<<camera_ip << " ("<<current_ip<<")");
          break;
        }
      }

      if (it == devices.end())
      {
        GRABLOG("Camera::Camera() Camera "<<camera_ip<<" not found!");
        yat::OSStream oss;
        oss << "Camera "
            << camera_ip
            << " not found";
        THROW_YAT("BASLER_DRIVER_ERROR", oss.str().c_str());
      }

      
      // Create the camera object of the first available camera
      // The camera object is used to set and get all available
      // camera features.
      GRABLOG( "Camera::Camera() TransportLayer::CreateDevice" );
      scoped_ptr<Camera_t> camera( new Camera_t(tl->get().CreateDevice( *it )) );

      if( !camera->GetDevice() )
      {
        THROW_YAT("BASLER_CAM_NOT_AVAILABLE", "Unable to get the camera from transport_layer.");
      }

      Camera_t::TlParams_t TlParams(camera->GetTLNodeMap());

      TlParams.ReadTimeout.SetValue(TLREAD_TIMEOUT);
      GRABLOG("Camera::Camera() Get 'Read' timeout: " <<
              std::dec << TlParams.ReadTimeout.GetValue() << " ms");

      TlParams.WriteTimeout.SetValue(TLWRITE_TIMEOUT);
      GRABLOG("Camera::Camera() Get 'Write' timeout: " <<
              std::dec << TlParams.WriteTimeout.GetValue() << " ms");

      TlParams.HeartbeatTimeout.SetValue(TLHEARTBEAT_TIMEOUT);
      GRABLOG("Camera::Camera() Get 'Heartbeat' timeout: " <<
              std::dec << TlParams.HeartbeatTimeout.GetValue() << " ms");

      GRABLOG( "Camera::Camera() Camera::Open" );
      camera->Open();

      // camera object created successfully (no exception) : 
      // keep a reference on the TL since it needs to be destroyed after the camera
      tl_ = tl;
      camera_.swap( camera );
    }

    ~Camera()
    {
      GRABLOG( "Camera::~Camera()" );
      camera_->GetDevice()->DeregisterRemovalCallback(cameraRemovalHandle_);
      camera_->Close();

      // release camera_, THEN tl_ 
      camera_.reset();
      tl_.reset();
    }

    Camera_t& get()
    {
      return *camera_;
    }

    Pylon::DeviceCallbackHandle getRemovalHandle()
    {
      return cameraRemovalHandle_;
    }

    void setRemovalHandle(Pylon::DeviceCallbackHandle h)
    {
      cameraRemovalHandle_ = h;
    }

  private:
    scoped_ptr<Camera_t> camera_;
    shared_ptr<TransportLayer> tl_; // TL must be deleted AFTER the camera object
    Pylon::DeviceCallbackHandle cameraRemovalHandle_;
  };

  class StreamGrabber
  {
  public:
    StreamGrabber( shared_ptr<Camera> camera )
    {
      GRABLOG("StreamGrabber::StreamGrabber()");
      // Get the first stream grabber object of the selected camera
      scoped_ptr<Camera_t::StreamGrabber_t> stream_grabber;
      GRABLOG("StreamGrabber::StreamGrabber() new Camera_t::StreamGrabber_t( camera.GetStreamGrabber(0) )");
      stream_grabber.reset( new Camera_t::StreamGrabber_t(camera->get().GetStreamGrabber( 0 )) );

      stream_grabber->Open();

      // stream grabber opened successfully : keep the reference active to properly close it
      stream_grabber_.swap(stream_grabber);
      // stream grabber depends on camera so it must be active during the lifetime of the stream grabber
      camera_ = camera;
    }

    ~StreamGrabber()
    {
      GRABLOG( "StreamGrabber::~StreamGrabber()" );
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
      GRABLOG("ChunkParser::ChunkParser() Camera::CreateChunkParser");
      chunk_parser_ = camera->get().CreateChunkParser();
      camera_ = camera;
   }

    ~ChunkParser()
    {
      GRABLOG("ChunkParser::~ChunkParser() Camera::DestroyChunkParser");
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
      GRABLOG("RegisteredBuffer::RegisteredBuffer()");
      data_.reset( new yat_uint8_t[image_size] );
      GRABLOG("RegisteredBuffer::RegisteredBuffer() StreamGrabber::RegisterBuffer "  << std::hex << reinterpret_cast<uintptr_t>(data_.get()));
      handle_ = stream_grabber_->get().RegisterBuffer( data_.get(), image_size );
      GRABLOG("RegisteredBuffer::RegisteredBuffer() Successfully registered buffer. handle = "  << std::hex << handle_);
    }
    
    ~RegisteredBuffer()
    {
      GRABLOG( "DeregisterBuffer : " << std::hex << handle_  );
      stream_grabber_->get().DeregisterBuffer( handle_ );
      
      // data must be released after deregistering buffer
      GRABLOG("RegisteredBuffer::~RegisteredBuffer() delete " << std::hex << reinterpret_cast<uintptr_t>(data_.get()));
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
      GRABLOG("RegisteredBufferList::RegisteredBufferList()");
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
      GRABLOG("QueuedBufferList::QueuedBufferList()");
      // queue them
      for (size_t i = 0; i < reg_buffer_list->buf_list_.size() ; ++i)
      {
        RegisteredBuffer& reg_buffer = *(reg_buffer_list->buf_list_[i]);
        GRABLOG("QueuedBufferList::QueuedBufferList() StreamGrabber::QueueBuffer with handle " << std::hex << reg_buffer.handle());
        stream_grabber_->get().QueueBuffer( reg_buffer.handle() );
      }
    }

    ~QueuedBufferList()
    {
      //- put all pending buffers to the stream_grabber's output queue
      GRABLOG("QueuedBufferList::~QueuedBufferList() StreamGrabber::CancelGrab");
      stream_grabber_->get().CancelGrab();

      //- retrieve them to flush the output queue
      GRABLOG("QueuedBufferList::~QueuedBufferList() StreamGrabber::flush queue");
      Pylon::GrabResult r;
      while( stream_grabber_->get().RetrieveResult( r ) )
      {
        // empty loop : it's normal
        //
        // RetrieveResult returns false
        // as soon as there is no buffer in the queue
      }

      // unregister all buffers
      GRABLOG("QueuedBufferList::~QueuedBufferList() delete RegisteredBufferList");
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
      GRABLOG("StreamGrabberRessourceLock::StreamGrabberRessourceLock() PrepareGrab");
      stream_grabber_->get().PrepareGrab();
    }

    ~StreamGrabberRessourceLock()
    {
      GRABLOG("StreamGrabberRessourceLock::StreamGrabberRessourceLock() FinishGrab");
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
        first_image_received_( false ),
        eventGrabber_(grabber_object.camera->get().GetEventGrabber())
    {
#ifndef YAT_WIN32
      termination_event_ = Pylon::WaitObjectEx::Create();
#endif
      try
      {

        //event config
        eventGrabber_.NumBuffer.SetValue(kNUM_EVENTBUFFERS);// must be set before open is called!!
        // Enable resending of event messages when lost messages are detected:
        // Loss of messages is detected by sending acknowledges for every event message.
        // When the camera doesn't receive the acknowledge, it will resend the message up to
        // 'RetryCount' times.
        eventGrabber_.RetryCount = 3;
        eventGrabber_.Open();
        pEventAdapter = grabber_.camera->get().CreateEventAdapter();
        if (! pEventAdapter)
          GRABLOG("AcqTask::AcqTask() Failed to create an event adapter");
        else
          GRABLOG("AcqTask::AcqTask() event adapter well created");
      }
      catch( GenICam::GenericException & e )
      {
        GRABLOG("AcqTask::AcqTask() GenICamException creating the event adapter: "<<e.what());
      }
      catch(...)
      {
        GRABLOG("AcqTask::AcqTask() Exception creating the event adapter");
      }
      wait_objects_.Add( grabber_.stream_grabber->get().GetWaitObject() );  // getting informed about buffers
      wait_objects_.Add( termination_event_ ); // getting informed about termination request
      wait_objects_.Add( eventGrabber_.GetWaitObject() );

      this->go();
    }

    ~AcqTask()
    {
      GRABLOG("AcqTask::~AcqTask()");
      eventGrabber_.Close();
      grabber_.camera->get().DestroyEventAdapter(pEventAdapter);
      GRABLOG("AcqTask::~AcqTask() completely destroyed");
    }

    void start()
    {
      GRABLOG("AcqTask::Start()");
      setpriority(PRIO_PROCESS, syscall(SYS_gettid), -20);
      this->wait_msg_handled( yat::Message::allocate(MSG_ID_START, DEFAULT_MSG_PRIORITY, true ) );
    }

    void stop()
    {
      // signal the termination event to cancel the blocking call where we wait on a new image
      GRABLOG("AcqTask::Stop() termination_event_.Signal()");
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

            if ( grabber_.camera->get().TriggerMode.GetValue() == TriggerMode_On)
              grabber_.state = STANDBY;
            else
              grabber_.state = RUNNING;
          }
          catch( GenICam::GenericException & e )
          {
            // transfer the exception to the caller (this message is 'waited')
            THROW_GENICAM( e, "Error when starting acquisition" );
          }
          catch(...)
          {
            throw;
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
              GRABLOG("wake up due to a wait time out (" <<
                      std::dec << kWAIT_TIMEOUT << "ms)");
              if (grabber_.state == FAULT)
                GRABLOG("It's on fault!");
              else if (grabber_.state == CLOSE)
                this->post( yat::Message::allocate(MSG_ID_STOP) );
              else
                this->post( yat::Message::allocate(MSG_ID_GET_IMAGE) );
              return;
            }

            // index == 0 -> the stream grabber signaled the wait object
            // index == 1 means that AcqTask::stop has been called
            if ( index == 0 )
            {
              grabber_.state = RUNNING;
              if ( grabber_.stream_grabber->get().RetrieveResult( result ) )
              {
                switch( result.Status() )
                {
                  case Pylon::Idle:
                    GRABLOG( "Pylon::Idle result received" );
                    break;
                  case Pylon::Queued:
                    GRABLOG( "Pylon::Queued result received" );
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
                      yat_uint64_t timestamp = static_cast<yat_uint64_t>(grabber_.camera->get().ChunkTimestamp.GetValue());
                      // each clock tick is 8 ns for a Pilot camera
                      grabber_.frame_rate = 125000000. / (double(timestamp) - double(grabber_.last_timestamp));
                      grabber_.last_timestamp = timestamp;
                      grabber_.image_callback(new_image);
                    }
                    break;
                  }
                  case Pylon::Canceled:
                    GRABLOG( "Pylon::Canceled result received" );
                    break;
                  case Pylon::Failed:
                    GRABLOG( "Pylon::Failed result received: (" << std::hex << result.GetErrorCode() <<\
                                                         ") [@" << result.GetTimeStamp() <<\
                                                           "] " << result.GetErrorDescription());
                    break;
                  default:
                    GRABLOG( "Unknown Pylon result received: (" << result.Status() << ")" );
                    break;
                }
              }

              // reuse the buffer for grabbing 
              grabber_.stream_grabber->get().QueueBuffer(result.Handle(), result.Context() );

              if ( current_nb_image_ == nb_image_ )
                this->post( yat::Message::allocate(MSG_ID_STOP) ); 
              else
                this->post( yat::Message::allocate(MSG_ID_GET_IMAGE) ); 
            }
            // index == 1 means that AcqTask::stop has been called
            else if ( index == 1 )
            {
              GRABLOG("AcqTask::stop has been called");
            }
            // index == 2 -> the event grabber signaled the wait object
            else if ( index == 2 )
            {
              GRABLOG("Pylon::EventResult");
              Pylon::EventResult EvResult;
              if (eventGrabber_.RetrieveEvent(EvResult))
              {
                  if (EvResult.Succeeded())
                  {
                      GRABLOG("Successfully got an event message!");
                      // To figure out the content of the event message, pass it to the event adapter.
                      // DeliverMessage will fire the registered callback when the buffer contains
                      // an end-of-exposure event.
                      if (pEventAdapter)
                      {
                        pEventAdapter->DeliverMessage(EvResult.Buffer, sizeof EvResult.Buffer);
                      }
                  }
                  else
                  {
                      GRABLOG("Error retrieving event:" << EvResult.ErrorDescription());
                  }
              }
            }
          }
          catch( GenICam::AccessException & e )
          {
            GRABLOG("Can not access the camera");
            grabber_.state = FAULT;
            THROW_GENICAM(e,"Can not access the camera");
          }
          catch( GenICam::GenericException & e )
          {
            try
            {
              GRABLOG( "camera.AcquisitionStop.Execute()" );
              grabber_.camera->get().AcquisitionStop.Execute();
            }
            catch( ... ) {}
            GRABLOG("GenICam error during acquisition");
            grabber_.state = FAULT;
            THROW_GENICAM( e, "Error during acquisition" );
          }
          catch( yat::Exception & )
          {
            GRABLOG("Yat exception during acquisition");
            grabber_.state = FAULT;
            throw;
          }
          catch(...)
          {
            GRABLOG("Unknown error during acquisition");
            grabber_.state = FAULT;
            THROW_YAT("UNKNOWN_ERROR", "Unknown error during acquisition");
          }
        }
        break;
      case MSG_ID_STOP :
        {
          GRABLOG("MSG_ID_STOP");
          try
          {
            if ( grabber_.state == RUNNING || grabber_.state == STANDBY)
            {
              GRABLOG( "Camera.AcquisitionStop.Execute()" );
              grabber_.camera->get().AcquisitionStop.Execute();
              grabber_.state = OPEN;
            }
          }
          catch( GenICam::GenericException & e )
          {
            // transfer the exception to the caller (this message is 'waited')
            GRABLOG("MSG_ID_STOP exception:"<<e.what());
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
    Camera_t::EventGrabber_t eventGrabber_;
    Pylon::IEventAdapter *pEventAdapter;
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
    this->camera_present = false;
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
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_black_level );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_black_level );
    list.push_back(attr_info);

    attr_info.name   = "Gain";
    attr_info.label  = "Gain";
    attr_info.desc   = "Gain";
    attr_info.unit   = "%";
    attr_info.display_format = "%4.2f";
    attr_info.data_type = yat::PlugInDataType::DOUBLE;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_gain );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_gain );
    list.push_back(attr_info);

    attr_info.name   = "TriggerMode";
    attr_info.label  = "Trigger Mode";
    attr_info.desc   = "Trigger Mode (0 : internal, 1 : external - timed, 2 : external - pulse width)";
    attr_info.unit   = " ";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::INT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_trigger_mode );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_trigger_mode );
    list.push_back(attr_info);

    attr_info.name   = "TriggerLine";
    attr_info.label  = "Trigger Line";
    attr_info.desc   = "Trigger Line (0 : Line1, 1 : Line2)";
    attr_info.unit   = " ";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::INT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_trigger_line );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_trigger_line );
    list.push_back(attr_info);

    attr_info.name   = "TriggerActivation";
    attr_info.label  = "Trigger Activation";
    attr_info.desc   = "Trigger Activation (0 : rising edge, 1 : falling edge)";
    attr_info.unit   = " ";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::INT16;
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
    attr_info.data_type = yat::PlugInDataType::UINT32;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_overruns );
    list.push_back(attr_info);

    attr_info.name   = "InternalAcquisitionBuffers";
    attr_info.label  = "Internal Acquisition Buffers";
    attr_info.desc   = "";
    attr_info.unit   = " ";
    attr_info.display_format = "%6d";
    attr_info.data_type = yat::PlugInDataType::UINT32;
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

    attr_info.name   = "EthernetFrameTransmissionDelay";
    attr_info.label  = "Frame Transmission Delay";
    attr_info.desc   = "This value sets the frame transfer delay for the selected stream channel. This value sets a delay betweem when the camera would normally begin transmitted an acquired image (frame) and when it actually begins transmitting the acquired image. (1 tick = 8ns)";
    attr_info.unit   = "ticks";
    attr_info.display_format = "%6d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_frame_transmission_delay );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_frame_transmission_delay );
    list.push_back(attr_info);

    attr_info.name   = "EthernetInterPacketDelay";
    attr_info.label  = "Inter Packet Delay";
    attr_info.desc   = "This value sets a delay between the transmission of each packet for the selected stream channel. The delay is measured in ticks. (1 tick = 8ns)";
    attr_info.unit   = "ticks";
    attr_info.display_format = "%6d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_packet_transmission_delay );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_packet_transmission_delay );
    list.push_back(attr_info);

    attr_info.name   = "BinningVertical";
    attr_info.label  = "Vertical Binning";
    attr_info.desc   = "Binning increases the camera's response to light by summing the charges from adjacent pixels into one pixel.";
    attr_info.unit   = "pixels";
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_binning_vertical );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_binning_vertical );
    list.push_back(attr_info);

    attr_info.name   = "BinningHorizontal";
    attr_info.label  = "Horizontal Binning";
    attr_info.desc   = "Binning increases the camera's response to light by summing the charges from adjacent pixels into one pixel.";
    attr_info.unit   = "pixels";
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_binning_horizontal );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_binning_horizontal );
    list.push_back(attr_info);

    attr_info.name   = "EthernetPayloadPacketSize";
    attr_info.label  = "Ethernet Payload Packet Size";
    attr_info.desc   = "Size of the ethernet frames to configure jumbo frames.";
    attr_info.unit   = "bytes";
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_ethernet_payload_packet_size );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_ethernet_payload_packet_size );
    list.push_back(attr_info);

    attr_info.name   = "EthernetPayloadImageSize";
    attr_info.label  = "Ethernet Payload Image Size";
    attr_info.desc   = "Total number of bytes sent in a payload.";
    attr_info.unit   = "bytes";
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_ethernet_payload_image_size );
    list.push_back(attr_info);

    attr_info.name   = "EthernetEnableResend";
    attr_info.label  = "Ethernet Enable Resend";
    attr_info.desc   = "Enables the packet resend mechanism.";
    attr_info.unit   = "";
    attr_info.display_format = "%1d";
    attr_info.data_type = yat::PlugInDataType::BOOLEAN;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_ethernet_enable_resend );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_ethernet_enable_resend );
    list.push_back(attr_info);

    attr_info.name   = "EthernetPacketTimeout";
    attr_info.label  = "Ethernet Packet Timeout";
    attr_info.desc   = "How long the filter driver will wait for the next expected packet, before iniciates a resend request.";
    attr_info.unit   = "ms";
    attr_info.display_format = "%4d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_ethernet_packet_timeout );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_ethernet_packet_timeout );
    list.push_back(attr_info);

    attr_info.name   = "EthernetFrameRetention";
    attr_info.label  = "Ethernet Frame Retention";
    attr_info.desc   = "Sets the timeout for the frame retention timer.";
    attr_info.unit   = "ms";
    attr_info.display_format = "%4d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_ethernet_frame_retention );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_ethernet_frame_retention );
    list.push_back(attr_info);

    attr_info.name   = "EthernetReceiveWindowSize";
    attr_info.label  = "Ethernet Receive Window Size";
    attr_info.desc   = "Set the size of the receive window.";
    attr_info.unit   = "packets";
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_ethernet_receive_window_size );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_ethernet_receive_window_size );
    list.push_back(attr_info);

    attr_info.name   = "EthernetBandwidthUse";
    attr_info.label  = "Ethernet Bandwidth Use";
    attr_info.desc   = "Indicates the bandwidth what will be used by the camera to transmit image and chunck feature data and to handle resends and controls data transmissions.";
    attr_info.unit   = "Bytes/second";
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_ethernet_bandwidth_use );
    list.push_back(attr_info);

    attr_info.name   = "Camera_FirmwareVersion";
    attr_info.label  = "Camera Firmware Version";
    attr_info.desc   = "The version of the firmware in the camera";
    attr_info.unit   = "";
    attr_info.display_format = "%3.1f";
    attr_info.data_type = yat::PlugInDataType::STRING;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_firmware_version );
    list.push_back(attr_info);

    attr_info.name   = "Camera_DeviceVersion";
    attr_info.label  = "Camera Device Version";
    attr_info.desc   = "The device version number of the camera";
    attr_info.unit   = "";
    attr_info.display_format = "%3.1f";
    attr_info.data_type = yat::PlugInDataType::STRING;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_device_version );
    list.push_back(attr_info);

    attr_info.name   = "Camera_DeviceModel";
    attr_info.label  = "Camera Device Model";
    attr_info.desc   = "The model name of the camera";
    attr_info.unit   = "";
    attr_info.display_format = "%3.1f";
    attr_info.data_type = yat::PlugInDataType::STRING;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_device_model );
    list.push_back(attr_info);

    attr_info.name   = "Camera_SerialNumber";
    attr_info.label  = "Camera Serial Number";
    attr_info.desc   = "The serial number of the camera";
    attr_info.unit   = "";
    attr_info.display_format = "%3.1f";
    attr_info.data_type = yat::PlugInDataType::STRING;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_serial_number );
    list.push_back(attr_info);

    attr_info.name   = "AcquisitionFrameCount";
    attr_info.label  = "Acquisition Frame Count";
    attr_info.desc   = "Set the number of images to take when a trigger is received.";
    attr_info.unit   = "images";
    attr_info.display_format = "%2d";
    attr_info.data_type = yat::PlugInDataType::UINT16;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_acquisition_frame_count );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_acquisition_frame_count );
    list.push_back(attr_info);

    attr_info.name   = "PixelFormat";
    attr_info.label  = "Pixel Format";
    attr_info.desc   = "Determine the format of the image data transmitted by the camera.";
    attr_info.unit   = "Format";
    attr_info.display_format = "%3.1f";
    attr_info.data_type = yat::PlugInDataType::STRING;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;//_WRITE;
    //attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::set_pixel_format );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_pixel_format );
    list.push_back(attr_info);

    attr_info.name = "TimeStamp";
    attr_info.label = "Time Stamp";
    attr_info.desc = "Time stamp of the last acquired image.";
    attr_info.unit = " ";
    attr_info.display_format = "%8d";
    attr_info.data_type = yat::PlugInDataType::UINT64;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_last_timestamp );
    list.push_back(attr_info);

    attr_info.name   = "TemperatureSensorBoard";
    attr_info.label  = "Sensor Board Temperature";
    attr_info.desc   = "Show current temperature of the sensor board";
    attr_info.unit   = "celcius";
    attr_info.display_format = "%3.1f";
    attr_info.data_type = yat::PlugInDataType::DOUBLE;
    attr_info.write_type = yat::PlugInAttrWriteType::READ;
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<BaslerGrabber&>(*this), &BaslerGrabber::get_sensor_board_temperature );
    list.push_back(attr_info);
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
    prop_infos["Gain"]              = yat::PlugInPropType::DOUBLE;
    prop_infos["BlackLevel"]        = yat::PlugInPropType::UINT16;
    prop_infos["TriggerMode"]       = yat::PlugInPropType::INT16;
    prop_infos["TriggerLine"]       = yat::PlugInPropType::INT16;
    prop_infos["TriggerActivation"] = yat::PlugInPropType::INT16;
    prop_infos["FrameTransmissionDelay"]    = yat::PlugInPropType::UINT16;
    prop_infos["BinningVertical"]           = yat::PlugInPropType::UINT8;
    prop_infos["BinningHorizontal"]         = yat::PlugInPropType::UINT8;
    prop_infos["EthernetPayloadPacketSize"] = yat::PlugInPropType::UINT16;
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
    try
    {
      this->camera_ip = yat::any_cast<std::string>(prop_values["CameraIP"]);
    }
    catch(yat::Exception& ex)
    {
      std::cout << "Oh lala: " << ex.errors[0].reason << std::endl << ex.errors[0].desc << std::endl;
      RETHROW_YAT_ERROR(ex,
                        "PROPERTY_ERROR",
                        "Error while setting 'CameraIP' property",
                        "BaslerGrabber::set_properties");
    }

#   define REGISTER_PROP( prop_name, type, type_ref )                          \
    {                                                                          \
      yat::Any& prop_value = prop_values[prop_name];                           \
      GRABLOG( "Get Pluggin property " << prop_name );                         \
      if (!prop_value.empty())                                                 \
        type_ref = yat::any_cast<type>(prop_value);                            \
    }

#   define REGISTER_ENUM( prop_name, type, type_ref )                          \
    {                                                                          \
      yat::Any& prop_value = prop_values[prop_name];                           \
      GRABLOG( "Get Pluggin property " << prop_name << " (enum)");             \
      if (!prop_value.empty()){                                                \
        type_ref = static_cast<type>(yat::any_cast<yat_int16_t>(prop_value));  \
        }\
    }

    REGISTER_PROP("ExposureTime",             double,             init_cfg.exposure_time);
    REGISTER_PROP("FrameRate",                double,             init_cfg.frame_rate);
    REGISTER_PROP("Gain",                     double,             init_cfg.gain);
    REGISTER_PROP("BlackLevel",               yat_uint16_t,       init_cfg.blacklevel);
    REGISTER_ENUM("TriggerMode",              Mode,               init_cfg.acq_mode);
    REGISTER_ENUM("TriggerLine",              TriggerSourceEnums, init_cfg.trigger_line);
    REGISTER_ENUM("TriggerActivation",        TriggerActivation,  init_cfg.trigger_activation);
    REGISTER_PROP("FrameTransmissionDelay",   yat_uint16_t,       init_cfg.frame_transmission_delay);
    REGISTER_PROP("BinningVertical",          yat_uint16_t,       init_cfg.binning_vertical);
    REGISTER_PROP("BinningHorizontal",        yat_uint16_t,       init_cfg.binning_horizontal);
    REGISTER_PROP("EthernetPayloadPacketSize",yat_uint16_t,       init_cfg.ethernet_payload_packet_size);

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
  // BaslerGrabber::set_state
  // ============================================================================
  void BaslerGrabber::set_state( GrabberState mystate)
  {
    state = mystate;
  }
  

  // ============================================================================
  // BaslerGrabber::is_open
  // ============================================================================
  bool BaslerGrabber::is_open( void ) const
  {
    return state == OPEN || state == RUNNING || state == STANDBY;
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
  // BaslerGrabber::is_camera_present
  // ============================================================================
  bool BaslerGrabber::is_camera_present( void ) const
  {
    return camera_present;
  }

  // ============================================================================
  // BaslerGrabber::open
  // ============================================================================
  void BaslerGrabber::open()
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::open" );
    GRABLOG("BaslerGrabber::Open()");
    yat::MutexLock guard(this->mutex);
    try
    {
      transport_layer.reset( new TransportLayer() );
      camera.reset( new Camera(transport_layer, this->camera_ip) );
      register_camera_removal_cb();//camera->registerRemovalCallback();
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
    GRABLOG("BaslerGrabber::Close()");
    yat::MutexLock guard(this->mutex);
    try
    {
      chunk_parser.reset();
      stream_grabber.reset();
      camera.reset();
      transport_layer.reset();
      this->state = CLOSE;
      camera_present = false;
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
    GRABLOG("BaslerGrabber::Start()");
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
    GRABLOG("BaslerGrabber::Stop()");
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
    GRABLOG("BaslerGrabber::Snap()");
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
  // BaslerGrabber::reset_camera
  // ============================================================================
  void BaslerGrabber::reset_camera()
    throw (yat::Exception)
  {
    //FUNCTION_NAME( "BaslerGrabber::reset_camera" );
    GRABLOG("BaslerGrabber::ResetCamera()");
    yat::MutexLock guard(this->mutex);

    this->camera.reset();
    this->camera->get().DeviceReset.Execute();
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
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the exposure time");
    }

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
      container = static_cast<yat_int16_t>(value);
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
      yat_int16_t value = static_cast<yat_int16_t>( this->camera->get().SensorWidth.GetValue() );
      container = yat::any_cast<yat_int16_t>(value);
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
      yat_int16_t value = static_cast<yat_int16_t>( this->camera->get().SensorHeight.GetValue() );
      container = yat::any_cast<yat_int16_t>(value);
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
  // BaslerGrabber::get_sensor_board_temperature
  // ============================================================================
  void BaslerGrabber::get_sensor_board_temperature( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_sensor_board_temperature" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the sensor board temperature");
    }

    try
    {
      double value = static_cast<double>(this->camera->get().TemperatureAbs.GetValue());
      container = value;
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting sensor board temperature");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting sensor board temperature");
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
      yat_int16_t mode = yat::any_cast<yat_int16_t>(container);
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
        container = yat_int16_t(0);
      else if (this->camera->get().ExposureMode.GetValue() != ExposureMode_TriggerWidth)
        container = yat_int16_t(1);
      else
        container = yat_int16_t(2);
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
      TriggerActivation trigger_activation = static_cast<TriggerActivation>(yat::any_cast<yat_int16_t>(container));
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
      container = static_cast<yat_int16_t>(this->camera->get().TriggerActivation.GetValue());
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
      TriggerSourceEnums trigger_line = static_cast<TriggerSourceEnums>(yat::any_cast<yat_int16_t>(container));
      
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
      container = static_cast<yat_int16_t>(this->camera->get().TriggerSource.GetValue());
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
    return NULL;
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
      //double base = 1E3 * exposure_time_ms / raw;//unused
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
      double percentage = yat::any_cast<double>(container);
      yat_uint16_t minvalue = static_cast<yat_uint16_t>(this->camera->get().GainRaw.GetMin());
      yat_uint16_t maxvalue = static_cast<yat_uint16_t>(this->camera->get().GainRaw.GetMax());
      yat_uint16_t value = static_cast<yat_uint16_t>(((maxvalue-minvalue)*percentage)/100)+minvalue;
      //this->camera->get().GainRaw.SetValue( yat::any_cast<yat_uint16_t>(container) );
      this->camera->get().GainRaw.SetValue( value );
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
      yat_uint16_t minvalue = static_cast<yat_uint16_t>(this->camera->get().GainRaw.GetMin());
      yat_uint16_t maxvalue = static_cast<yat_uint16_t>(this->camera->get().GainRaw.GetMax());
      yat_uint16_t value = static_cast<yat_uint16_t>(this->camera->get().GainRaw.GetValue());
      double percentage = ((value-minvalue)*100)/(maxvalue-minvalue);
      container = yat::any_cast<double>(percentage);
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
      this->camera->get().BlackLevelRaw.SetValue( yat::any_cast<yat_uint16_t>(container) );
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
      container = static_cast<yat_uint16_t>(this->camera->get().BlackLevelRaw.GetValue());
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

    acquisition_buffer_nb = yat::any_cast<yat_uint32_t>(container);
  }

  // ============================================================================
  // BaslerGrabber::get_internal_acquisition_buffers
  // ============================================================================
  void BaslerGrabber::get_internal_acquisition_buffers( yat::Any& container )
    throw (yat::Exception)
  {
    container = acquisition_buffer_nb;
  }

//  // ============================================================================
//  // BaslerGrabber::set_averaging
//  // ============================================================================
//  void BaslerGrabber::set_averaging( const yat::Any& container )
//    throw (yat::Exception)
//  {
//    FUNCTION_NAME( "BaslerGrabber::set_averaging" );
//    yat::MutexLock guard(this->mutex);
//    if (!this->is_open())
//    {
//      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set Averaging");
//    }
//
//    bool was_running = false;
//    if (this->is_running())
//    {
//      this->stop();
//      was_running = true;
//    }
//
//    try
//    {
//      this->camera->get().AveragingNumberOfFrames.SetValue( yat::any_cast<yat_int32_t>(container) );
//    }
//    catch( GenICam::GenericException &e )
//    {
//      THROW_GENICAM(e, "Error when setting Averaging");
//    }
//    catch(...)
//    {
//      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting Averaging");
//    }
//
//    if ( was_running )
//      this->start();
//  }

//  // ============================================================================
//  // BaslerGrabber::get_averaging
//  // ============================================================================
//  void BaslerGrabber::get_averaging( yat::Any& container )
//    throw (yat::Exception)
//  {
//    FUNCTION_NAME( "BaslerGrabber::get_averaging" );
//    yat::MutexLock guard(this->mutex);
//    if (!this->is_open())
//    {
//      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get Averaging");
//    }
//
//    try
//    {
//      container = static_cast<yat_int32_t>(this->camera->get().AveragingNumberOfFrames.GetValue());
//    }
//    catch( GenICam::GenericException &e )
//    {
//      THROW_GENICAM(e, "Error when getting Averaging");
//    }
//    catch(...)
//    {
//      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting Averaging");
//    }
//  }

  // ============================================================================
  // BaslerGrabber::set_frame_transmission_delay
  // ============================================================================
  void BaslerGrabber::set_frame_transmission_delay( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_frame_transmission_delay" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the frame transmission delay");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().GevSCFTD.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting frame transmission delay");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting frame transmission delay");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_frame_transmission_delay
  // ============================================================================
  void BaslerGrabber::get_frame_transmission_delay( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_frame_transmission_delay" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the frame transmission delay");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().GevSCFTD.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting frame transmission delay");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting frame transmission delay");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_packet_transmission_delay
  // ============================================================================
  void BaslerGrabber::set_packet_transmission_delay( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_packet_transmission_delay" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the packet transmission delay");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().GevSCPD.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting packet transmission delay");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting packet transmission delay");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_packet_transmission_delay
  // ============================================================================
  void BaslerGrabber::get_packet_transmission_delay( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_packet_transmission_delay" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the packet transmission delay");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().GevSCPD.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting packet transmission delay");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting packet transmission delay");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_binning_vertical
  // ============================================================================
  void BaslerGrabber::set_binning_vertical( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_binning_vertical" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the vertical binning");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().BinningVertical.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting vertical binning");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting vertical binning");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_binning_vertical
  // ============================================================================
  void BaslerGrabber::get_binning_vertical( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_binning_vertical" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the vertical binning");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().BinningVertical.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting vertical binning");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting vertical binning");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_binning_horizontal
  // ============================================================================
  void BaslerGrabber::set_binning_horizontal( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_binning_horizontal" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the horizontal binning");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().BinningHorizontal.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting horizontal binning");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting horizontal binning");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_binning_horizontal
  // ============================================================================
  void BaslerGrabber::get_binning_horizontal( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_binning_horizontal" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the horizontal binning");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().BinningHorizontal.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting horizontal binning");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting horizontal binning");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_ethernet_payload_packet_size
  // ============================================================================
  void BaslerGrabber::set_ethernet_payload_packet_size( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_ethernet_payload_packet_size" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the ethernet payload packet size");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->camera->get().GevSCPSPacketSize.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting ethernet payload packet size");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting ethernet payload packet size");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_ethernet_payload_packet_size
  // ============================================================================
  void BaslerGrabber::get_ethernet_payload_packet_size( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_ethernet_payload_packet_size" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the ethernet payload packet size");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().GevSCPSPacketSize.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting ethernet payload packet size");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting ethernet payload packet size");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_ethernet_payload_image_size
  // ============================================================================
  void BaslerGrabber::get_ethernet_payload_image_size( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_ethernet_payload_image_size" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the ethernet payload image size");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().PayloadSize.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting ethernet payload image size");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting ethernet payload image size");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_ethernet_enable_resend
  // ============================================================================
  void BaslerGrabber::set_ethernet_enable_resend( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_ethernet_enable_resend" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the ethernet enable resend");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->stream_grabber->get().EnableResend.SetValue( yat::any_cast<bool>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting ethernet enable resend");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting ethernet enable resend");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_ethernet_enable_resend
  // ============================================================================
  void BaslerGrabber::get_ethernet_enable_resend( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_ethernet_enable_resend" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the ethernet enable resend");
    }

    try
    {
      container = static_cast<bool>(this->stream_grabber->get().EnableResend.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting ethernet enable resend");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting ethernet enable resend");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_ethernet_packet_timeout
  // ============================================================================
  void BaslerGrabber::set_ethernet_packet_timeout( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_ethernet_packet_timeout" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the ethernet packet timeout");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->stream_grabber->get().PacketTimeout.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting ethernet packet timeout");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting ethernet packet timeout");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_ethernet_packet_timeout
  // ============================================================================
  void BaslerGrabber::get_ethernet_packet_timeout( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_ethernet_packet_timeout" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the ethernet packet timeout");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->stream_grabber->get().PacketTimeout.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting ethernet packet timeout");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting ethernet packet timeout");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_ethernet_frame_retention
  // ============================================================================
  void BaslerGrabber::set_ethernet_frame_retention( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_ethernet_frame_retention" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the ethernet frame retention");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->stream_grabber->get().FrameRetention.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting ethernet frame retention");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting ethernet frame retention");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_ethernet_frame_retention
  // ============================================================================
  void BaslerGrabber::get_ethernet_frame_retention( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_ethernet_frame_retention" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the ethernet frame retention");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->stream_grabber->get().FrameRetention.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting ethernet frame retention");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting ethernet frame retention");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_ethernet_receive_window_size
  // ============================================================================
  void BaslerGrabber::set_ethernet_receive_window_size( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_ethernet_receive_window_size" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the ethernet receive window size");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      this->stream_grabber->get().ReceiveWindowSize.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting ethernet receive window size");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting ethernet receive window size");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_ethernet_receive_window_size
  // ============================================================================
  void BaslerGrabber::get_ethernet_receive_window_size( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_ethernet_receive_window_size" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the ethernet receive window size");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->stream_grabber->get().ReceiveWindowSize.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting ethernet receive window size");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting ethernet receive window size");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_ethernet_bandwidth_use
  // ============================================================================
  void BaslerGrabber::get_ethernet_bandwidth_use( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_ethernet_bandwidth_use" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the ethernet bandwidth use");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().GevSCBWA.GetValue());
      //FIXME: or it's the GevSCDCT?
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting ethernet bandwidth use");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting ethernet bandwidth use");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_firmware_version
  // ============================================================================
  void BaslerGrabber::get_firmware_version( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_firmware_version" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the firmware version");
    }

    try
    {
      std::string value = static_cast<std::string>(this->camera->get().DeviceFirmwareVersion.GetValue());
      size_t where = static_cast<size_t>(value.find(';V'));
      container = value.substr(where,value.size() - where);
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting firmware version");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting firmware version");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_device_version
  // ============================================================================
  void BaslerGrabber::get_device_version( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_device_version" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the device version");
    }

    try
    {
      container = static_cast<std::string>(this->camera->get().DeviceVersion.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting device version");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting device version");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_device_model
  // ============================================================================
  void BaslerGrabber::get_device_model( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_device_model" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the device model");
    }

    try
    {
      container = static_cast<std::string>(this->camera->get().DeviceModelName.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting device model");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting device model");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_serial_number
  // ============================================================================
  void BaslerGrabber::get_serial_number( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_serial_number" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the serial number");
    }

    try
    {
      container = static_cast<std::string>(this->camera->get().DeviceID.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting serial number");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting serial number");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_acquisition_frame_count
  // ============================================================================
  void BaslerGrabber::set_acquisition_frame_count( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_acquisition_frame_count" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the acquisition frame count");
    }

    if (this->is_running())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must NOT be RUNNING to set the acquisition frame count");
    }

    try
    {
      this->camera->get().AcquisitionFrameCount.SetValue( yat::any_cast<yat_uint16_t>(container) );
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting acquisition frame count");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting acquisition frame count");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_acquisition_frame_count
  // ============================================================================
  void BaslerGrabber::get_acquisition_frame_count( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_acquisition_frame_count" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the acquisition frame count");
    }

    try
    {
      container = static_cast<yat_uint16_t>(this->camera->get().AcquisitionFrameCount.GetValue());
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting acquisition frame count");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting acquisition frame count");
    }
  }


  // ============================================================================
  // BaslerGrabber::set_pixel_format
  // ============================================================================
  void BaslerGrabber::set_pixel_format( const yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::set_pixel_format" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to set the pixel format");
    }

    bool was_running = false;
    if (this->is_running())
    {
      this->stop();
      was_running = true;
    }

    try
    {
      pixel_format = yat::any_cast<std::string>(container);
      if (pixel_format.compare("Mono8") == 0)
        this->camera->get().PixelFormat.SetValue(PixelFormat_Mono8);
      else if (pixel_format.compare("Mono12") == 0)
        this->camera->get().PixelFormat.SetValue(PixelFormat_Mono12Packed);
      else if (pixel_format.compare("Mono16") == 0)
        this->camera->get().PixelFormat.SetValue(PixelFormat_Mono16);
      else
        THROW_YAT("UNSUPPORTED FORMAT", "This format is not supported.");

    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when setting pixel format");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when setting pixel format");
    }

    if ( was_running )
      this->start();
  }


  // ============================================================================
  // BaslerGrabber::get_pixel_format
  // ============================================================================
  void BaslerGrabber::get_pixel_format( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_pixel_format" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the pixel format");
    }

    try
    {
      PixelFormatEnums pixelFormatInt = this->camera->get().PixelFormat.GetValue();
      //std::cout << "pixel format = " << pixelFormat << std::endl;
      switch( pixelFormatInt )
        {
          //MONO:
          //8
          case PixelFormat_Mono8: //PixelFormat_Mono8Signed
            pixel_format = static_cast<std::string>("Mono8");break;
          //12
          case PixelFormat_Mono12Packed:
            pixel_format = static_cast<std::string>("Mono12");break;
          //16
          case PixelFormat_Mono16:
            pixel_format = static_cast<std::string>("Mono16");break;
          //else
          default:
            pixel_format = static_cast<std::string>("Unknown");
        }
      container = pixel_format;
    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting pixel format");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting pixel format");
    }
  }


  // ============================================================================
  // BaslerGrabber::get_last_timestamp
  // ============================================================================
  void BaslerGrabber::get_last_timestamp( yat::Any& container )
    throw (yat::Exception)
  {
    FUNCTION_NAME( "BaslerGrabber::get_last_timestamp" );
    yat::MutexLock guard(this->mutex);
    if (!this->is_open())
    {
      THROW_YAT("ATTRIBUTE_UNAVAILABLE", "Device must be OPEN to get the time stamp");
    }

    try
    {
      //std::cout << "get_timestamp = " << last_timestamp << std::endl;
      //container = yat::any_cast<yat_uint64_t>(last_timestamp);
      container = static_cast<yat_uint64_t>(last_timestamp);

    }
    catch( GenICam::GenericException &e )
    {
      THROW_GENICAM(e, "Error when getting time stamp");
    }
    catch(...)
    {
      THROW_YAT("UNKNOWN_ERROR", "Unknown error when getting time stamp");
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

      GRABLOG("Device Model Name \"" << model_name << "\"");
      if ( model_name_start == "sc" )
      {
        model_family = DEVICE_MODEL_SCOUT;
        GRABLOG("Recognized a Basler Scout");
      }
      else if ( model_name_start == "pi" )
      {
        model_family = DEVICE_MODEL_PILOT;
        GRABLOG("Recognized a Basler Pilot");
      }
      else if ( model_name_start == "ac" )
      {
        model_family = DEVICE_MODEL_ACE;
        GRABLOG("Recognized a Basler Ace");
        THROW_YAT("UNKNOWN_CAMERA_MODEL", "Not yet supported 'Ace' model. Currently supported models are 'Scout' and 'Pilot'");
        //at least problems with PixelFormat and ExposureTime settings
      }
      else
      {
        GRABLOG("Not recognized camera");
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
      yat_uint16_t minvalue = static_cast<yat_uint16_t>(this->camera->get().GainRaw.GetMin());
      yat_uint16_t maxvalue = static_cast<yat_uint16_t>(this->camera->get().GainRaw.GetMax());
      yat_uint16_t value = static_cast<yat_uint16_t>(((maxvalue-minvalue)*init_cfg.gain)/100)+minvalue;
      //this->camera->get().GainRaw.SetValue( init_cfg.gain );
      this->camera->get().GainRaw.SetValue( value );

      this->camera->get().BlackLevelRaw.SetValue( init_cfg.blacklevel );
      this->camera->get().GevSCFTD.SetValue( init_cfg.frame_transmission_delay);

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

  void BaslerGrabber::register_camera_removal_cb()
  {
    GRABLOG("BaslerGrabber::register_camera_removal_cb()"
            " Register Camera Surprise Removal Callback");

    Pylon::IPylonDevice* m_pCamera = camera->get().GetDevice();
    GrabAPI::BaslerGrabber &ref2this = *this;
    this->camera_present = true;
    camera->setRemovalHandle(Pylon::RegisterRemovalCallback(m_pCamera,ref2this,&BaslerGrabber::removal_callback_method));
  }

  void BaslerGrabber::removal_callback_method(Pylon::IPylonDevice* ipyloncamera)
  {
    GRABLOG("BaslerGrabber::removal_callback_method()"
            " Detected camera surprise removal");
    GRABLOG("BaslerGrabber::removal_callback_method() Deregister callback");
    ipyloncamera->DeregisterRemovalCallback(camera->getRemovalHandle());
    this->state = FAULT;
    this->camera_present = false;
    GRABLOG("BaslerGrabber::removal_callback_method() change state to fault");

  }

}
