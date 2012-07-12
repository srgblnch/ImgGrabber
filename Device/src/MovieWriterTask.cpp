
#include "MovieWriterTask.h"

#include <isl/ErrorHandler.h>
#include <isl/Exception.h>
#include <isl/Image.h>
#include <isl/movie/MovieWriter.h>

#include <sstream>
#include <iostream>
#include <stdint.h>



/// @todo fora!!
#include <unistd.h>
#include <sched.h>

       #define _GNU_SOURCE        /* or _BSD_SOURCE or _SVID_SOURCE */
       #include <unistd.h>
       #include <sys/syscall.h>   /* For SYS_xxx definitions */
       #include <sys/time.h>
       #include <sys/resource.h>



#define kMSG_WRITE            ( yat::FIRST_USER_MSG      )
/// @todo
//#define kMSG_START_RECORDING  ( yat::FIRST_USER_MSG + 1  )
//#define kMSG_STOP_RECORDING   ( yat::FIRST_USER_MSG + 2  )

namespace GrabAPI {


MovieWriterTask::MovieWriterTask()
:   yat::Task( Config( false, // timeout msg
                       0,
                       false, // no periodic msg
                       0,
                       false,  // use the lock during msg handling
                       kDEFAULT_LO_WATER_MARK,
                       kDEFAULT_HI_WATER_MARK,
                       true,
                       0 )),
    movie_writer(0),
    m_finishing(false)
{
}

MovieWriterTask::~MovieWriterTask()
{
    SAFE_DELETE_PTR(this->movie_writer);
}

void MovieWriterTask::handle_message(yat::Message& _msg)
  throw (yat::Exception)
{
  switch (_msg.type())
  {
  case yat::TASK_INIT:         this->on_init(_msg);            break;
  case yat::TASK_EXIT:         this->on_exit();                break;
  case yat::TASK_TIMEOUT:                                      break;
  case kMSG_WRITE:             this->on_write(_msg);           break;
  case kMSG_START_RECORDING:   this->on_start_recording(_msg); break;
  case kMSG_STOP_RECORDING:    this->on_stop_recording();      break;
  }
}

void MovieWriterTask::on_init(yat::Message & _msg)
{
  // struct sched_param param;
  // param.sched_priority = 0;
  // pthread_setschedparam(pthread_self(), SCHED_RR, &param);

  int ret = setpriority(PRIO_PROCESS, syscall(SYS_gettid), 0);

  /*
  std::cout << "on_init begin" << std::endl;

  isl::MovieConfig *_cfg;
  _msg.detach_data(_cfg);
  config = *_cfg; /// @todo segur que aixo ho vull ?

  std::cout << "on_init end" << std::endl;
  */
}

void MovieWriterTask::on_exit()
{
}

void MovieWriterTask::on_write(yat::Message & _msg)
{
  SharedImage* shared_image;
  _msg.detach_data(shared_image);

  static size_t cagarro = 0;
  //std::cout << "MovieWriterTask::on_write()" << std::endl;
  std::cout << "MovieWriterTask::on_write() " << m_finishing << " " << cagarro << std::endl;
  cagarro ++;

  if (this->movie_writer != 0)
  {
    const GrabAPI::Image* img = shared_image->get_image();

    /// @todo eing?? cal? Comorl?
    isl::Image record_image( static_cast<int>(img->width()), static_cast<int>(img->height()), isl::ISL_STORAGE_USHORT );
    record_image.unserialize(static_cast<void*>(img->base()));

    try
    {
      //here is where the ISL is used to store the image in a file
      this->movie_writer->write_frame(record_image);
      //usleep(1000000);
      //usleep(500000);
      //yat::Timestamp ts;
      //_GET_TIME( ts );
      //if(_ELAPSED_SEC(this->movie_start, ts)>this->movie_config.duration_s)
      //{
      //  this->on_stop_recording();
      //}
    }
    catch( isl::Exception& ex )
    {
      std::cout << "MovieWriterTask::on_write() EXCEPCIO ISL" << std::endl;
      isl::ErrorHandler::reset();
      SAFE_RELEASE(shared_image);
      this->on_stop_recording();
      throw;
    }
    catch (...)
    {
      std::cout << "MovieWriterTask::on_write() EXCEPCIO RARA" << std::endl;
      SAFE_RELEASE(shared_image);
      this->on_stop_recording();
      throw;
    }
  }
  SAFE_RELEASE(shared_image);
}

  //! START_RECORDING msg handler
void MovieWriterTask::on_start_recording(yat::Message& _msg)
  throw (yat::Exception)
{
  std::cout << "MovieWriterTask::on_start_recording()" << std::endl;
  if ( this->is_saving_movie() )
  {
      THROW_YAT_ERROR("SEQUENCE_ERROR",
                      "A movie is already being saved. Stop the"
                      " current movie first",
                      "GrabberTask::on_start_recording");
  }

  //- check if it is possible to save a video
  //- there are problem in the movie when the width of an image is odd

  ///@todo versionar
  /*
      ROI r = this->get_roi();
      if( (r.width & 1) != 0 )
      {
          THROW_YAT_ERROR("DIMENSION_ERROR",
                          "The width of the image must have an even number of"
                          " pixels\nUse the 'SetROI' command to define a ROI"
                          " suitable for saving a movie",
                          "GrabberTask::on_start_recording");
      }
  */
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
    //std::cout << "on_start_recording 10" << std::endl;
    //- get the bit depth from the grabber object

    //std::cout << "on_start_recording 20" << std::endl;
    isl::MovieConfig isl_conf;
    isl_conf.format = this->movie_config.format;
    isl_conf.file_basename = this->movie_config.file_basename;
    isl_conf.bit_depth = this->bit_depth;
    /// @todo adaptar        isl_conf.frame_rate = this->fps_computer.get_frame_rate();

    //std::cout << "on_start_recording 30" << std::endl;
    this->movie_writer = isl::MovieWriterFactory::create( isl_conf );
    //std::cout << "on_start_recording 40" << std::endl;
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
                      "MovieWriterTask::on_start_recording");
  }
  catch(...)
  {
    THROW_YAT_ERROR("UNKNOWN_ERROR",
                    "Unable to instantiate MovieWriter",
                    "MovieWriterTask::on_start_recording");
  }
}

void MovieWriterTask::start_recording(GrabAPI::RecordMovieConfig rmconf, int32_t bit_depth)
{
  std::cout << "MovieWriterTask::start_recording()" << std::endl;
  this->bit_depth = bit_depth;
  /// @todo potser podria posar aqui lo de la ROI

  yat::Message* msg = yat::Message::allocate( kMSG_START_RECORDING, DEFAULT_MSG_PRIORITY, true );
  
  msg->attach_data(rmconf);

  this->wait_msg_handled(msg, 3000);
}

void MovieWriterTask::on_stop_recording()
  throw (yat::Exception)
{
  std::cout << "MovieWriterTask::on_stop_recording()" << std::endl;
  SAFE_DELETE_PTR(this->movie_writer);
  m_finishing = false;
}

void MovieWriterTask::write(const SharedImage* shared_image)
{
  // get a cheap duplicate of the shared image, so that when we
  // return the execution it won't be deleted
  /// @todo ai aques const_cast...

  if (!this->is_saving_movie() || m_finishing)
    return;

  std::cout << "MovieWriterTask::write()" << std::endl;
  static size_t cagarro = 0;

  yat::Timestamp ts;
  _GET_TIME( ts );

  SharedImage* img = const_cast<SharedImage*>(shared_image)->duplicate();

  yat::Message* msg = 0;

  //std::cout << "CACAEO: " << cagarro << ", " << ts.tv_sec << ":" << ts.tv_usec << std::endl;

  if(_ELAPSED_SEC(this->movie_start, ts)>this->movie_config.duration_s)
  {
    //this->on_stop_recording();
    ///@todo pero ara aqui tb ens falta un stop diferent per temps real, no? o,mmm
    /// aix... que no s'e pas com ni si cal de debo...
    msg = yat::Message::allocate(kMSG_STOP_RECORDING, DEFAULT_MSG_PRIORITY, true);
    
    /// @todo i home no enviar el missatge cada vegada tp no es bo xq aleshores se n'envien varios
    /// abans noe l processa de debo

    cagarro = 0;
    m_finishing = true;
  }
  else
  {
    cagarro++;
    msg = yat::Message::allocate(kMSG_WRITE, DEFAULT_MSG_PRIORITY, true);
    msg->attach_data(img);
  }

  this->post(msg);
  /**
  @todo opaposapo
  msg queue maxim tamany lala?
  SharedImage no pas a message, xq caldria algo que no fos punter i shi pogues fer delete!
  */
}

bool MovieWriterTask::is_saving_movie( void ) const
{
  return this->movie_writer != 0;
}

std::string MovieWriterTask::movie_remaining_time( void ) const
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

} // namespace GrabAPI
