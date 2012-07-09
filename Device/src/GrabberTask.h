/*!
 * \file
 * \brief    Declaration of GrabberTask class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#ifndef _GRABAPI_GRABBERTASK_H_
#define _GRABAPI_GRABBERTASK_H_

#include <yat/threading/Task.h>
#include <yat/utils/Signal.h>
#include <IGrabber.h>
#include <deque>

namespace isl
{
  struct IMovieWriter;
  class  Exception;
}

namespace Tango
{
  class Device_3Impl;
}

namespace GrabAPI
{

#define kMSG_OPEN            ( yat::FIRST_USER_MSG      )
#define kMSG_CLOSE           ( yat::FIRST_USER_MSG + 1  )
#define kMSG_START           ( yat::FIRST_USER_MSG + 2  )
#define kMSG_STOP            ( yat::FIRST_USER_MSG + 3  )
#define kMSG_SNAP            ( yat::FIRST_USER_MSG + 4  )
#define kMSG_START_RECORDING ( yat::FIRST_USER_MSG + 5  )
#define kMSG_STOP_RECORDING  ( yat::FIRST_USER_MSG + 6  )
#define kMSG_SET_ROI         ( yat::FIRST_USER_MSG + 7  )
#define kMSG_RESET_ROI       ( yat::FIRST_USER_MSG + 8  )
#define kMSG_SAVE_SETTING    ( yat::FIRST_USER_MSG + 9  )
#define kMSG_RESET_CAMERA    ( yat::FIRST_USER_MSG + 10 )

struct RecordMovieConfig
{
  //- format : "MPG", "AVI", "JPG", "PNG, "TIF", "BMP", "DIB", "PGM", "RAS", "TXT"
  std::string format;
  //- file_basename contains the full path + the basename of the file(s) to save
  std::string file_basename;
  //- the duration of the movie in seconds
  double duration_s;
};


class SharedImage : private yat::SharedObject
{
public:

  /**
   * Constructor
   */
  SharedImage(GrabAPI::Image*);

  /**
   * Destructor
   */
  ~SharedImage( void );

  /**
   * Duplicate (shallow copy) this shared object 
   */
  SharedImage * duplicate (void);

  /**
   * Release this shared object 
   */
  void release (void);

  /**
   * Access the underlying GrabAPI::Image pointer
   */
  const GrabAPI::Image* get_image() const;

private:
  //! the managed object
  GrabAPI::Image* image;

  //! not implemented cpy ctor
  SharedImage(const SharedImage&);
  
  //! not implemented assignment op
  SharedImage& operator= (const SharedImage&);
};

class FrameRateComputer
{
public:
  FrameRateComputer();

  void notify_new_image( void );

  double get_frame_rate( void );

  yat_uint32_t get_image_counter ( void );

  void reset( void );

private:
  double frame_rate;
  std::deque<double> rep;
  yat::Timestamp init_tv;
  yat_uint32_t image_counter;
};



class ISL2YATException : public yat::Exception
{
public:
  ISL2YATException(const isl::Exception&);
};


struct GrabberTaskInit
{
  Tango::Device_3Impl* device;
  IGrabber* grabber;
  bool      auto_start;
  bool      auto_open;
};


class GrabberTask : public yat::Task
{
public:
  typedef yat::Signal<const GrabAPI::SharedImage*, yat::Mutex> SignalType;
  typedef SignalType::Slot SlotType;

  //- ctor
  GrabberTask (SignalType & sig);

  //- dtor
  virtual ~GrabberTask (void);

  //- get state and status in a thread safe manner
  void get_state_status(GrabberState& state, std::string& status);
  
  
  void set_state_status(GrabberState state);

  //- get the last available image
  GrabAPI::SharedImage* get_last_image( void );

  ROI get_roi( void ) throw (yat::Exception);

  void get_resulting_frame_rate( yat::Any& )
    throw (yat::Exception);

  void get_image_counter( yat::Any& )
    throw (yat::Exception);

  bool is_saving_movie( void );
  bool is_camera_present( void );

  std::string movie_remaining_time( void );

  int16_t get_bit_depth() const;

protected:
  //- handle_message
  virtual void handle_message (yat::Message& msg)
    throw (yat::Exception);

private:
  void image_callback(Image* new_image);

  //! INIT msg handler
  void on_init(yat::Message& msg)
    throw (yat::Exception);

  //! EXIT msg handler
  void on_exit()
    throw (yat::Exception);

  //! OPEN msg handler
  void on_open()
    throw (yat::Exception);

  //! CLOSE msg handler
  void on_close()
    throw (yat::Exception);

  //! START msg handler
  void on_start()
    throw (yat::Exception);

  //! STOP msg handler
  void on_stop()
    throw (yat::Exception);
  
  //! SNAP msg handler
  void on_snap()
    throw (yat::Exception);

  //! START_RECORDING msg handler
  void on_start_recording(yat::Message& msg)
    throw (yat::Exception);

  //! STOP_RECORDING msg handler
  void on_stop_recording()
    throw (yat::Exception);

  //! SET_ROI msg handler
  void on_set_roi(yat::Message& msg)
    throw (yat::Exception);

  //! RESET_ROI msg handler
  void on_reset_roi()
    throw (yat::Exception);

  //! SAVE_SETTINGS msg handler
  void on_save_settings()
    throw (yat::Exception);

  //! RESET_CAMERA msg handler
  void on_reset_camera()
    throw (yat::Exception);

  void release_grabber( void );

private:
  Tango::Device_3Impl* device;

  yat::Mutex   state_status_mutex;
  std::string  last_error_desc;

  GrabAPI::IGrabber*    grabber;
  GrabAPI::SharedImage* last_image;

  GrabAPI::ROI          current_roi;

  //FIXME: Perhaps we should remove this because it's split out to another object
//  isl::IMovieWriter* movie_writer;
//  RecordMovieConfig  movie_config;
//  yat::Timestamp     movie_start;
//  yat::Timestamp     movie_end;

  FrameRateComputer fps_computer;
  SignalType & image_available_observer_sig;
};

} // namespace

#if defined (YAT_INLINE_IMPL)
# include "GrabberTask.i"
#endif

#endif
