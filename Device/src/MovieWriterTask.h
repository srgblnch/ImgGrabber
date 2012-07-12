
#pragma once

#include "GrabberTask.h"

#include <yat/threading/Task.h>

namespace isl
{
  struct IMovieWriter;
  class Exception;
}

namespace GrabAPI {
  class MovieWriterTask : public yat::Task
  {
    isl::IMovieWriter* movie_writer;
    RecordMovieConfig  movie_config;
    yat::Timestamp     movie_start;
    yat::Timestamp     movie_end;
    
    bool               m_finishing;

    int32_t bit_depth;
  public:
    MovieWriterTask();
    ~MovieWriterTask();

    void handle_message(yat::Message& _msg)
    throw (yat::Exception);

    void on_init(yat::Message & _msg);
    void on_exit();
    void on_write(yat::Message & _msg);

void on_start_recording(yat::Message& _msg) throw (yat::Exception);
void start_recording(GrabAPI::RecordMovieConfig rmconf, int32_t bit_depth) ;
void on_stop_recording() throw (yat::Exception);
void write(const SharedImage* shared_image);
bool is_saving_movie( void ) const;
std::string movie_remaining_time( void ) const;


  };

}
