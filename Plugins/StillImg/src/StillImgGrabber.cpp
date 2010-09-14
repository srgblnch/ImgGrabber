/*!
 * \file
 * \brief    Definition of StillImgGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */


#include "StillImgGrabber.h"

#include <memory>
#include <yat/plugin/PlugInSymbols.h>
#include <iostream>

EXPORT_SINGLECLASS_PLUGIN(GrabAPI::StillImgGrabber, GrabAPI::StillImgGrabberInfo);

namespace GrabAPI
{

  class ISL2YATException : public yat::Exception
  {
  public:
    ISL2YATException(const isl::Exception& ex)
    {
      const isl::ErrorList& isl_errors = ex.errors;
      for (size_t i = 0; i < isl_errors.size(); i++)
      {
        this->push_error( isl_errors[i].reason,
                          isl_errors[i].description,
                          isl_errors[i].origin);
      }
    }
  };



  // ============================================================================
  // StillImgGrabber::StillImgGrabber
  // ============================================================================
  StillImgGrabber::StillImgGrabber()
    : grab_thread(0),
      filename("not_defined"),
      image(0),
      frame_rate(10.0),
      bit_depth(0)
  {
  }

  // ============================================================================
  // StillImgGrabber::~StillImgGrabber
  // ============================================================================
  StillImgGrabber::~StillImgGrabber()
  {
    SAFE_DELETE_PTR(this->image);
  }

  // ============================================================================
  // StillImgGrabber::enumerate_attributes
  // ============================================================================
  void StillImgGrabber::enumerate_attributes( yat::PlugInAttrInfoList& list ) const
    throw (yat::Exception)
  {
    yat::PlugInAttrInfo attr_info;

    attr_info.name   = "Filename";
    attr_info.label  = "Filename";
    attr_info.desc   = "Path to the file to read";
    attr_info.unit   = " ";
    attr_info.display_format = " ";
    attr_info.data_type = yat::PlugInDataType::STRING;
    attr_info.write_type = yat::PlugInAttrWriteType::READ_WRITE;
    attr_info.set_cb = yat::SetAttrCB::instanciate( const_cast<StillImgGrabber&>(*this), &StillImgGrabber::set_filename );
    attr_info.get_cb = yat::GetAttrCB::instanciate( const_cast<StillImgGrabber&>(*this), &StillImgGrabber::get_filename );
    list.push_back(attr_info);
  }

  // ============================================================================
  // StillImgGrabber::enumerate_attributes
  // ============================================================================
  void StillImgGrabber::enumerate_properties( yat::PlugInPropInfos& /*prop_infos*/ ) const
    throw (yat::Exception)
  {

  }

  // ============================================================================
  // StillImgGrabber::set_properties
  // ============================================================================
  void StillImgGrabber::set_properties( yat::PlugInPropValues& /*prop_values*/ )
    throw (yat::Exception)
  {

  }



  // ============================================================================
  // StillImgGrabber::initialize
  // ============================================================================
  void StillImgGrabber::initialize()
    throw (yat::Exception)
  {
  }
  
  // ============================================================================
  // StillImgGrabber::uninitialize
  // ============================================================================
  void StillImgGrabber::uninitialize()
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::set_image_handler
  // ============================================================================
  void StillImgGrabber::set_image_handler(ImageHandlerCallback callback)
    throw (yat::Exception)
  {
    this->image_callback = callback;
  }

  // ============================================================================
  // StillImgGrabber::get_state
  // ============================================================================
  GrabberState StillImgGrabber::get_state( void ) const
  {
    return this->grab_thread ? RUNNING : OPEN;
  }

  // ============================================================================
  // StillImgGrabber::is_open
  // ============================================================================
  bool StillImgGrabber::is_open( void ) const
  {
    //- TODO
    return true;
  }

  // ============================================================================
  // StillImgGrabber::is_closed
  // ============================================================================
  bool StillImgGrabber::is_closed( void ) const
  {
    //- TODO
    return !this->is_open();
  }


  // ============================================================================
  // StillImgGrabber::is_running
  // ============================================================================
  bool StillImgGrabber::is_running( void ) const
  {
    return this->grab_thread != 0;
  }

  // ============================================================================
  // StillImgGrabber::open
  // ============================================================================
  void StillImgGrabber::open()
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::close
  // ============================================================================
  void StillImgGrabber::close()
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::start
  // ============================================================================
  void StillImgGrabber::start()
    throw (yat::Exception)
  {
    if (this->image == 0)
    {
      THROW_YAT_ERROR("SOFTWARE_FAILURE",
                      "No image has been read : specify a filename",
                      "StillImgGrabber::start");
    }

    try
    {
      size_t timeout_ms = static_cast<size_t>(1000.0 / this->frame_rate);
      this->grab_thread = new GrabThread(*this, timeout_ms);
      this->grab_thread->go();
    }
    catch(...)
    {
      THROW_YAT_ERROR("SOFTWARE_FAILURE",
                      "Error when starting the thread",
                      "StillImgGrabber::start");
    }
  }

  // ============================================================================
  // StillImgGrabber::stop
  // ============================================================================
  void StillImgGrabber::stop()
    throw (yat::Exception)
  {
    try
    {
      this->grab_thread->exit();
      this->grab_thread = 0;
    }
    catch(...)
    {
      THROW_YAT_ERROR("SOFTWARE_FAILURE",
                      "Error when stoping the acquisition thread",
                      "StillImgGrabber::start");
    }
  }



  // ============================================================================
  // StillImgGrabber::snap
  // ============================================================================
  void StillImgGrabber::snap()
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::set_roi
  // ============================================================================
  void StillImgGrabber::set_roi( ROI /*roi*/ )
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::get_roi
  // ============================================================================
  ROI StillImgGrabber::get_roi()
    throw (yat::Exception)
  {
    ROI r;
    return r;
  }

  // ============================================================================
  // StillImgGrabber::reset_roi
  // ============================================================================
  void StillImgGrabber::reset_roi()
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::get_settings
  // ============================================================================
  void StillImgGrabber::get_settings( yat::PlugInPropValues& /*prop_values*/ ) const
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::set_exposure_time
  // ============================================================================
  void StillImgGrabber::set_exposure_time( const yat::Any& )
    throw (yat::Exception)
  {
  }

  // ============================================================================
  // StillImgGrabber::get_exposure_time
  // ============================================================================
  void StillImgGrabber::get_exposure_time( yat::Any& container )
    throw (yat::Exception)
  {
    container = double(0.0);
  }

  // ============================================================================
  // StillImgGrabber::set_frame_rate
  // ============================================================================
  void StillImgGrabber::set_frame_rate( const yat::Any& container )
    throw (yat::Exception)
  {
    if (this->grab_thread == 0)
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Attribute not available in the current state",
                      "StillImgGrabber::set_frame_rate");
    }

    double contained = yat::any_cast<double>(container);
    size_t timeout_ms = static_cast<size_t>(1000.0 / contained);

    this->grab_thread->set_periodic_msg_period(timeout_ms);
    this->frame_rate = double(1000.0 / this->grab_thread->get_periodic_msg_period());
  }

  // ============================================================================
  // StillImgGrabber::get_frame_rate
  // ============================================================================
  void StillImgGrabber::get_frame_rate( yat::Any& container )
    throw (yat::Exception)
  {
    if (this->grab_thread == 0)
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Attribute not available in the current state",
                      "StillImgGrabber::set_frame_rate");
    }

    container = this->frame_rate;
  }
  // ============================================================================
  // StillImgGrabber::get_bit_depth
  // ============================================================================
  void StillImgGrabber::get_bit_depth( yat::Any& container )
    throw (yat::Exception)
  {
    if (!this->image)
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Attribute not available in the current state",
                      "StillImgGrabber::get_sensor_width");
    }

    container = this->bit_depth;
  }

  // ============================================================================
  // StillImgGrabber::get_sensor_width
  // ============================================================================
  void StillImgGrabber::get_sensor_width( yat::Any& container )
    throw (yat::Exception)
  {
    if (!this->image)
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Attribute not available in the current state",
                      "StillImgGrabber::get_sensor_width");
    }

    container = long( image->width() );
  }

  // ============================================================================
  // StillImgGrabber::get_sensor_height
  // ============================================================================
  void StillImgGrabber::get_sensor_height( yat::Any& container )
    throw (yat::Exception)
  {
    if (!this->image)
    {
      THROW_YAT_ERROR("ATTRIBUTE_UNAVAILABLE",
                      "Attribute not available in the current state",
                      "StillImgGrabber::get_sensor_height");
    }

    container = long( image->height() );
  }

  // ============================================================================
  // StillImgGrabber::set_filename
  // ============================================================================
  void StillImgGrabber::set_filename( const yat::Any& container )
    throw (yat::Exception)
  {
    std::string f = yat::any_cast<std::string>(container);
    
    try
    {
      //- try to load the image
      std::auto_ptr<isl::Image> isl_im( new isl::Image(f.c_str()) );
      this->bit_depth = isl_im->bit_per_pixel();
      isl_im->convert(isl::ISL_STORAGE_SHORT);

      GrabAPI::Image* old_im = this->image;
      GrabAPI::Image* new_im = new GrabAPI::Image(isl_im->width(),
                                                  isl_im->height());
      isl_im->serialize(new_im->base());

      this->image = new_im;
      SAFE_DELETE_PTR(old_im);

      //- ok, now store the new filename
      this->filename = f;
    }
    catch(isl::Exception& isl_ex)
    {
      ISL2YATException ex(isl_ex);
      RETHROW_YAT_ERROR(ex,
                        "SOFTWARE_FAILURE",
                        "Unable to load the specified file",
                        "StillImgGrabber::set_filename");
    }
    catch(...)
    {
      THROW_YAT_ERROR("UNKNWON_ERROR",
                      "Unable to load the specified file",
                      "StillImgGrabber::set_filename");
    }
  }

  // ============================================================================
  // StillImgGrabber::get_filename
  // ============================================================================
  void StillImgGrabber::get_filename( yat::Any& container)
    throw (yat::Exception)
  {
    container = this->filename;
  }

  // ============================================================================
  // GrabThread::GrabThread
  // ============================================================================
  GrabThread::GrabThread(StillImgGrabber& grabber,
                         size_t timeout_ms)
    : grabber_(grabber)
  {
    this->set_periodic_msg_period(timeout_ms);
    this->enable_periodic_msg(true);
  }


  // ============================================================================
  // GrabThread::~GrabThread
  // ============================================================================
  GrabThread::~GrabThread()
  {
  }

  // ============================================================================
  // GrabThread::handle_message
  // ============================================================================
  void GrabThread::handle_message (yat::Message& _msg)
    throw (yat::Exception)
  {
    switch (_msg.type())
	  {
    case yat::TASK_INIT:
      {

      }
      break;
    case yat::TASK_EXIT:
      {

      }
      break;
    case yat::TASK_PERIODIC:
      {
        GrabAPI::Image* im = new GrabAPI::Image(*this->grabber_.image);
        this->grabber_.image_callback(im);
      }
      break;
    }
  }
}
