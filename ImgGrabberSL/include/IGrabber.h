#ifndef _GRABAPI_IGRABBER_H
#define _GRABAPI_IGRABBER_H

#include <yat/plugin/IPlugInObjectWithAttr.h>
#include <yat/plugin/IPlugInInfo.h>
#include <yat/memory/DataBuffer.h>

namespace GrabAPI
{
  const std::string kIGrabberInterfaceName( "IGrabber" );

  //typedef yat::ImageBuffer<unsigned short> Image;
  struct Image : public yat::ImageBuffer<unsigned short>
  {
    long bit_depth;
    
    Image (size_t width = 0, size_t height = 0)
        throw (yat::Exception)
      : yat::ImageBuffer<unsigned short>(width, height)
    {}
    Image (size_t width, size_t height, unsigned short *base)
        throw (yat::Exception)
      : yat::ImageBuffer<unsigned short>(width, height, base)
    {}
    Image (const yat::ImageBuffer<unsigned short>& im)
        throw (yat::Exception)
      : yat::ImageBuffer<unsigned short>(im), bit_depth(0)
    {}
    Image (const Image& im)
        throw (yat::Exception)
      : yat::ImageBuffer<unsigned short>(im), bit_depth(im.bit_depth)
    {}
    Image (size_t width, size_t height, const yat::Buffer<unsigned short>& buf)
        throw (yat::Exception)
      : yat::ImageBuffer<unsigned short>(width, height, buf)
    {}
  };

  struct ROI
  {
    ROI(int _x = 0, int _y = 0, int _width = 0, int _height = 0)
      : x(_x), y(_y), width(_width), height(_height)
    {};

    bool is_empty()
    {
      return x == 0 && y == 0 && width == 0 && height == 0;
    }

    int x;
    int y;
    int width;
    int height;
  };

  typedef enum
  {
    OPEN,    //- hardware session is opened, but acquisition is not started
    CLOSE,   //- hardware session is released
    RUNNING, //- hardware session is opened, and acquisition is started
    FAULT,   //- a fatal error occured
    UNKNOWN  //- unknown state
  } GrabberState;

  YAT_DEFINE_CALLBACK( ImageHandlerCallback, GrabAPI::Image* );

  class IGrabber : public yat::IPlugInObjectWithAttr
  {
  public:
    //! Called at object initialization
    virtual void initialize( void )
      throw (yat::Exception) = 0;

    //! Called at object destruction
    virtual void uninitialize( void )
      throw (yat::Exception) = 0;

    //! Configure the callback called for new images
    virtual void set_image_handler( ImageHandlerCallback callback )
      throw (yat::Exception) = 0;

    /*******************************************************************************
     * STATE
     *******************************************************************************/

    //! Returns the current state
    virtual GrabberState get_state( void ) const = 0;

    virtual void set_state( GrabberState mystate)  = 0;
    
    //! Returns true if the hardware session is opened (OPEN and RUNNING state)
    virtual bool is_open( void ) const = 0;
    
    //! Returns true if the hardware session is closed (CLOSE state)
    virtual bool is_closed( void ) const = 0;

    //! Returns true if the acquisition is started (RUNNING state only)
    virtual bool is_running( void ) const = 0;


    /*******************************************************************************
     * COMMANDS
     *******************************************************************************/

    //! Open a session with the hardware
    virtual void open( void )
      throw (yat::Exception) = 0;

    //! Close any access to the hardware
    virtual void close( void )
      throw (yat::Exception) = 0;

    //! Start image acquisition
    virtual void start( void )
      throw (yat::Exception) = 0;

    //! Stop (Pause) image acquisition
    virtual void stop( void )
      throw (yat::Exception) = 0;

    //! Snap one image
    virtual void snap( void )
      throw (yat::Exception) = 0;

    //! Configure the ROI
    virtual void set_roi( ROI roi )
      throw (yat::Exception) = 0;

    //! Get the current ROI
    virtual ROI  get_roi( void )
      throw (yat::Exception) = 0;
    
    //! Configure the ROI to the full image
    virtual void reset_roi( void )
      throw (yat::Exception) = 0;

    virtual void get_settings( yat::PlugInPropValues& prop_values ) const
      throw (yat::Exception) = 0;

    /*******************************************************************************
     * ATTRIBUTES
     *******************************************************************************/

    //! Set the exposure time
    virtual void set_exposure_time( const yat::Any& )
      throw (yat::Exception) = 0;

    //! Get the exposure time
    virtual void get_exposure_time( yat::Any& )
      throw (yat::Exception) = 0;

    //! Set the frame rate
    virtual void set_frame_rate( const yat::Any& )
      throw (yat::Exception) = 0;

    //! Get the frame rate
    virtual void get_frame_rate( yat::Any& )
      throw (yat::Exception) = 0;

    //! Get the bit depth
    virtual void get_bit_depth( yat::Any& )
      throw (yat::Exception) = 0;

    //! Get the sensor width
    virtual void get_sensor_width( yat::Any& )
      throw (yat::Exception) = 0;

    //! Get the sensor height
    virtual void get_sensor_height( yat::Any& )
      throw (yat::Exception) = 0;
  };

}

#endif
