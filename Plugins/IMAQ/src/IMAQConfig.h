/*!
 * \file
 * \brief    Declaration of IMAQ configuration classes
 * \author   Julien Malik - Synchrotron SOLEIL
 */
#ifndef _IMAQCONFIG_H_
#define _IMAQCONFIG_H_

#include <IMAQGrabber.h>
#include <niimaq.h>

namespace GrabAPI
{
  const double MIN_BLACK_REF_LEVEL  = 0.0;
  const double MAX_BLACK_REF_LEVEL  = 0.5;

  const double MIN_WHITE_REF_LEVEL  = 0.0;
  const double MAX_WHITE_REF_LEVEL  = 1.4;

  const double MIN_SHUT_DELAY_MS    = 0.00004;
  const double MAX_SHUT_DELAY_MS    = 500.;

  const double MIN_EXPOSURE_TIME_MS = 0.00804;
  const double MAX_EXPOSURE_TIME_MS = 250.008;

  const double MIN_FRAME_RATE = 1.0;
  const double MAX_FRAME_RATE = 30.0;


  //- Constants dependent on the installation of the hardware system :
  //- they cannot change, they are not 'configurable'
  //- This reflects how camera signals are wired at Soleil
  const uInt32 TIMEBASE                  = PULSE_TIMEBASE_50MHZ;

  const uInt32 EXTERNAL_TRIGGER_LINEIN   = IMG_EXT_TRIG0;
  const uInt32 EXTERNAL_TRIGGER_POLARITY = IMG_TRIG_POLAR_ACTIVEH;
 
  const uInt32 SHUTTER_LINEOUT           = IMG_EXT_TRIG2;
  const uInt32 SHUTTER_POLARITY          = IMG_PULSE_POLAR_ACTIVEH;

  const uInt32 VSYNC_LINEOUT             = IMG_EXT_TRIG3;
  const uInt32 VSYNC_POLARITY            = IMG_PULSE_POLAR_ACTIVEL;
  const double VSYNC_WIDTH_MS            = 0.2;
  const double VSYNC_DELAY_MS            = 0.2;

  const double MIN_DEAD_TIME_MS          = 30;
  
  const long   NUM_CHANNELS = 4;
  const int    NUM_BUFFERS  = 4;

  typedef enum Mode
  {
    MODE_CONTINUOUS = 0,
    MODE_HSYNC_TRIGGER,
    MODE_EXTERNAL_TRIGGER,
    MODE_SOFTWARE_TRIGGER
  };

  struct PulseConfig
  {
    uInt32  input_line,
            input_polarity,
            output_line,
            output_polarity,
            mode,
            delay,
            width;
  };

  struct ChannelConfig
  {
    ChannelConfig();

    std::string   interface_name;
    long   channel;
    Mode   mode;
    double exposure_time_ms;
    double frame_rate;
    double shutter_delay_ms;
    double white_ref_level;
    double black_ref_level;
    ROI    roi;
  };

  template <typename T>
  bool is_in_range( T value, T min, T max )
  {
    return value >= min && value <= max;
  }

} // namespace



#endif
