/*!
 * \file
 * \brief    Definition of IMAQGrabber class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

#include "IMAQGrabber.h"

namespace GrabAPI
{

  const std::string  DEFAULT_INTERFACE_NAME    = "undefined";
  const long         DEFAULT_CHANNEL           = 0;
  const double       DEFAULT_EXPOSURE_TIME_MS  = 50.0;
  const double       DEFAULT_FRAME_RATE        = 10.0;
  const double       DEFAULT_SHUTTER_DELAY_MS  = 0.00004;
  const double       DEFAULT_BLACK_REF_LEVEL   = 0.0;
  const double       DEFAULT_WHITE_REF_LEVEL   = 0.8;
  const Mode         DEFAULT_MODE              = MODE_HSYNC_TRIGGER;

  ChannelConfig::ChannelConfig()
    : interface_name(DEFAULT_INTERFACE_NAME),
      channel(DEFAULT_CHANNEL),
      mode(DEFAULT_MODE),
      exposure_time_ms(DEFAULT_EXPOSURE_TIME_MS),
      frame_rate(DEFAULT_FRAME_RATE),
      shutter_delay_ms(DEFAULT_SHUTTER_DELAY_MS),
      white_ref_level(DEFAULT_WHITE_REF_LEVEL),
      black_ref_level(DEFAULT_BLACK_REF_LEVEL)
  {
  }

}
