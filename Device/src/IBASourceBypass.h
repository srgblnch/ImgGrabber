
#pragma once

#include "IGrabber.h"
#include <IBASource.h>
#include <string>

namespace ImgGrabber_ns
{
  class ImgGrabber;
}

class IBASourceBypass : public IBASource
{
public:
  typedef ImgGrabber_ns::ImgGrabber ImgGrabber;
private:
  ImgGrabber* image_grabber_;
public:

  IBASourceBypass(ImgGrabber* imggrb);
  ~IBASourceBypass();

  virtual void get_image(ImageAndInfo & imginf) throw (yat::Exception);
  virtual void set_callback_attribute(const std::string & /*name*/) {};

  void on_new_image_available(const GrabAPI::SharedImage* simage);

  //inline void on_new_image_available(isl::Image* image)
  //{ this->process(image); }

  static isl::Image* convert_image_grabber_to_isl(const GrabAPI::Image* gimage)  throw (yat::Exception);
};


class IBASourceFactoryBypass : public IBASourceFactory
{
  static IBASourceFactoryBypass s_singleton;

protected:
  virtual IBASource* create_object(const std::string & name);

public:
  static void register_factory()
  { IBASourceFactory::register_factory(&IBASourceFactoryBypass::s_singleton); }
};
