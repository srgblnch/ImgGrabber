
#include "ImgGrabber.h"
#include "GrabberTask.h"
#include "IBASourceBypass.h"
#include "isl/Image.h"
#include "isl/Exception.h"
#include <isl/ErrorHandler.h>
#include <cassert>

IBASourceBypass::IBASourceBypass(ImgGrabber_ns::ImgGrabber* imggrb)
{
  assert(imggrb);

  this->image_grabber_ = imggrb;

  this->image_grabber_->add_image_available_observer(ImgGrabber_ns::ImgGrabber::SlotType::instanciate(*this, &IBASourceBypass::on_new_image_available));
}

IBASourceBypass::~IBASourceBypass()
{
  this->image_grabber_->del_image_available_observer(ImgGrabber_ns::ImgGrabber::SlotType::instanciate(*this, &IBASourceBypass::on_new_image_available));
}

/*virtual*/
void IBASourceBypass::get_image(ImageAndInfo & imginf) throw (yat::Exception)
{ 
  if (!this->image_grabber_->is_Image_allowed(Tango::READ_REQ))
    THROW_YAT_ERROR("ATTR_NOT_ALLOWED",
                    "Image attribute cannot be read in this state.",
                    "IBASourceBypass::get_image");

  GrabAPI::SharedImage* si = 0;
  // From now on: act as if it was a Tango request:
  {
    Tango::AutoTangoMonitor guard(this->image_grabber_);

    // Ask for the attributes to be updated...
    std::vector<long> attr_list; /// @todo fill attr_list...
    this->image_grabber_->read_attr_hardware(attr_list);

    // Get the last image
    si = this->image_grabber_->get_last_image();
  }

  const GrabAPI::Image* gimage = si->get_image();
  if (gimage == NULL) {
    SAFE_RELEASE(si);
    THROW_YAT_ERROR("EMPTY_IMAGE",
                    "The image is empty.",
                    "IBASourceBypass::get_image");
  }

  // So we got the shared image. Now copy it to an image
  // in the format expected by ImageBeamAnalyzer
  try {
    imginf.bit_depth = gimage->bit_depth;
    imginf.image = this->convert_image_grabber_to_isl(gimage);
  } catch (yat::Exception &) {
    SAFE_RELEASE(si);
    throw;
  } catch (...) {
    SAFE_RELEASE(si);
    THROW_YAT_ERROR(  "UNKNOWN_ERROR",
                      "Unknown problem when converting ImgGrabber image to isl::Image",
                      "IBASourceBypass::get_image" );
  }

  SAFE_RELEASE(si);
}

/*static*/
isl::Image* IBASourceBypass::convert_image_grabber_to_isl(const GrabAPI::Image* gimage) throw (yat::Exception)
{
  try {
    isl::Image* image = new isl::Image(gimage->width(), gimage->height(), isl::ISL_STORAGE_USHORT);
    image->unserialize(gimage->base());
    return image;
  } catch(isl::Exception & ex) {
    GrabAPI::ISL2YATException yat_exc(ex);
    isl::ErrorHandler::reset();
    RETHROW_YAT_ERROR(yat_exc,
                      "SOFTWARE_FAILURE",
                      "Unable to convert from GrabAPI::Image to isl::Image.",
                      "IBASourceBypass::convert_image_grabber_to_isl.");
  } catch(std::bad_alloc &) {
    THROW_YAT_ERROR("OUT_OF_MEMORY",
                    "Allocation of isl::Image failed [std::bad_alloc]",
                    "IBASourceBypass::convert_image_grabber_to_isl");
  } catch(...) {
     THROW_YAT_ERROR("UNKNOWN_ERROR",
                    "Unable to convert from GrabAPI::Image to isl::Image. [Unknown exception caught]",
                    "IBASourceBypass::convert_image_grabber_to_isl");
  }
}

void IBASourceBypass::on_new_image_available(const GrabAPI::SharedImage* simage)
{
  // So we got the shared image. Now copy it to an image
  // in the format expected by ImageBeamAnalyzer

  if (!this->observer_registered())
    return;

  const GrabAPI::Image* gimage = simage->get_image();

  ImageAndInfo imginf(  this->convert_image_grabber_to_isl(gimage),
                        gimage->bit_depth );
  this->process(imginf);
}

/*static*/ IBASourceFactoryBypass IBASourceFactoryBypass::s_singleton;

/*virtual*/ IBASource* IBASourceFactoryBypass::create_object(const std::string & name)
{
  Tango::DeviceImpl* dev = 0;
  try {
    // If in the device is not served by one device pattern implemented
    // in this process, it throws DevFailed
    dev = Tango::Util::instance()->Tango::Util::get_device_by_name(name);
  } catch (...) {
    // try to create using next factory instead
    return this->create_next(name);
  }

  if ( dev->get_device_class()->get_name() == "ImgGrabber" ) {
    ImgGrabber_ns::ImgGrabber* imggrb = static_cast<ImgGrabber_ns::ImgGrabber*>(dev);
    return new IBASourceBypass(imggrb);
  }
	
  // try to create using next factory instead
  return this->create_next(name);
}
