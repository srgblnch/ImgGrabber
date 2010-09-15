/*!
 * \file
 * \brief    Inlined code for GrabberTask class
 * \author   Julien Malik - Synchrotron SOLEIL
 */

namespace GrabAPI
{


  YAT_INLINE SharedImage* SharedImage::duplicate(void)
  {
    return reinterpret_cast < SharedImage* >(SharedObject::duplicate ());
  }
  
  YAT_INLINE void SharedImage::release()
  {
    SharedObject::release ();
  }

  YAT_INLINE const GrabAPI::Image* SharedImage::get_image() const
  {
    return this->image;
  }


} // namespace
