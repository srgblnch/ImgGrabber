/*!
 * \file
 * \brief    
 * \author   N. Leclercq, J. Malik - Synchrotron SOLEIL
 */

#ifndef _GRABBERATTR_H_
#define _GRABBERATTR_H_

// ============================================================================
// DEPENDENCIES
// ============================================================================
#include <tango.h>
#include <yat/plugin/PlugInTypes.h>
#include <yat/Portability.h>
#include <yat4tango/CommonHeader.h>
#include <yat4tango/ExceptionHelper.h>

namespace GrabAPI
{
  struct DeviceAttr
  {
    virtual void register_device( Tango::DeviceImpl *, yat::PlugInAttrInfo ) = 0;
  };


  class GrabberAttrManager
  {
  public:
    static GrabberAttrManager& instance();

    void register_attr( Tango::DeviceImpl *, yat::PlugInAttrInfo );

  private:
    GrabberAttrManager();
    void remove_attributes();
    std::map <std::string, DeviceAttr*> dyn_attr_;
  };


  template <typename T>
  class GrabberAttrT : public Tango::Attr, public DeviceAttr
  {
  public:
    GrabberAttrT(yat::PlugInAttrInfo info);

    ~GrabberAttrT(void);

  public: // Tango::Attr implementation
	  virtual void read(Tango::DeviceImpl *dev, Tango::Attribute &att);

	  virtual void write(Tango::DeviceImpl *dev, Tango::WAttribute &att);
	  virtual void memorize_attribute(std::string devName, std::string attrName, T write_value);

    virtual bool is_allowed (Tango::DeviceImpl *dev, Tango::AttReqType ty);


  public:
    virtual void register_device( Tango::DeviceImpl *, yat::PlugInAttrInfo );

  private:
    //- one struct of these kind for each device
    //- allows to have several device in the same instance, with the same attributes names
    struct DeviceAttrInfo
    {
      yat::PlugInAttrInfo info;
      yat::Any read_value;
    };

    std::map< Tango::DeviceImpl *, DeviceAttrInfo > callbacks_;
  };


  template <>
  class GrabberAttrT<std::string> :public Tango::Attr, public DeviceAttr
  {
  public:
    GrabberAttrT(yat::PlugInAttrInfo info);

    ~GrabberAttrT(void);

  public:
	  virtual void read(Tango::DeviceImpl *dev, Tango::Attribute &att);

	  virtual void write(Tango::DeviceImpl *dev, Tango::WAttribute &att);
	  virtual void memorize_attribute(std::string devName, std::string attrName, char* write_value);

    virtual bool is_allowed (Tango::DeviceImpl *dev, Tango::AttReqType ty);

  public:
    virtual void register_device( Tango::DeviceImpl *, yat::PlugInAttrInfo );

  private:
    //- one struct of these kind for each device
    //- allows to have several device in the same instance, with the same attributes names
    struct DeviceAttrInfo
    {
      yat::PlugInAttrInfo info;
      yat::Any read_value;
      char * read_ptr;
    };

    std::map<Tango::DeviceImpl *, DeviceAttrInfo> callbacks_;
  };

  //- this class converts (at compile time) a supported plugin 
  //- type to its Tango equivalent descriptor
  template <typename T> struct TangoType
  {
    enum { Value };
  };

# define MAP_TO_TANGO_TYPE( plugin_type, tango_type ) \
  template <> struct TangoType<plugin_type> \
  { \
    enum { Value = tango_type }; \
  };

  MAP_TO_TANGO_TYPE(bool, Tango::DEV_BOOLEAN)
  MAP_TO_TANGO_TYPE(yat_uint8_t, Tango::DEV_UCHAR)
  MAP_TO_TANGO_TYPE(yat_int16_t, Tango::DEV_SHORT)
  MAP_TO_TANGO_TYPE(yat_uint16_t, Tango::DEV_USHORT)
  MAP_TO_TANGO_TYPE(yat_int32_t, Tango::DEV_LONG)
  MAP_TO_TANGO_TYPE(yat_uint32_t, Tango::DEV_ULONG)
  MAP_TO_TANGO_TYPE(yat_int64_t, Tango::DEV_LONG64)
  MAP_TO_TANGO_TYPE(yat_uint64_t, Tango::DEV_ULONG64)
  MAP_TO_TANGO_TYPE(float, Tango::DEV_FLOAT)
  MAP_TO_TANGO_TYPE(double, Tango::DEV_DOUBLE)
  MAP_TO_TANGO_TYPE(std::string, Tango::DEV_STRING)

  struct TangoWriteType
  {
    Tango::AttrWriteType operator()( int plugin_write_type );
  };

} // namespace

#include "GrabberAttr.tpp"

#endif
