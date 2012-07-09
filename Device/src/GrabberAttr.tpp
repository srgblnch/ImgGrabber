/*!
 * \file
 * \brief    
 * \author   N. Leclercq, J. Malik - Synchrotron SOLEIL
 */
#ifndef _GRABBERATTR_TPP_
#define _GRABBERATTR_TPP_



// ============================================================================
// DEPENDENCIES
// ============================================================================
#include "GrabberAttr.h"


namespace GrabAPI
{

  template <typename T>
  GrabberAttrT<T>::GrabberAttrT(yat::PlugInAttrInfo info)
    : Tango::Attr(info.name.c_str(),
                  TangoType<T>::Value,
                  TangoWriteType()(info.write_type))
  {
    Tango::UserDefaultAttrProp	prop;
    prop.set_label        (info.label.c_str());
    prop.set_unit         (info.unit.c_str());
    prop.set_standard_unit(info.unit.c_str());
    prop.set_display_unit (info.unit.c_str());
    prop.set_description  (info.desc.c_str());
    prop.set_format       (info.display_format.c_str());
    this->set_default_properties(prop);
  }

  template <typename T>
  GrabberAttrT<T>::~GrabberAttrT(void)
  {
  }

  template <typename T>
  void GrabberAttrT<T>::read(Tango::DeviceImpl *dev, Tango::Attribute &att)
  {
    //- retrieve the value from the plugin :
    try
    {
      this->callbacks_[dev].info.get_cb( this->callbacks_[dev].read_value );
      if (this->callbacks_[dev].read_value.empty())
        THROW_DEVFAILED("NO_VALUE",
                        "No value has been assigned to the container",
                        "PlugInAttr<T>::read");
    }
    catch( yat::Exception& ex )
    {
      yat4tango::YATDevFailed df(ex);
      RETHROW_DEVFAILED(df,
                        "SOFTWARE_FAILURE",
                        "Error while reading a plugin attribute",
                        "PlugInAttr<T>::read");
    }
    catch( ... )
    {
      THROW_DEVFAILED("UNKNWON_ERROR",
                      "Unknwon error while reading a plugin attribute",
                      "PlugInAttr<T>::read");
    }

    //- assign it to the Tango::Attribute
    att.set_value( yat::any_cast<T>(&this->callbacks_[dev].read_value) );
  }



  template <typename T>
  void GrabberAttrT<T>::write(Tango::DeviceImpl* dev, Tango::WAttribute &att)
  {
    T write_value;
    att.get_write_value(write_value);
    yat::Any container(write_value);

    try
    {
      this->callbacks_[dev].info.set_cb(container);
      this->memorize_attribute(dev->name(),att.get_name(),write_value);
    }
    catch( yat::Exception& ex )
    {
      yat4tango::YATDevFailed df(ex);
      RETHROW_DEVFAILED(df,
                        "SOFTWARE_FAILURE",
                        "Error while writing a plugin attribute",
                        "PlugInAttr<T>::read");
    }
    catch( ... )
    {
      THROW_DEVFAILED("UNKNWON_ERROR",
                      "Unknwon error while writing a plugin attribute",
                      "PlugInAttr<T>::read");
    }
  }

  template <typename T>
  void GrabberAttrT<T>::memorize_attribute(std::string devName,
                                           std::string attrName,
                                           T write_value)
  {
    try
    {
      std::cout << "memorize_attribute(" << attrName << "); ";
      Tango::Database *db = new Tango::Database();
      Tango::DbData db_data;
      Tango::DbDatum dev_prop(attrName);
      dev_prop << write_value;
      db_data.push_back(dev_prop);
      db->put_device_property(devName, db_data);
      std::cout << "value=" << write_value << ";" << std::endl;
    }
    catch(...)
    {
        std::cout << "Malo!" << std::endl;
        THROW_DEVFAILED("UNKNOWN_ERROR",
                        "Tango error during memorize attribute ",
                        "GrabberAttrT<std::string>::memorize_attribute");
    }
    
  }

  template <typename T>
  bool GrabberAttrT<T>::is_allowed (Tango::DeviceImpl *, Tango::AttReqType)
  {
    return (true);
  }

  template <typename T>
  void GrabberAttrT<T>::register_device( Tango::DeviceImpl* dev, yat::PlugInAttrInfo info )
  {
    callbacks_[ dev ].info = info;
  }






}

#endif
