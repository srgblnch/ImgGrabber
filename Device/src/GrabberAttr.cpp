// ============================================================================
//
// = CONTEXT
//    TANGO Project - ImgBeamAnalyzer DeviceServer - Data class
//
// = File
//    Data.cpp
//
// = AUTHOR
//    Julien Malik
//
// ============================================================================

// ============================================================================
// DEPENDENCIES
// ============================================================================
#include "GrabberAttr.h"

namespace GrabAPI
{

  Tango::AttrWriteType TangoWriteType::operator()(int plugin_write_type)
  {
    switch (plugin_write_type)
    {
    case yat::PlugInAttrWriteType::READ       : return Tango::READ;
    case yat::PlugInAttrWriteType::WRITE      : return Tango::WRITE;
    case yat::PlugInAttrWriteType::READ_WRITE : return Tango::READ_WRITE;
    default: THROW_DEVFAILED("SOFTWARE_FAILURE",
                             "Unsupported write type",
                             "write_type");
    }
    return Tango::READ; // keep compiler happy
  };

  GrabberAttrT<std::string>::GrabberAttrT(yat::PlugInAttrInfo info)
    : Tango::Attr( 
                   info.name.c_str(), 
                   TangoType<std::string>::Value, 
                   TangoWriteType()(info.write_type)
                  )
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

  GrabberAttrT<std::string>::~GrabberAttrT(void)
  {
  }

  void GrabberAttrT<std::string>::read(Tango::DeviceImpl* dev, Tango::Attribute &att)
  {
    //- retrieve the value from the plugin :
    try
    {
      this->callbacks_[dev].info.get_cb( this->callbacks_[dev].read_value );
      if (this->callbacks_[dev].read_value.empty())
        THROW_DEVFAILED("NO_VALUE",
                        "No value has been assigned to the container",
                        "PlugInAttr<std::string>::read");
      this->callbacks_[dev].read_ptr = const_cast<char*>( (yat::any_cast<std::string>(&this->callbacks_[dev].read_value))->c_str() );
      att.set_value( &this->callbacks_[dev].read_ptr );
    }
    catch( yat::Exception& ex )
    {
      yat4tango::YATDevFailed df(ex);
      RETHROW_DEVFAILED(df,
                        "SOFTWARE_FAILURE",
                        "Error while reading a plugin attribute",
                        "PlugInAttr<std::string>::read");
    }
    catch( ... )
    {
      THROW_DEVFAILED("UNKNWON_ERROR",
                      "Unknwon error while reading a plugin attribute",
                      "PlugInAttr<std::string>::read");
    }
  }

  void GrabberAttrT<std::string>::write(Tango::DeviceImpl* dev, Tango::WAttribute &att)
  {
    char* write_value;
    att.get_write_value(write_value);
    std::string write_value_string(write_value);
    yat::Any container(write_value_string);

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
                        "PlugInAttr<std::string>::write");
    }
    catch( ... )
    {
      THROW_DEVFAILED("UNKNWON_ERROR",
                      "Unknwon error while writing a plugin attribute",
                      "PlugInAttr<std::string>::write");
    }
  }

  void GrabberAttrT<std::string>::memorize_attribute(std::string devName,
                                                       std::string attrName,
                                                       char* write_value)
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

  bool GrabberAttrT<std::string>::is_allowed (Tango::DeviceImpl *, Tango::AttReqType)
  {
    return (true);
  }

  void GrabberAttrT<std::string>::register_device( Tango::DeviceImpl* dev, yat::PlugInAttrInfo info )
  {
    callbacks_[ dev ].info = info;
    callbacks_[ dev ].read_ptr = 0;
  }



  GrabberAttrManager::GrabberAttrManager()
  {
  }

  void GrabberAttrManager::register_attr( Tango::DeviceImpl* dev, yat::PlugInAttrInfo info )
  {
    //- check attribute does not already exist
    std::map <std::string, DeviceAttr*>::iterator it = this->dyn_attr_.find(info.name);
    if (it == this->dyn_attr_.end())
    {
      //- we must create it
      Tango::Attr* a  = 0;
      DeviceAttr*  da = 0;
      switch ( info.data_type )
      {
      case yat::PlugInDataType::BOOLEAN:
        {
          GrabberAttrT<bool>* gat = new GrabberAttrT<bool>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::UINT8:
        {
          GrabberAttrT<yat_uint8_t>* gat = new GrabberAttrT<yat_uint8_t>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::INT16:
        {
          GrabberAttrT<yat_int16_t>* gat = new GrabberAttrT<yat_int16_t>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::UINT16:
        {
          GrabberAttrT<yat_uint16_t>* gat = new GrabberAttrT<yat_uint16_t>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::INT32:
        {
          GrabberAttrT<yat_int32_t>* gat = new GrabberAttrT<yat_int32_t>(info);
          a = gat;  da = gat;
        }
        break;
	  case yat::PlugInDataType::UINT32:
        {
          GrabberAttrT<yat_uint32_t>* gat = new GrabberAttrT<yat_uint32_t>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::INT64:
        {
          GrabberAttrT<yat_int64_t>* gat = new GrabberAttrT<yat_int64_t>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::UINT64:
        {
          GrabberAttrT<yat_uint64_t>* gat = new GrabberAttrT<yat_uint64_t>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::FLOAT:
        {
          GrabberAttrT<float>* gat = new GrabberAttrT<float>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::DOUBLE:
        {
          GrabberAttrT<double>* gat = new GrabberAttrT<double>(info);
          a = gat;  da = gat;
        }
        break;
      case yat::PlugInDataType::STRING:
        {
          GrabberAttrT<std::string>* gat = new GrabberAttrT<std::string>(info);
          a = gat;  da = gat;
        }
        break;
      }
      //- add it to the device (and class) attribute list
      dev->add_attribute( a );
      this->dyn_attr_[ info.name ] = da;
    }

    //- in every case, register the device-specific info
    this->dyn_attr_[ info.name ]->register_device( dev, info );
  }

  void GrabberAttrManager::remove_attributes()
  {

  }

  GrabberAttrManager& GrabberAttrManager::instance()
  {
    static GrabberAttrManager unique_instance;
    return unique_instance;
  }



}
