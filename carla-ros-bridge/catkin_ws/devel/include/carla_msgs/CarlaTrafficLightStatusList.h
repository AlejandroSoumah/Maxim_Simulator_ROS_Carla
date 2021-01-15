// Generated by gencpp from file carla_msgs/CarlaTrafficLightStatusList.msg
// DO NOT EDIT!


#ifndef CARLA_MSGS_MESSAGE_CARLATRAFFICLIGHTSTATUSLIST_H
#define CARLA_MSGS_MESSAGE_CARLATRAFFICLIGHTSTATUSLIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <carla_msgs/CarlaTrafficLightStatus.h>

namespace carla_msgs
{
template <class ContainerAllocator>
struct CarlaTrafficLightStatusList_
{
  typedef CarlaTrafficLightStatusList_<ContainerAllocator> Type;

  CarlaTrafficLightStatusList_()
    : traffic_lights()  {
    }
  CarlaTrafficLightStatusList_(const ContainerAllocator& _alloc)
    : traffic_lights(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::carla_msgs::CarlaTrafficLightStatus_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::carla_msgs::CarlaTrafficLightStatus_<ContainerAllocator> >::other >  _traffic_lights_type;
  _traffic_lights_type traffic_lights;





  typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> const> ConstPtr;

}; // struct CarlaTrafficLightStatusList_

typedef ::carla_msgs::CarlaTrafficLightStatusList_<std::allocator<void> > CarlaTrafficLightStatusList;

typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightStatusList > CarlaTrafficLightStatusListPtr;
typedef boost::shared_ptr< ::carla_msgs::CarlaTrafficLightStatusList const> CarlaTrafficLightStatusListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator2> & rhs)
{
  return lhs.traffic_lights == rhs.traffic_lights;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace carla_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "99769ac9a785c6de87ed86d7bb4a653a";
  }

  static const char* value(const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x99769ac9a785c6deULL;
  static const uint64_t static_value2 = 0x87ed86d7bb4a653aULL;
};

template<class ContainerAllocator>
struct DataType< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "carla_msgs/CarlaTrafficLightStatusList";
  }

  static const char* value(const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Copyright (c) 2020 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"CarlaTrafficLightStatus[] traffic_lights\n"
"\n"
"================================================================================\n"
"MSG: carla_msgs/CarlaTrafficLightStatus\n"
"#\n"
"# Copyright (c) 2020 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"uint32 id\n"
"\n"
"uint8 RED=0\n"
"uint8 YELLOW=1\n"
"uint8 GREEN=2\n"
"uint8 OFF=3\n"
"uint8 UNKNOWN=4\n"
"\n"
"uint8 state\n"
;
  }

  static const char* value(const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.traffic_lights);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CarlaTrafficLightStatusList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::carla_msgs::CarlaTrafficLightStatusList_<ContainerAllocator>& v)
  {
    s << indent << "traffic_lights[]" << std::endl;
    for (size_t i = 0; i < v.traffic_lights.size(); ++i)
    {
      s << indent << "  traffic_lights[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::carla_msgs::CarlaTrafficLightStatus_<ContainerAllocator> >::stream(s, indent + "    ", v.traffic_lights[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARLA_MSGS_MESSAGE_CARLATRAFFICLIGHTSTATUSLIST_H