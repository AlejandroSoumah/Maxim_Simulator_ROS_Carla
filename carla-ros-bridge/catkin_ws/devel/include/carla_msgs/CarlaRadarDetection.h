// Generated by gencpp from file carla_msgs/CarlaRadarDetection.msg
// DO NOT EDIT!


#ifndef CARLA_MSGS_MESSAGE_CARLARADARDETECTION_H
#define CARLA_MSGS_MESSAGE_CARLARADARDETECTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace carla_msgs
{
template <class ContainerAllocator>
struct CarlaRadarDetection_
{
  typedef CarlaRadarDetection_<ContainerAllocator> Type;

  CarlaRadarDetection_()
    : altitude(0.0)
    , azimuth(0.0)
    , depth(0.0)
    , velocity(0.0)  {
    }
  CarlaRadarDetection_(const ContainerAllocator& _alloc)
    : altitude(0.0)
    , azimuth(0.0)
    , depth(0.0)
    , velocity(0.0)  {
  (void)_alloc;
    }



   typedef float _altitude_type;
  _altitude_type altitude;

   typedef float _azimuth_type;
  _azimuth_type azimuth;

   typedef float _depth_type;
  _depth_type depth;

   typedef float _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> const> ConstPtr;

}; // struct CarlaRadarDetection_

typedef ::carla_msgs::CarlaRadarDetection_<std::allocator<void> > CarlaRadarDetection;

typedef boost::shared_ptr< ::carla_msgs::CarlaRadarDetection > CarlaRadarDetectionPtr;
typedef boost::shared_ptr< ::carla_msgs::CarlaRadarDetection const> CarlaRadarDetectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator2> & rhs)
{
  return lhs.altitude == rhs.altitude &&
    lhs.azimuth == rhs.azimuth &&
    lhs.depth == rhs.depth &&
    lhs.velocity == rhs.velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace carla_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a3ece6192cae1ab73b9842e20a38284a";
  }

  static const char* value(const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa3ece6192cae1ab7ULL;
  static const uint64_t static_value2 = 0x3b9842e20a38284aULL;
};

template<class ContainerAllocator>
struct DataType< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "carla_msgs/CarlaRadarDetection";
  }

  static const char* value(const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Copyright (c) 2020 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"\n"
"float32 altitude\n"
"float32 azimuth\n"
"float32 depth\n"
"float32 velocity\n"
;
  }

  static const char* value(const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.altitude);
      stream.next(m.azimuth);
      stream.next(m.depth);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CarlaRadarDetection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carla_msgs::CarlaRadarDetection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::carla_msgs::CarlaRadarDetection_<ContainerAllocator>& v)
  {
    s << indent << "altitude: ";
    Printer<float>::stream(s, indent + "  ", v.altitude);
    s << indent << "azimuth: ";
    Printer<float>::stream(s, indent + "  ", v.azimuth);
    s << indent << "depth: ";
    Printer<float>::stream(s, indent + "  ", v.depth);
    s << indent << "velocity: ";
    Printer<float>::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARLA_MSGS_MESSAGE_CARLARADARDETECTION_H
