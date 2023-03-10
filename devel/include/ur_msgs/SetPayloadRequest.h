// Generated by gencpp from file ur_msgs/SetPayloadRequest.msg
// DO NOT EDIT!


#ifndef UR_MSGS_MESSAGE_SETPAYLOADREQUEST_H
#define UR_MSGS_MESSAGE_SETPAYLOADREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>

namespace ur_msgs
{
template <class ContainerAllocator>
struct SetPayloadRequest_
{
  typedef SetPayloadRequest_<ContainerAllocator> Type;

  SetPayloadRequest_()
    : mass(0.0)
    , center_of_gravity()  {
    }
  SetPayloadRequest_(const ContainerAllocator& _alloc)
    : mass(0.0)
    , center_of_gravity(_alloc)  {
  (void)_alloc;
    }



   typedef float _mass_type;
  _mass_type mass;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _center_of_gravity_type;
  _center_of_gravity_type center_of_gravity;





  typedef boost::shared_ptr< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetPayloadRequest_

typedef ::ur_msgs::SetPayloadRequest_<std::allocator<void> > SetPayloadRequest;

typedef boost::shared_ptr< ::ur_msgs::SetPayloadRequest > SetPayloadRequestPtr;
typedef boost::shared_ptr< ::ur_msgs::SetPayloadRequest const> SetPayloadRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_msgs::SetPayloadRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ur_msgs::SetPayloadRequest_<ContainerAllocator1> & lhs, const ::ur_msgs::SetPayloadRequest_<ContainerAllocator2> & rhs)
{
  return lhs.mass == rhs.mass &&
    lhs.center_of_gravity == rhs.center_of_gravity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ur_msgs::SetPayloadRequest_<ContainerAllocator1> & lhs, const ::ur_msgs::SetPayloadRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ur_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6a2cd594b640cc49946d268b22a837bd";
  }

  static const char* value(const ::ur_msgs::SetPayloadRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6a2cd594b640cc49ULL;
  static const uint64_t static_value2 = 0x946d268b22a837bdULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_msgs/SetPayloadRequest";
  }

  static const char* value(const ::ur_msgs::SetPayloadRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 mass\n"
"geometry_msgs/Vector3 center_of_gravity\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::ur_msgs::SetPayloadRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mass);
      stream.next(m.center_of_gravity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPayloadRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_msgs::SetPayloadRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_msgs::SetPayloadRequest_<ContainerAllocator>& v)
  {
    s << indent << "mass: ";
    Printer<float>::stream(s, indent + "  ", v.mass);
    s << indent << "center_of_gravity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.center_of_gravity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_MSGS_MESSAGE_SETPAYLOADREQUEST_H
