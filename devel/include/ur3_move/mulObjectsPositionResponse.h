// Generated by gencpp from file ur3_move/mulObjectsPositionResponse.msg
// DO NOT EDIT!


#ifndef UR3_MOVE_MESSAGE_MULOBJECTSPOSITIONRESPONSE_H
#define UR3_MOVE_MESSAGE_MULOBJECTSPOSITIONRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace ur3_move
{
template <class ContainerAllocator>
struct mulObjectsPositionResponse_
{
  typedef mulObjectsPositionResponse_<ContainerAllocator> Type;

  mulObjectsPositionResponse_()
    : targets_pose()
    , angles()  {
    }
  mulObjectsPositionResponse_(const ContainerAllocator& _alloc)
    : targets_pose(_alloc)
    , angles(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::Pose_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Pose_<ContainerAllocator> >> _targets_pose_type;
  _targets_pose_type targets_pose;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _angles_type;
  _angles_type angles;





  typedef boost::shared_ptr< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct mulObjectsPositionResponse_

typedef ::ur3_move::mulObjectsPositionResponse_<std::allocator<void> > mulObjectsPositionResponse;

typedef boost::shared_ptr< ::ur3_move::mulObjectsPositionResponse > mulObjectsPositionResponsePtr;
typedef boost::shared_ptr< ::ur3_move::mulObjectsPositionResponse const> mulObjectsPositionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator1> & lhs, const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.targets_pose == rhs.targets_pose &&
    lhs.angles == rhs.angles;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator1> & lhs, const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ur3_move

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e6e6dc5574cc86cf8bf5dab8f005d505";
  }

  static const char* value(const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe6e6dc5574cc86cfULL;
  static const uint64_t static_value2 = 0x8bf5dab8f005d505ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur3_move/mulObjectsPositionResponse";
  }

  static const char* value(const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Pose[] targets_pose\n"
"float64[] angles\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.targets_pose);
      stream.next(m.angles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct mulObjectsPositionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur3_move::mulObjectsPositionResponse_<ContainerAllocator>& v)
  {
    s << indent << "targets_pose[]" << std::endl;
    for (size_t i = 0; i < v.targets_pose.size(); ++i)
    {
      s << indent << "  targets_pose[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "    ", v.targets_pose[i]);
    }
    s << indent << "angles[]" << std::endl;
    for (size_t i = 0; i < v.angles.size(); ++i)
    {
      s << indent << "  angles[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.angles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR3_MOVE_MESSAGE_MULOBJECTSPOSITIONRESPONSE_H
