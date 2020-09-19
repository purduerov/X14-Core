// Generated by gencpp from file shared_msgs/depth_msg.msg
// DO NOT EDIT!


#ifndef SHARED_MSGS_MESSAGE_DEPTH_MSG_H
#define SHARED_MSGS_MESSAGE_DEPTH_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace shared_msgs
{
template <class ContainerAllocator>
struct depth_msg_
{
  typedef depth_msg_<ContainerAllocator> Type;

  depth_msg_()
    : pressure(0.0)  {
    }
  depth_msg_(const ContainerAllocator& _alloc)
    : pressure(0.0)  {
  (void)_alloc;
    }



   typedef float _pressure_type;
  _pressure_type pressure;





  typedef boost::shared_ptr< ::shared_msgs::depth_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::shared_msgs::depth_msg_<ContainerAllocator> const> ConstPtr;

}; // struct depth_msg_

typedef ::shared_msgs::depth_msg_<std::allocator<void> > depth_msg;

typedef boost::shared_ptr< ::shared_msgs::depth_msg > depth_msgPtr;
typedef boost::shared_ptr< ::shared_msgs::depth_msg const> depth_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::shared_msgs::depth_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::shared_msgs::depth_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::shared_msgs::depth_msg_<ContainerAllocator1> & lhs, const ::shared_msgs::depth_msg_<ContainerAllocator2> & rhs)
{
  return lhs.pressure == rhs.pressure;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::shared_msgs::depth_msg_<ContainerAllocator1> & lhs, const ::shared_msgs::depth_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace shared_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::shared_msgs::depth_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::shared_msgs::depth_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::shared_msgs::depth_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::shared_msgs::depth_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::shared_msgs::depth_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::shared_msgs::depth_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::shared_msgs::depth_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d08eeab8c09d2ba14f8144e3bbf40d21";
  }

  static const char* value(const ::shared_msgs::depth_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd08eeab8c09d2ba1ULL;
  static const uint64_t static_value2 = 0x4f8144e3bbf40d21ULL;
};

template<class ContainerAllocator>
struct DataType< ::shared_msgs::depth_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "shared_msgs/depth_msg";
  }

  static const char* value(const ::shared_msgs::depth_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::shared_msgs::depth_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 pressure\n"
;
  }

  static const char* value(const ::shared_msgs::depth_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::shared_msgs::depth_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pressure);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct depth_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::shared_msgs::depth_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::shared_msgs::depth_msg_<ContainerAllocator>& v)
  {
    s << indent << "pressure: ";
    Printer<float>::stream(s, indent + "  ", v.pressure);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SHARED_MSGS_MESSAGE_DEPTH_MSG_H
