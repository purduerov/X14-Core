// Generated by gencpp from file shared_msgs/thrust_disable_inverted_msg.msg
// DO NOT EDIT!


#ifndef SHARED_MSGS_MESSAGE_THRUST_DISABLE_INVERTED_MSG_H
#define SHARED_MSGS_MESSAGE_THRUST_DISABLE_INVERTED_MSG_H


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
struct thrust_disable_inverted_msg_
{
  typedef thrust_disable_inverted_msg_<ContainerAllocator> Type;

  thrust_disable_inverted_msg_()
    : disable_thrusters()
    , inverted()  {
      disable_thrusters.assign(false);

      inverted.assign(0);
  }
  thrust_disable_inverted_msg_(const ContainerAllocator& _alloc)
    : disable_thrusters()
    , inverted()  {
  (void)_alloc;
      disable_thrusters.assign(false);

      inverted.assign(0);
  }



   typedef boost::array<uint8_t, 8>  _disable_thrusters_type;
  _disable_thrusters_type disable_thrusters;

   typedef boost::array<int8_t, 8>  _inverted_type;
  _inverted_type inverted;





  typedef boost::shared_ptr< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> const> ConstPtr;

}; // struct thrust_disable_inverted_msg_

typedef ::shared_msgs::thrust_disable_inverted_msg_<std::allocator<void> > thrust_disable_inverted_msg;

typedef boost::shared_ptr< ::shared_msgs::thrust_disable_inverted_msg > thrust_disable_inverted_msgPtr;
typedef boost::shared_ptr< ::shared_msgs::thrust_disable_inverted_msg const> thrust_disable_inverted_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator1> & lhs, const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator2> & rhs)
{
  return lhs.disable_thrusters == rhs.disable_thrusters &&
    lhs.inverted == rhs.inverted;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator1> & lhs, const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace shared_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b3d2e54482b6a65fb91c54d0d200315";
  }

  static const char* value(const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b3d2e54482b6a65ULL;
  static const uint64_t static_value2 = 0xfb91c54d0d200315ULL;
};

template<class ContainerAllocator>
struct DataType< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "shared_msgs/thrust_disable_inverted_msg";
  }

  static const char* value(const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool[8] disable_thrusters\n"
"int8[8] inverted\n"
;
  }

  static const char* value(const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.disable_thrusters);
      stream.next(m.inverted);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct thrust_disable_inverted_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::shared_msgs::thrust_disable_inverted_msg_<ContainerAllocator>& v)
  {
    s << indent << "disable_thrusters[]" << std::endl;
    for (size_t i = 0; i < v.disable_thrusters.size(); ++i)
    {
      s << indent << "  disable_thrusters[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.disable_thrusters[i]);
    }
    s << indent << "inverted[]" << std::endl;
    for (size_t i = 0; i < v.inverted.size(); ++i)
    {
      s << indent << "  inverted[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.inverted[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SHARED_MSGS_MESSAGE_THRUST_DISABLE_INVERTED_MSG_H
