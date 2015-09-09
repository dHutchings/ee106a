// Generated by gencpp from file my_chatter/TimestampString.msg
// DO NOT EDIT!


#ifndef MY_CHATTER_MESSAGE_TIMESTAMPSTRING_H
#define MY_CHATTER_MESSAGE_TIMESTAMPSTRING_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_chatter
{
template <class ContainerAllocator>
struct TimestampString_
{
  typedef TimestampString_<ContainerAllocator> Type;

  TimestampString_()
    : userInput()
    , timestamp(0)  {
    }
  TimestampString_(const ContainerAllocator& _alloc)
    : userInput(_alloc)
    , timestamp(0)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _userInput_type;
  _userInput_type userInput;

   typedef int32_t _timestamp_type;
  _timestamp_type timestamp;




  typedef boost::shared_ptr< ::my_chatter::TimestampString_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_chatter::TimestampString_<ContainerAllocator> const> ConstPtr;

}; // struct TimestampString_

typedef ::my_chatter::TimestampString_<std::allocator<void> > TimestampString;

typedef boost::shared_ptr< ::my_chatter::TimestampString > TimestampStringPtr;
typedef boost::shared_ptr< ::my_chatter::TimestampString const> TimestampStringConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_chatter::TimestampString_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_chatter::TimestampString_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace my_chatter

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'my_chatter': ['/home/cc/ee106a/fa15/class/ee106a-ab/ros_workspace/lab2/src/my_chatter/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::my_chatter::TimestampString_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_chatter::TimestampString_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_chatter::TimestampString_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_chatter::TimestampString_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_chatter::TimestampString_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_chatter::TimestampString_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f29425b1cb0ad76e15fd39bb8ecdb7f";
  }

  static const char* value(const ::my_chatter::TimestampString_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f29425b1cb0ad76ULL;
  static const uint64_t static_value2 = 0xe15fd39bb8ecdb7fULL;
};

template<class ContainerAllocator>
struct DataType< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_chatter/TimestampString";
  }

  static const char* value(const ::my_chatter::TimestampString_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string userInput\n\
int32 timestamp\n\
";
  }

  static const char* value(const ::my_chatter::TimestampString_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_chatter::TimestampString_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.userInput);
      stream.next(m.timestamp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct TimestampString_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_chatter::TimestampString_<ContainerAllocator>& v)
  {
    s << indent << "userInput: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.userInput);
    s << indent << "timestamp: ";
    Printer<int32_t>::stream(s, indent + "  ", v.timestamp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_CHATTER_MESSAGE_TIMESTAMPSTRING_H
