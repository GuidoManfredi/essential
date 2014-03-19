/* Auto-generated by genmsg_cpp for file /home/gmanfred/devel/ros/Vision_pipeline/icaro/hand_detect/srv/Rectangle.srv */
#ifndef HAND_DETECT_SERVICE_RECTANGLE_H
#define HAND_DETECT_SERVICE_RECTANGLE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"



#include "hand_detect/Rectangle.h"

namespace hand_detect
{
template <class ContainerAllocator>
struct RectangleRequest_ {
  typedef RectangleRequest_<ContainerAllocator> Type;

  RectangleRequest_()
  : num_hands(0)
  {
  }

  RectangleRequest_(const ContainerAllocator& _alloc)
  : num_hands(0)
  {
  }

  typedef int32_t _num_hands_type;
  int32_t num_hands;


  typedef boost::shared_ptr< ::hand_detect::RectangleRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hand_detect::RectangleRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RectangleRequest
typedef  ::hand_detect::RectangleRequest_<std::allocator<void> > RectangleRequest;

typedef boost::shared_ptr< ::hand_detect::RectangleRequest> RectangleRequestPtr;
typedef boost::shared_ptr< ::hand_detect::RectangleRequest const> RectangleRequestConstPtr;


template <class ContainerAllocator>
struct RectangleResponse_ {
  typedef RectangleResponse_<ContainerAllocator> Type;

  RectangleResponse_()
  : rect()
  {
  }

  RectangleResponse_(const ContainerAllocator& _alloc)
  : rect(_alloc)
  {
  }

  typedef std::vector< ::hand_detect::Rectangle_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::hand_detect::Rectangle_<ContainerAllocator> >::other >  _rect_type;
  std::vector< ::hand_detect::Rectangle_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::hand_detect::Rectangle_<ContainerAllocator> >::other >  rect;


  typedef boost::shared_ptr< ::hand_detect::RectangleResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hand_detect::RectangleResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RectangleResponse
typedef  ::hand_detect::RectangleResponse_<std::allocator<void> > RectangleResponse;

typedef boost::shared_ptr< ::hand_detect::RectangleResponse> RectangleResponsePtr;
typedef boost::shared_ptr< ::hand_detect::RectangleResponse const> RectangleResponseConstPtr;

struct Rectangle
{

typedef RectangleRequest Request;
typedef RectangleResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Rectangle
} // namespace hand_detect

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hand_detect::RectangleRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hand_detect::RectangleRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hand_detect::RectangleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "800a4f8be2c695cb0abaae7eb88676ad";
  }

  static const char* value(const  ::hand_detect::RectangleRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x800a4f8be2c695cbULL;
  static const uint64_t static_value2 = 0x0abaae7eb88676adULL;
};

template<class ContainerAllocator>
struct DataType< ::hand_detect::RectangleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_detect/RectangleRequest";
  }

  static const char* value(const  ::hand_detect::RectangleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hand_detect::RectangleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 num_hands\n\
\n\
";
  }

  static const char* value(const  ::hand_detect::RectangleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hand_detect::RectangleRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hand_detect::RectangleResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hand_detect::RectangleResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hand_detect::RectangleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c78a300dff07b60288103d0a98004613";
  }

  static const char* value(const  ::hand_detect::RectangleResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc78a300dff07b602ULL;
  static const uint64_t static_value2 = 0x88103d0a98004613ULL;
};

template<class ContainerAllocator>
struct DataType< ::hand_detect::RectangleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_detect/RectangleResponse";
  }

  static const char* value(const  ::hand_detect::RectangleResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hand_detect::RectangleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Rectangle[] rect\n\
\n\
\n\
================================================================================\n\
MSG: hand_detect/Rectangle\n\
int32 x\n\
int32 y\n\
int32 width\n\
int32 height\n\
\n\
";
  }

  static const char* value(const  ::hand_detect::RectangleResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hand_detect::RectangleRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.num_hands);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RectangleRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hand_detect::RectangleResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.rect);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RectangleResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<hand_detect::Rectangle> {
  static const char* value() 
  {
    return "9a06a55897df399bd353353c13175d07";
  }

  static const char* value(const hand_detect::Rectangle&) { return value(); } 
};

template<>
struct DataType<hand_detect::Rectangle> {
  static const char* value() 
  {
    return "hand_detect/Rectangle";
  }

  static const char* value(const hand_detect::Rectangle&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hand_detect::RectangleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9a06a55897df399bd353353c13175d07";
  }

  static const char* value(const hand_detect::RectangleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hand_detect::RectangleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_detect/Rectangle";
  }

  static const char* value(const hand_detect::RectangleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hand_detect::RectangleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9a06a55897df399bd353353c13175d07";
  }

  static const char* value(const hand_detect::RectangleResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hand_detect::RectangleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_detect/Rectangle";
  }

  static const char* value(const hand_detect::RectangleResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // HAND_DETECT_SERVICE_RECTANGLE_H

