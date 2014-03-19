/* Auto-generated by genmsg_cpp for file /home/gmanfred/devel/ros/my_packs/opencv_util/opencv_display/msg/LocaPose.msg */
#ifndef OPENCV_DISPLAY_MESSAGE_LOCAPOSE_H
#define OPENCV_DISPLAY_MESSAGE_LOCAPOSE_H
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


namespace opencv_display
{
template <class ContainerAllocator>
struct LocaPose_ {
  typedef LocaPose_<ContainerAllocator> Type;

  LocaPose_()
  : t00(0.0)
  , t01(0.0)
  , t02(0.0)
  , t03(0.0)
  , t10(0.0)
  , t11(0.0)
  , t12(0.0)
  , t13(0.0)
  , t20(0.0)
  , t21(0.0)
  , t22(0.0)
  , t23(0.0)
  {
  }

  LocaPose_(const ContainerAllocator& _alloc)
  : t00(0.0)
  , t01(0.0)
  , t02(0.0)
  , t03(0.0)
  , t10(0.0)
  , t11(0.0)
  , t12(0.0)
  , t13(0.0)
  , t20(0.0)
  , t21(0.0)
  , t22(0.0)
  , t23(0.0)
  {
  }

  typedef double _t00_type;
  double t00;

  typedef double _t01_type;
  double t01;

  typedef double _t02_type;
  double t02;

  typedef double _t03_type;
  double t03;

  typedef double _t10_type;
  double t10;

  typedef double _t11_type;
  double t11;

  typedef double _t12_type;
  double t12;

  typedef double _t13_type;
  double t13;

  typedef double _t20_type;
  double t20;

  typedef double _t21_type;
  double t21;

  typedef double _t22_type;
  double t22;

  typedef double _t23_type;
  double t23;


  typedef boost::shared_ptr< ::opencv_display::LocaPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opencv_display::LocaPose_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LocaPose
typedef  ::opencv_display::LocaPose_<std::allocator<void> > LocaPose;

typedef boost::shared_ptr< ::opencv_display::LocaPose> LocaPosePtr;
typedef boost::shared_ptr< ::opencv_display::LocaPose const> LocaPoseConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::opencv_display::LocaPose_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::opencv_display::LocaPose_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace opencv_display

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::opencv_display::LocaPose_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::opencv_display::LocaPose_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::opencv_display::LocaPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "af3859092f247ba2ebdd9f8841635b39";
  }

  static const char* value(const  ::opencv_display::LocaPose_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xaf3859092f247ba2ULL;
  static const uint64_t static_value2 = 0xebdd9f8841635b39ULL;
};

template<class ContainerAllocator>
struct DataType< ::opencv_display::LocaPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "opencv_display/LocaPose";
  }

  static const char* value(const  ::opencv_display::LocaPose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::opencv_display::LocaPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 t00\n\
float64 t01\n\
float64 t02\n\
float64 t03\n\
float64 t10\n\
float64 t11\n\
float64 t12\n\
float64 t13\n\
float64 t20\n\
float64 t21\n\
float64 t22\n\
float64 t23\n\
\n\
";
  }

  static const char* value(const  ::opencv_display::LocaPose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::opencv_display::LocaPose_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::opencv_display::LocaPose_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.t00);
    stream.next(m.t01);
    stream.next(m.t02);
    stream.next(m.t03);
    stream.next(m.t10);
    stream.next(m.t11);
    stream.next(m.t12);
    stream.next(m.t13);
    stream.next(m.t20);
    stream.next(m.t21);
    stream.next(m.t22);
    stream.next(m.t23);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LocaPose_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opencv_display::LocaPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::opencv_display::LocaPose_<ContainerAllocator> & v) 
  {
    s << indent << "t00: ";
    Printer<double>::stream(s, indent + "  ", v.t00);
    s << indent << "t01: ";
    Printer<double>::stream(s, indent + "  ", v.t01);
    s << indent << "t02: ";
    Printer<double>::stream(s, indent + "  ", v.t02);
    s << indent << "t03: ";
    Printer<double>::stream(s, indent + "  ", v.t03);
    s << indent << "t10: ";
    Printer<double>::stream(s, indent + "  ", v.t10);
    s << indent << "t11: ";
    Printer<double>::stream(s, indent + "  ", v.t11);
    s << indent << "t12: ";
    Printer<double>::stream(s, indent + "  ", v.t12);
    s << indent << "t13: ";
    Printer<double>::stream(s, indent + "  ", v.t13);
    s << indent << "t20: ";
    Printer<double>::stream(s, indent + "  ", v.t20);
    s << indent << "t21: ";
    Printer<double>::stream(s, indent + "  ", v.t21);
    s << indent << "t22: ";
    Printer<double>::stream(s, indent + "  ", v.t22);
    s << indent << "t23: ";
    Printer<double>::stream(s, indent + "  ", v.t23);
  }
};


} // namespace message_operations
} // namespace ros

#endif // OPENCV_DISPLAY_MESSAGE_LOCAPOSE_H

