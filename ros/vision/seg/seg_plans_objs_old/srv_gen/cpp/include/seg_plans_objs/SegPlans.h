/* Auto-generated by genmsg_cpp for file /home/gmanfred/devel/ros/packs/vision/seg/seg_plans_objs/srv/SegPlans.srv */
#ifndef SEG_PLANS_OBJS_SERVICE_SEGPLANS_H
#define SEG_PLANS_OBJS_SERVICE_SEGPLANS_H
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



#include "sensor_msgs/PointCloud2.h"

namespace seg_plans_objs
{
template <class ContainerAllocator>
struct SegPlansRequest_ {
  typedef SegPlansRequest_<ContainerAllocator> Type;

  SegPlansRequest_()
  : num_plans_requested(0)
  {
  }

  SegPlansRequest_(const ContainerAllocator& _alloc)
  : num_plans_requested(0)
  {
  }

  typedef int64_t _num_plans_requested_type;
  int64_t num_plans_requested;


  typedef boost::shared_ptr< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SegPlansRequest
typedef  ::seg_plans_objs::SegPlansRequest_<std::allocator<void> > SegPlansRequest;

typedef boost::shared_ptr< ::seg_plans_objs::SegPlansRequest> SegPlansRequestPtr;
typedef boost::shared_ptr< ::seg_plans_objs::SegPlansRequest const> SegPlansRequestConstPtr;


template <class ContainerAllocator>
struct SegPlansResponse_ {
  typedef SegPlansResponse_<ContainerAllocator> Type;

  SegPlansResponse_()
  : segmented_plans()
  {
  }

  SegPlansResponse_(const ContainerAllocator& _alloc)
  : segmented_plans(_alloc)
  {
  }

  typedef std::vector< ::sensor_msgs::PointCloud2_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::PointCloud2_<ContainerAllocator> >::other >  _segmented_plans_type;
  std::vector< ::sensor_msgs::PointCloud2_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::PointCloud2_<ContainerAllocator> >::other >  segmented_plans;


  typedef boost::shared_ptr< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SegPlansResponse
typedef  ::seg_plans_objs::SegPlansResponse_<std::allocator<void> > SegPlansResponse;

typedef boost::shared_ptr< ::seg_plans_objs::SegPlansResponse> SegPlansResponsePtr;
typedef boost::shared_ptr< ::seg_plans_objs::SegPlansResponse const> SegPlansResponseConstPtr;

struct SegPlans
{

typedef SegPlansRequest Request;
typedef SegPlansResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SegPlans
} // namespace seg_plans_objs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6c042850bdcce03549506f85b146e77b";
  }

  static const char* value(const  ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6c042850bdcce035ULL;
  static const uint64_t static_value2 = 0x49506f85b146e77bULL;
};

template<class ContainerAllocator>
struct DataType< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "seg_plans_objs/SegPlansRequest";
  }

  static const char* value(const  ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int64 num_plans_requested\n\
\n\
";
  }

  static const char* value(const  ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3231d5cf1e568de703776582fa3afffe";
  }

  static const char* value(const  ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3231d5cf1e568de7ULL;
  static const uint64_t static_value2 = 0x03776582fa3afffeULL;
};

template<class ContainerAllocator>
struct DataType< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "seg_plans_objs/SegPlansResponse";
  }

  static const char* value(const  ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "sensor_msgs/PointCloud2[] segmented_plans\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/PointCloud2\n\
# This message holds a collection of N-dimensional points, which may\n\
# contain additional information such as normals, intensity, etc. The\n\
# point data is stored as a binary blob, its layout described by the\n\
# contents of the \"fields\" array.\n\
\n\
# The point cloud data may be organized 2d (image-like) or 1d\n\
# (unordered). Point clouds organized as 2d images may be produced by\n\
# camera depth sensors such as stereo or time-of-flight.\n\
\n\
# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n\
# points).\n\
Header header\n\
\n\
# 2D structure of the point cloud. If the cloud is unordered, height is\n\
# 1 and width is the length of the point cloud.\n\
uint32 height\n\
uint32 width\n\
\n\
# Describes the channels and their layout in the binary data blob.\n\
PointField[] fields\n\
\n\
bool    is_bigendian # Is this data bigendian?\n\
uint32  point_step   # Length of a point in bytes\n\
uint32  row_step     # Length of a row in bytes\n\
uint8[] data         # Actual point data, size is (row_step*height)\n\
\n\
bool is_dense        # True if there are no invalid points\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: sensor_msgs/PointField\n\
# This message holds the description of one point entry in the\n\
# PointCloud2 message format.\n\
uint8 INT8    = 1\n\
uint8 UINT8   = 2\n\
uint8 INT16   = 3\n\
uint8 UINT16  = 4\n\
uint8 INT32   = 5\n\
uint8 UINT32  = 6\n\
uint8 FLOAT32 = 7\n\
uint8 FLOAT64 = 8\n\
\n\
string name      # Name of field\n\
uint32 offset    # Offset from start of point struct\n\
uint8  datatype  # Datatype enumeration, see above\n\
uint32 count     # How many elements in the field\n\
\n\
";
  }

  static const char* value(const  ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::seg_plans_objs::SegPlansRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.num_plans_requested);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SegPlansRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::seg_plans_objs::SegPlansResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.segmented_plans);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SegPlansResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<seg_plans_objs::SegPlans> {
  static const char* value() 
  {
    return "73a9f83b02cede52fedb253954bf89a9";
  }

  static const char* value(const seg_plans_objs::SegPlans&) { return value(); } 
};

template<>
struct DataType<seg_plans_objs::SegPlans> {
  static const char* value() 
  {
    return "seg_plans_objs/SegPlans";
  }

  static const char* value(const seg_plans_objs::SegPlans&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<seg_plans_objs::SegPlansRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "73a9f83b02cede52fedb253954bf89a9";
  }

  static const char* value(const seg_plans_objs::SegPlansRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<seg_plans_objs::SegPlansRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "seg_plans_objs/SegPlans";
  }

  static const char* value(const seg_plans_objs::SegPlansRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<seg_plans_objs::SegPlansResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "73a9f83b02cede52fedb253954bf89a9";
  }

  static const char* value(const seg_plans_objs::SegPlansResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<seg_plans_objs::SegPlansResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "seg_plans_objs/SegPlans";
  }

  static const char* value(const seg_plans_objs::SegPlansResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // SEG_PLANS_OBJS_SERVICE_SEGPLANS_H
