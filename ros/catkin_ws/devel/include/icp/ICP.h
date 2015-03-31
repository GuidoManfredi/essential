/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/gmanfred/devel/essential/ros/catkin_ws/src/icp/srv/ICP.srv
 *
 */


#ifndef ICP_MESSAGE_ICP_H
#define ICP_MESSAGE_ICP_H

#include <ros/service_traits.h>


#include <icp/ICPRequest.h>
#include <icp/ICPResponse.h>


namespace icp
{

struct ICP
{

typedef ICPRequest Request;
typedef ICPResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ICP
} // namespace icp


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::icp::ICP > {
  static const char* value()
  {
    return "90464cf3aacbe934064b407378a1c0f1";
  }

  static const char* value(const ::icp::ICP&) { return value(); }
};

template<>
struct DataType< ::icp::ICP > {
  static const char* value()
  {
    return "icp/ICP";
  }

  static const char* value(const ::icp::ICP&) { return value(); }
};


// service_traits::MD5Sum< ::icp::ICPRequest> should match 
// service_traits::MD5Sum< ::icp::ICP > 
template<>
struct MD5Sum< ::icp::ICPRequest>
{
  static const char* value()
  {
    return MD5Sum< ::icp::ICP >::value();
  }
  static const char* value(const ::icp::ICPRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::icp::ICPRequest> should match 
// service_traits::DataType< ::icp::ICP > 
template<>
struct DataType< ::icp::ICPRequest>
{
  static const char* value()
  {
    return DataType< ::icp::ICP >::value();
  }
  static const char* value(const ::icp::ICPRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::icp::ICPResponse> should match 
// service_traits::MD5Sum< ::icp::ICP > 
template<>
struct MD5Sum< ::icp::ICPResponse>
{
  static const char* value()
  {
    return MD5Sum< ::icp::ICP >::value();
  }
  static const char* value(const ::icp::ICPResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::icp::ICPResponse> should match 
// service_traits::DataType< ::icp::ICP > 
template<>
struct DataType< ::icp::ICPResponse>
{
  static const char* value()
  {
    return DataType< ::icp::ICP >::value();
  }
  static const char* value(const ::icp::ICPResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ICP_MESSAGE_ICP_H