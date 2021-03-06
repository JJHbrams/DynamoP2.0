// Generated by gencpp from file dynamo_planner/physics_data_sampler.msg
// DO NOT EDIT!


#ifndef DYNAMO_PLANNER_MESSAGE_PHYSICS_DATA_SAMPLER_H
#define DYNAMO_PLANNER_MESSAGE_PHYSICS_DATA_SAMPLER_H

#include <ros/service_traits.h>


#include <dynamo_planner/physics_data_samplerRequest.h>
#include <dynamo_planner/physics_data_samplerResponse.h>


namespace dynamo_planner
{

struct physics_data_sampler
{

typedef physics_data_samplerRequest Request;
typedef physics_data_samplerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct physics_data_sampler
} // namespace dynamo_planner


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dynamo_planner::physics_data_sampler > {
  static const char* value()
  {
    return "d13b09ac032499e03ad2334da1a598d0";
  }

  static const char* value(const ::dynamo_planner::physics_data_sampler&) { return value(); }
};

template<>
struct DataType< ::dynamo_planner::physics_data_sampler > {
  static const char* value()
  {
    return "dynamo_planner/physics_data_sampler";
  }

  static const char* value(const ::dynamo_planner::physics_data_sampler&) { return value(); }
};


// service_traits::MD5Sum< ::dynamo_planner::physics_data_samplerRequest> should match
// service_traits::MD5Sum< ::dynamo_planner::physics_data_sampler >
template<>
struct MD5Sum< ::dynamo_planner::physics_data_samplerRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dynamo_planner::physics_data_sampler >::value();
  }
  static const char* value(const ::dynamo_planner::physics_data_samplerRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamo_planner::physics_data_samplerRequest> should match
// service_traits::DataType< ::dynamo_planner::physics_data_sampler >
template<>
struct DataType< ::dynamo_planner::physics_data_samplerRequest>
{
  static const char* value()
  {
    return DataType< ::dynamo_planner::physics_data_sampler >::value();
  }
  static const char* value(const ::dynamo_planner::physics_data_samplerRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dynamo_planner::physics_data_samplerResponse> should match
// service_traits::MD5Sum< ::dynamo_planner::physics_data_sampler >
template<>
struct MD5Sum< ::dynamo_planner::physics_data_samplerResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dynamo_planner::physics_data_sampler >::value();
  }
  static const char* value(const ::dynamo_planner::physics_data_samplerResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamo_planner::physics_data_samplerResponse> should match
// service_traits::DataType< ::dynamo_planner::physics_data_sampler >
template<>
struct DataType< ::dynamo_planner::physics_data_samplerResponse>
{
  static const char* value()
  {
    return DataType< ::dynamo_planner::physics_data_sampler >::value();
  }
  static const char* value(const ::dynamo_planner::physics_data_samplerResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMO_PLANNER_MESSAGE_PHYSICS_DATA_SAMPLER_H
