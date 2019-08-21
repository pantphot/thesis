#include <iostream>
#include <string>
#include "PointHeader_PubSubTypes.h"
#include "../../include/NGSIv2/idl/JsonNGSIv2PubSubTypes.h"

#if defined(_WIN32) && defined (BUILD_SHARED_LIBS)
#if defined (_MSC_VER)
#pragma warning(disable: 4251)
#endif
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define  USER_LIB_EXPORT __declspec(dllexport)
#else
#define  USER_LIB_EXPORT __declspec(dllimport)
#endif
#else
#define USER_LIB_EXPORT
#endif

using eprosima::fastrtps::rtps::SerializedPayload_t;

extern "C" void USER_LIB_EXPORT transform(SerializedPayload_t *serialized_input, SerializedPayload_t *serialized_output){
    // User types
    nettools_msgs::msg::dds_::PointHeader_ point;
    nettools_msgs::msg::dds_::PointHeader_PubSubType point_pst;
    std::string data;
    JsonNGSIv2PubSubType string_pst;
    JsonNGSIv2 string_data;
    std:: string json;
    // Deserialization
    point_pst.deserialize(serialized_input, &point);
    double zero = 0.0 ;
    if (point.x_() != 0){
      json = "{\"stamp\":{ \"value\":"+ std::to_string(point.stamp_()) +"}, \
      \"frame_id\":{ \"value\":"+ point.frame_id_() +"}, \
      \"x\":{ \"value\":"+ std::to_string(point.x_()) +"}, \
      \"y\": { \"value\":"+ std::to_string(point.y_()) + "}, \
      \"z\": { \"value\":"+ std::to_string(point.z_()) + "}}";
    }
    else{
      std::cout<<"entered!"<<std::endl;
      json = "{\"stamp\":{ \"value\":"+ std::to_string(point.stamp_()) +"}, \
      \"frame_id\":{ \"value\":"+ point.frame_id_() +"}, \
      \"x\":{ \"value\":"+ std::to_string(zero) +"}, \
      \"y\": { \"value\":"+std::to_string(zero)+ "}, \
      \"z\": { \"value\":"+std::to_string(zero)+"}}";
    }
   string_data.entityId("PointHeader"); // Fixed for the example
   string_data.data(json);
   std::cout << string_data.data() << std::endl;

   // Serialization
   serialized_output->reserve(string_pst.m_typeSize);
   string_pst.serialize(&string_data, serialized_output);
}
