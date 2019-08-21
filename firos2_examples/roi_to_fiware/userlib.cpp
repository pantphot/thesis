#include <iostream>
#include <string>
#include "RoiWithHeaderCompact_PubSubTypes.h"
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
    nettools_msgs::msg::dds_::RoiWithHeaderCompact_ roi;
    nettools_msgs::msg::dds_::RoiWithHeaderCompact_PubSubType roi_pst;
    std::string data;

    // std_msgs::msg::dds_::String_ data;
    // std_msgs::msg::dds_::String_PubSubType data_pst;

    JsonNGSIv2PubSubType string_pst;
    JsonNGSIv2 string_data;

    // Deserialization
    roi_pst.deserialize(serialized_input, &roi);

    // "Hello World: X"
    // -->
    // std::string json = "{ \"id\": \"Helloworld\", \"type\": \"Helloworld\", \
            // \"count\": { \"value\": x, \"type\": \"Number\" } }";

            // std::string msg = data.data_();
            // msg = msg.substr(msg.find(":") + 2); // ": "
     // std::cout << "/* stamp1 */" <<roi_with_header.header_().stamp_()<<std::endl;
    // std::cout << "/* stamp2 */" <<roi.nanosec_()<<std::endl;

    std::string frame_id = roi.frame_id_();
    double sec = roi.sec_();
    double nanosec= roi.nanosec_();
    long x_offset = roi.x_offset_();
    long y_offset = roi.y_offset_();
    long height = roi.height_();
    long width = roi.width_();
    bool do_rectify =roi.do_rectify_();
    std::string rectify;
    if (do_rectify==true){
      rectify = "1";
    }
    else {
       rectify = "0";
    }
    double image_width = roi.image_width_();
    double image_height = roi.image_height_();


    // std::cout << "frame_id "<< frame_id << std::endl;
    // std::cout << "stamp =  "<< stamp << std::endl;
    // std::cout << "x_offset  "<< x_offset << std::endl;
    // std::cout << "y "<< y_offset << std::endl;
    // std::cout << "height "<< height << std::endl;
    // std::cout << "width "<< width << std::endl;
    // std::cout << "image_width "<< image_width << std::endl;
    // std::cout << "image_height "<<image_height << std::endl;
    // std::cout << "rectify "<<rectify << std::endl;


    // Custom transformation
    // std:: string json = "{\"count\": { \"value\": " + msg + "} }";
    std:: string json = "{\"frame_id\":{ \"value\":"+ frame_id +"}, \
    \"sec\": { \"value\":"+ std::to_string(sec) + "}, \
    \"nanosec\": { \"value\":"+ std::to_string(nanosec) + "}, \
    \"x_offset\": { \"value\":"+ std::to_string(x_offset) + "}, \
    \"y_offset\": { \"value\":"+ std::to_string(y_offset) + "}, \
    \"height\": { \"value\":"+ std::to_string(height) + "}, \
    \"width\": { \"value\":"+ std::to_string(width) + "}, \
    \"do_rectify\": { \"value\":"+ rectify + "}, \
    \"image_width\": { \"value\":"+ std::to_string(image_width) + "}, \
    \"image_height\": { \"value\":"+ std::to_string(image_height)+ "}}";

    string_data.entityId("Roi"); // Fixed for the example
    string_data.data(json);
    std::cout << string_data.data() << std::endl;

    // Serialization
    serialized_output->reserve(string_pst.m_typeSize);
    string_pst.serialize(&string_data, serialized_output);
}
