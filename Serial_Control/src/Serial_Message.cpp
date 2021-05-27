#include "Serial_Message.h"
using namespace std;

MSG_CON::MSG_CON()
{
//    sub_heading = node_.subscribe("/IMU",1, &MSG_CON::subImuCallback, this);
//    sub_vel = node_.subscribe("/MSG_CON/Rx_Vel",1, &MSG_CON::subENCODECallback, this);
//    sub_steer = node_.subscribe("/MSG_CON/Rx_Steer",1, &MSG_CON::subSteerCallback, this);
//    sub_GPS_latitude = node_.subscribe("/GPS1",1, &MSG_CON::subGPS1Callback, this);
//    sub_GPS_longtitude = node_.subscribe("/GPS2",1, &MSG_CON::subGPS2Callback, this);
//    sub_GPS_speed = node_.subscribe("/GPS3",1, &MSG_CON::subGPS3Callback, this);
//    sub_GPS_heading = node_.subscribe("/GPS4",1, &MSG_CON::subGPS4Callback, this);

//    sub_cross = node_.subscribe("/Camera/cross",1, &MSG_CON::subCrossCallback, this);
//    sub_uturnbool = node_.subscribe("/Lidar/UturnBool",1, &MSG_CON::subUturnBoolCallback, this);
//    sub_uturndistance = node_.subscribe("/Lidar/UturnDistance",1, &MSG_CON::subUturnDistanceCallback, this);
//    sub_dynamic = node_.subscribe("/Lidar/dynamicBool",1, &MSG_CON::subDynamicCallback, this);


//    sub_wayPointY = node_.subscribe("/Camera/wayPointY",1, &MSG_CON::subwayPointYCallback, this);
//    sub_wayPointYleft = node_.subscribe("/Camera/wayPointYleft",1, &MSG_CON::subwayPointYleftCallback, this);
//    sub_wayPointYright = node_.subscribe("/Camera/wayPointYright",1, &MSG_CON::subwayPointYrightCallback, this);


//    pub_SteerCont = node_.advertise<std_msgs::Float64>("/MSG_CON/SteerContol",1);

//    sub_coneX = node_.subscribe("/Lidar/cone_x",1, &MSG_CON::subConeXCallback, this);
//    sub_coneY = node_.subscribe("/Lidar/cone_y",1, &MSG_CON::subConeYCallback, this);
//    sub_ob = node_.subscribe("/Lidar/ob",1, &MSG_CON::subObCallback, this);
//    sub_parkTiming = node_.subscribe("/Camera/parkTiming",1, &MSG_CON::subParkTimingCallback, this);




//    sub_sign_crosswalk = node_.subscribe("/traffic/crosswalk",1, &MSG_CON::subSignCrosswalkCallback, this);
//    sub_sign_parking = node_.subscribe("/traffic/parking",1, &MSG_CON::subSignParkingCallback, this);
//    sub_sign_uturn = node_.subscribe("/traffic/u_turn",1, &MSG_CON::subSignUturnCallback, this);
//    sub_sign_warning = node_.subscribe("/traffic/warning",1, &MSG_CON::subSignWarningCallback, this);
 //   sub_sign_blue = node_.subscribe("/traffic/bluesign",1, &MSG_CON::subSignBlueCallback, this);



    Heading = 0;
    Vel =0;
    Steer = 0;
    ConeX = 0;
}

//void MSG_CON::subImuCallback(const std_msgs::Float32 &subImuMsgs)
//{
//    Heading = subImuMsgs.data;
//    //std::cout << "Heading: " << Heading << std::endl;
//}
void MSG_CON::subENCODECallback(const std_msgs::Int8 &subencodeMsgs)
{
    Vel = subencodeMsgs.data;
    //std::cout << "Vel: " << Vel << std::endl;
}
void MSG_CON::subSteerCallback(const std_msgs::Int8 &substeerMsgs)
{
    Steer = substeerMsgs.data;
    //std::cout << "Steer: " << Steer << std::endl;
}
//void MSG_CON::subGPS1Callback(const std_msgs::Int8 &subgps1Msgs)
//{
//     GPS_La = subgps1Msgs.data;
//    //std::cout << "GPS_La: " << GPS_La << std::endl;
//}
//void MSG_CON::subGPS2Callback(const std_msgs::Int8 &subgps2Msgs)
//{
//     GPS_lo = subgps2Msgs.data;
//    //std::cout << "GPS_lo: " << GPS_lo << std::endl;
//}
//void MSG_CON::subGPS3Callback(const std_msgs::Int8 &subgps3Msgs)
//{
//     GPS_sp = subgps3Msgs.data;
//    //std::cout << "GPS_sp: " << GPS_sp << std::endl;
//}
//void MSG_CON::subGPS4Callback(const std_msgs::Int8 &subgps4Msgs)
//{
//     GPS_he = subgps4Msgs.data;
//    //std::cout << "GPS_he: " << GPS_he << std::endl;
//}
////void MSG_CON::subtestCallback(const std_msgs::Float32 &subtestMsgs)
////{
////     test_v = subtestMsgs.data;
////    std::cout << "test_v: " << test_v << std::endl;
////}

//void MSG_CON::subUturnBoolCallback(const std_msgs::Bool &subUturnBoolCallback)
//{
//     UturnBool = subUturnBoolCallback.data;
//    //std::cout << "Uturn: " << Uturn << std::endl;
//}

//void MSG_CON::subUturnDistanceCallback(const std_msgs::Float32 &subUturnDistanceMsgs)
//{
//     UturnDistance = subUturnDistanceMsgs.data;
//    //std::cout << "Uturn: " << Uturn << std::endl;
//}

//void MSG_CON::subDynamicCallback(const std_msgs::Bool &subDynamicCallback)
//{
//     Dynamic = subDynamicCallback.data;
//    //std::cout << "Dynamic: " << Dynamic << std::endl;
//}

//void MSG_CON::subwayPointYCallback(const std_msgs::Float32 &subwayPointYMsgs)
//{
//    wayPointY = subwayPointYMsgs.data;
//    //std::cout << "wayPointY: " << wayPointY << std::endl;
//}
//void MSG_CON::subwayPointYleftCallback(const std_msgs::Float32 &subwayPointYleftMsgs)
//{
//    wayPointYleft = subwayPointYleftMsgs.data;
//    //std::cout << "wayPointYleft: " << wayPointYleft << std::endl;
//}
//void MSG_CON::subwayPointYrightCallback(const std_msgs::Float32 &subwayPointYrightMsgs)
//{
//    wayPointYright = subwayPointYrightMsgs.data;
//    //std::cout << "wayPointYright: " << wayPointYright << std::endl;
//}
//void MSG_CON::subCrossCallback(const std_msgs::Int8 &subCrossCallback)
//{
//     Cross = subCrossCallback.data;
//     //std::cout << "Cross: " << Cross << std::endl;
//}
//void MSG_CON::subConeXCallback(const std_msgs::Float32 &subConeXMsgs)
//{
//     ConeX = subConeXMsgs.data;
//    //std::cout << "ConeX: " << ConeX << std::endl;
//}
//void MSG_CON::subConeYCallback(const std_msgs::Float32 &subConeYMsgs)
//{
//     ConeY = subConeYMsgs.data;

//   // std::cout << "ConeY: " << ConeY << std::endl;
//}
//void MSG_CON::subObCallback(const std_msgs::Int32 &subObMsgs)
//{
//     Ob = subObMsgs.data;

//   // std::cout << "Ob: " << Ob << std::endl;
//}
//void MSG_CON::subParkTimingCallback(const std_msgs::Bool &subParkTimingMsgs)
//{
//     ParkTiming = subParkTimingMsgs.data;
//   // std::cout << "ParkTiming: " << ParkTiming << std::endl;
//}

////void MSG_CON::subSignCrosswalkCallback(const std_msgs::Bool &subSignCrosswalkMsgs)
////{
////     sign_crosswalk = subSignCrosswalkMsgs.data;
////   // std::cout << "sign_crosswalk: " << sign_crosswalk << std::endl;
////}
////void MSG_CON::subSignParkingCallback(const std_msgs::Bool &subSignParkingMsgs)
////{
////     sign_parking = subSignParkingMsgs.data;
////   // std::cout << "sign_parking: " << sign_parking << std::endl;
////}
////void MSG_CON::subSignUturnCallback(const std_msgs::Bool &subSignUturnMsgs)
////{
////     sign_uturn = subSignUturnMsgs.data;
////   // std::cout << "sign_uturn: " << sign_uturn << std::endl;
////}
//void MSG_CON::subSignWarningCallback(const std_msgs::Float32 &subSignWarningMsgs)
//{
//     sign_warning = subSignWarningMsgs.data;
//   // std::cout << "sign_crosswalk: " << sign_crosswalk << std::endl;
//}

//void MSG_CON::subSignBlueCallback(const std_msgs::Float32 &subSignBlueMsgs)
//{
//     sign_blue = subSignBlueMsgs.data;
//   // std::cout << "sign_crosswalk: " << sign_crosswalk << std::endl;
//}
