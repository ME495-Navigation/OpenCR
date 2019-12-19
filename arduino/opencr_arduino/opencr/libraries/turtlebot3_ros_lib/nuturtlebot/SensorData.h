#ifndef _ROS_nuturtlebot_SensorData_h
#define _ROS_nuturtlebot_SensorData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace nuturtlebot
{

  class SensorData : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef int32_t _left_encoder_type;
      _left_encoder_type left_encoder;
      typedef int32_t _right_encoder_type;
      _right_encoder_type right_encoder;
      typedef int16_t _accelX_type;
      _accelX_type accelX;
      typedef int16_t _accelY_type;
      _accelY_type accelY;
      typedef int16_t _accelZ_type;
      _accelZ_type accelZ;
      typedef int16_t _gyroX_type;
      _gyroX_type gyroX;
      typedef int16_t _gyroY_type;
      _gyroY_type gyroY;
      typedef int16_t _gyroZ_type;
      _gyroZ_type gyroZ;
      typedef int16_t _magX_type;
      _magX_type magX;
      typedef int16_t _magY_type;
      _magY_type magY;
      typedef int16_t _magZ_type;
      _magZ_type magZ;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;

    SensorData():
      stamp(),
      left_encoder(0),
      right_encoder(0),
      accelX(0),
      accelY(0),
      accelZ(0),
      gyroX(0),
      gyroY(0),
      gyroZ(0),
      magX(0),
      magY(0),
      magZ(0),
      battery_voltage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_left_encoder;
      u_left_encoder.real = this->left_encoder;
      *(outbuffer + offset + 0) = (u_left_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_encoder);
      union {
        int32_t real;
        uint32_t base;
      } u_right_encoder;
      u_right_encoder.real = this->right_encoder;
      *(outbuffer + offset + 0) = (u_right_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_accelX;
      u_accelX.real = this->accelX;
      *(outbuffer + offset + 0) = (u_accelX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accelX.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->accelX);
      union {
        int16_t real;
        uint16_t base;
      } u_accelY;
      u_accelY.real = this->accelY;
      *(outbuffer + offset + 0) = (u_accelY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accelY.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->accelY);
      union {
        int16_t real;
        uint16_t base;
      } u_accelZ;
      u_accelZ.real = this->accelZ;
      *(outbuffer + offset + 0) = (u_accelZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accelZ.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->accelZ);
      union {
        int16_t real;
        uint16_t base;
      } u_gyroX;
      u_gyroX.real = this->gyroX;
      *(outbuffer + offset + 0) = (u_gyroX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyroX.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gyroX);
      union {
        int16_t real;
        uint16_t base;
      } u_gyroY;
      u_gyroY.real = this->gyroY;
      *(outbuffer + offset + 0) = (u_gyroY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyroY.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gyroY);
      union {
        int16_t real;
        uint16_t base;
      } u_gyroZ;
      u_gyroZ.real = this->gyroZ;
      *(outbuffer + offset + 0) = (u_gyroZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyroZ.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gyroZ);
      union {
        int16_t real;
        uint16_t base;
      } u_magX;
      u_magX.real = this->magX;
      *(outbuffer + offset + 0) = (u_magX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_magX.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->magX);
      union {
        int16_t real;
        uint16_t base;
      } u_magY;
      u_magY.real = this->magY;
      *(outbuffer + offset + 0) = (u_magY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_magY.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->magY);
      union {
        int16_t real;
        uint16_t base;
      } u_magZ;
      u_magZ.real = this->magZ;
      *(outbuffer + offset + 0) = (u_magZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_magZ.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->magZ);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_left_encoder;
      u_left_encoder.base = 0;
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_encoder = u_left_encoder.real;
      offset += sizeof(this->left_encoder);
      union {
        int32_t real;
        uint32_t base;
      } u_right_encoder;
      u_right_encoder.base = 0;
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_encoder = u_right_encoder.real;
      offset += sizeof(this->right_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_accelX;
      u_accelX.base = 0;
      u_accelX.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accelX.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accelX = u_accelX.real;
      offset += sizeof(this->accelX);
      union {
        int16_t real;
        uint16_t base;
      } u_accelY;
      u_accelY.base = 0;
      u_accelY.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accelY.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accelY = u_accelY.real;
      offset += sizeof(this->accelY);
      union {
        int16_t real;
        uint16_t base;
      } u_accelZ;
      u_accelZ.base = 0;
      u_accelZ.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accelZ.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accelZ = u_accelZ.real;
      offset += sizeof(this->accelZ);
      union {
        int16_t real;
        uint16_t base;
      } u_gyroX;
      u_gyroX.base = 0;
      u_gyroX.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyroX.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gyroX = u_gyroX.real;
      offset += sizeof(this->gyroX);
      union {
        int16_t real;
        uint16_t base;
      } u_gyroY;
      u_gyroY.base = 0;
      u_gyroY.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyroY.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gyroY = u_gyroY.real;
      offset += sizeof(this->gyroY);
      union {
        int16_t real;
        uint16_t base;
      } u_gyroZ;
      u_gyroZ.base = 0;
      u_gyroZ.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyroZ.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gyroZ = u_gyroZ.real;
      offset += sizeof(this->gyroZ);
      union {
        int16_t real;
        uint16_t base;
      } u_magX;
      u_magX.base = 0;
      u_magX.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_magX.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->magX = u_magX.real;
      offset += sizeof(this->magX);
      union {
        int16_t real;
        uint16_t base;
      } u_magY;
      u_magY.base = 0;
      u_magY.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_magY.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->magY = u_magY.real;
      offset += sizeof(this->magY);
      union {
        int16_t real;
        uint16_t base;
      } u_magZ;
      u_magZ.base = 0;
      u_magZ.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_magZ.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->magZ = u_magZ.real;
      offset += sizeof(this->magZ);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
     return offset;
    }

    const char * getType(){ return "nuturtlebot/SensorData"; };
    const char * getMD5(){ return "e5bb303db7aaeaf900294f1b85ecc818"; };

  };

}
#endif
