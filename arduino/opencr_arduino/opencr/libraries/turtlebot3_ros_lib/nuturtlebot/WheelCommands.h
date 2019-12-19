#ifndef _ROS_nuturtlebot_WheelCommands_h
#define _ROS_nuturtlebot_WheelCommands_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nuturtlebot
{

  class WheelCommands : public ros::Msg
  {
    public:
      typedef int32_t _left_velocity_type;
      _left_velocity_type left_velocity;
      typedef int32_t _right_velocity_type;
      _right_velocity_type right_velocity;

    WheelCommands():
      left_velocity(0),
      right_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_left_velocity;
      u_left_velocity.real = this->left_velocity;
      *(outbuffer + offset + 0) = (u_left_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_velocity);
      union {
        int32_t real;
        uint32_t base;
      } u_right_velocity;
      u_right_velocity.real = this->right_velocity;
      *(outbuffer + offset + 0) = (u_right_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_left_velocity;
      u_left_velocity.base = 0;
      u_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_velocity = u_left_velocity.real;
      offset += sizeof(this->left_velocity);
      union {
        int32_t real;
        uint32_t base;
      } u_right_velocity;
      u_right_velocity.base = 0;
      u_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_velocity = u_right_velocity.real;
      offset += sizeof(this->right_velocity);
     return offset;
    }

    const char * getType(){ return "nuturtlebot/WheelCommands"; };
    const char * getMD5(){ return "55f26e03298a44797b568254274af487"; };

  };

}
#endif
