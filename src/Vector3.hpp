#ifndef VECTOR3_H_
#define VECTOR3_H_

#include <Message.hpp>

typedef struct Vector3Type {
    float x;
    float y;
    float z;
} vector3_t;

//  create message
typedef sb::Message<vector3_t> Vector3;

#endif