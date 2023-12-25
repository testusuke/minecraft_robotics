#ifndef BUTTON_H_
#define BUTTON_H_

#include <Message.hpp>

typedef struct ButtonType {
    bool state;
} button_t;

//  create message
typedef sb::Message<button_t> Button;

#endif