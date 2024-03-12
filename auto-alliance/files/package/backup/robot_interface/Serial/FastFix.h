//
// Created by soulde on 2023/4/17.
//

#ifndef SERIALTEST_FASTFIX_H
#define SERIALTEST_FASTFIX_H

#include "iostream"

#define LOG(type) std::clog<<#type<<" "

#define throw_with_trace(type, msg) std::clog<<#type<<" "<<msg
#endif //SERIALTEST_FASTFIX_H
