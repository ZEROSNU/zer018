#include <iostream>
#include <stdio.h>
#include <string>

#define Z_DEBUG false

inline void z_print(const std::string &message) { if(Z_DEBUG) std::cout<<message<<std::endl;}
