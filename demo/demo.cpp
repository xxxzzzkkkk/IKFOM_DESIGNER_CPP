#include "demo.hpp"
#include <iostream>

int main() {
    demo test;
    test.build();  

   
    MX x = test.x[0];  
    MX u = test.u[0];  
    MX w = test.w[0];  
    MX v = test.v[0];  
    MX p = test.p[0];  

    MX f_val     = test.f(MXVector{x, u, w, p})[0];
    MX h_val     = test.h(MXVector{x, v, p})[0];
    MX df_dx_val = test.df_dx(MXVector{x, u, w, p})[0];
    MX df_dw_val = test.df_dw(MXVector{x, u, w, p})[0];
    MX dh_dx_val = test.dh_dx(MXVector{x, v, p})[0];
    MX dh_dv_val = test.dh_dv(MXVector{x, v, p})[0];

    std::cout << "f(x,u,w,p) = " << f_val << std::endl;
    std::cout << "h(x,v,p)   = " << h_val << std::endl;
    std::cout << "df/dx      = " << df_dx_val << std::endl;
    std::cout << "df/dw      = " << df_dw_val << std::endl;
    std::cout << "dh/dx      = " << dh_dx_val << std::endl;
    std::cout << "dh/dv      = " << dh_dv_val << std::endl;

    return 0;
}

