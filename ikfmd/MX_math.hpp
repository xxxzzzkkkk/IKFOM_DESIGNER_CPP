#include <casadi/casadi.hpp>

using namespace casadi;

MX hat(MX v){
    return skew(v);
}

MX hat_square(MX v){
    return mtimes(skew(v),skew(v).T());
}

MX boxplus_dcm_small_d(MX dcm, MX delta){
    return mtimes(dcm, (MX::eye(3)+hat(delta)));
}

MX boxminus_dcm_small_d(MX dcm_perturb, MX dcm){
    MX d = mtimes(dcm_perturb.T(), dcm);
    return inv_skew(0.5 * (d - d.T()));
}

MX dcm_from_axis_angle(MX axis, MX angle){
    return MX::eye(3)  + sin(angle)*hat(axis) + (1-cos(angle))*hat_square(axis);
}

MX dcm_x(MX angle){
    MX axis = vertcat(MX(1), MX(0), MX(0)); 
    return dcm_from_axis_angle(axis, angle);
}

MX dcm_y(MX angle){
    MX axis = vertcat(MX(0), MX(1), MX(0)); 
    return dcm_from_axis_angle(axis, angle);
}

MX dcm_z(MX angle){
    MX axis = vertcat(MX(0), MX(0), MX(1));
    return dcm_from_axis_angle(axis, angle);
}