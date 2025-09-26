#pragma once
#include "BaseDesigner.hpp"
#include "MX_math.hpp"

class demo : public BaseDesigner {
public:
    demo() : BaseDesigner("demo") {}

    std::vector<MX> define_parameters() override {return {MX::sym("A",1)};}
    std::vector<MX> define_states() override { return {MX::sym("R",3,3)}; }
    std::vector<MX> define_states_perturbation() override { return {MX::sym("dR",3)}; }
    std::vector<MX> define_inputs() override { return {MX::sym("angle_vel",3)}; }
    std::vector<MX> define_process_noises() override { return {MX::sym("n_angle_vel",3)}; }
    std::vector<MX> define_measurement_noises() override { return {MX::sym("n_lin_acc",3)}; }

    MX dyn(const std::vector<MX>& x,
       const std::vector<MX>& u,
       const std::vector<MX>& w,
       const std::vector<MX>& p) override
    { 
        return u[0] + w[0];
    }

    MX meas(const std::vector<MX>& x,
            const std::vector<MX>& v,
            const std::vector<MX>& p) override
    {  
        MX axis = vertcat(MX(0), MX(0), MX(1));
        return mtimes(x[0].T(), axis) + v[0];
    }

    MX perturb_states(const std::vector<MX>& states, 
                     const std::vector<MX>& perturbation) override
    {
        return boxplus_dcm_small_d(states[0], perturbation[0]);
    }

    //MX get_measure_perturb(const std::vector<MX>& measure, 
    //                 const std::vector<MX>& measure_perturb) override
    //{
    //    return measure_perturb[0] - measure[0];
    //}
};

