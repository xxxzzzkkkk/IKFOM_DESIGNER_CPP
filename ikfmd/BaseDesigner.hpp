#pragma once
#include <casadi/casadi.hpp>
#include <vector>
#include <iostream>

using namespace casadi;

class BaseDesigner : public Callback {
public:
    BaseDesigner(const std::string& name, const Dict& opts=Dict());
    virtual ~BaseDesigner() {}

    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }
    void init() override {}
    std::vector<DM> eval(const std::vector<DM>& arg) const override { return {}; }


    std::vector<MX> p, x, dx, u, w, v;
    MX dx_vec, w_vec, v_vec;
    MX expr_dyn, expr_meas;
    MX jac_df_dx, jac_df_dw, jac_dh_dx, jac_dh_dv;

 
    Function f, df_dx, df_dw, h, dh_dx, dh_dv;

protected:
    virtual std::vector<MX> define_parameters() = 0;
    virtual std::vector<MX> define_states() = 0;
    virtual std::vector<MX> define_states_perturbation() = 0;
    virtual std::vector<MX> define_inputs() = 0;
    virtual std::vector<MX> define_process_noises() = 0;
    virtual std::vector<MX> define_measurement_noises() = 0;

    virtual MX dyn(const std::vector<MX>& x,
                   const std::vector<MX>& u,
                   const std::vector<MX>& w,
                   const std::vector<MX>& p) = 0;

    virtual MX meas(const std::vector<MX>& x,
                    const std::vector<MX>& v,
                    const std::vector<MX>& p) = 0;

    virtual MX perturb_states(const std::vector<MX>& states, 
                             const std::vector<MX>& perturbation) = 0;

    //virtual MX get_measure_perturb(const std::vector<MX>& measure, 
    //                        const std::vector<MX>& measure_perturb) = 0;

    

private:
    void init_functions();
public:
    void build();
};
