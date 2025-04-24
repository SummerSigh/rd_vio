#pragma once

#include <ceres/ceres.h>
#include <rdvio/types.h>
#include <rdvio/geometry/lie_algebra.h>

namespace rdvio {

// In newer Ceres versions, LocalParameterization is replaced with Manifold
struct QuaternionParameterization : public ceres::Manifold {
    bool Plus(const double *q, const double *delta,
              double *q_plus_delta) const override {
        map<quaternion> result(q_plus_delta);
        result = (const_map<quaternion>(q) * expmap(const_map<vector<3>>(delta)))
                     .normalized();
        return true;
    }
    
    bool Minus(const double* y, const double* x, double* delta) const override {
        // Default implementation
        return false;
    }
    
    bool PlusJacobian(const double* x, double* jacobian) const override {
        map<matrix<4, 3, true>> J(jacobian);
        J.setIdentity(); // the composited jacobian is computed in
                         // PreIntegrationError::Evaluate(), we simply forward
                         // it.
        return true;
    }
    
    bool MinusJacobian(const double* x, double* jacobian) const override {
        // Default implementation
        return false;
    }
    
    int AmbientSize() const override { return 4; }
    int TangentSize() const override { return 3; }
};

} // namespace rdvio

