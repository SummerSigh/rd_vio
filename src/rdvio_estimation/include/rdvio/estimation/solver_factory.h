#pragma once

#include <rdvio/estimation/solver.h>
#include <memory>
#include <string>

namespace rdvio {

enum class SolverType {
    STUB,     // No optimization (for testing)
    CERES,    // Google Ceres solver
    G2O       // g2o graph optimization
};

class SolverFactory {
public:
    static void set_solver_type(SolverType type);
    static SolverType get_solver_type();
    static std::unique_ptr<Solver> create();
    
private:
    static SolverType solver_type_;
};

} // namespace rdvio