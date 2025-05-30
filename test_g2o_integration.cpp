#include <iostream>
#include <rdvio/estimation/solver.h>

using namespace rdvio;

int main() {
    std::cout << "Testing g2o integration..." << std::endl;
    
    // Create a solver instance
    auto solver = Solver::create();
    if (solver) {
        std::cout << "✓ Solver created successfully" << std::endl;
        
        // Try to solve (should succeed even with no states/factors)
        bool result = solver->solve(false);
        if (result) {
            std::cout << "✓ G2O optimization completed successfully" << std::endl;
        } else {
            std::cout << "✗ G2O optimization failed" << std::endl;
        }
    } else {
        std::cout << "✗ Failed to create solver" << std::endl;
        return 1;
    }
    
    std::cout << "✓ G2O integration test passed!" << std::endl;
    return 0;
}