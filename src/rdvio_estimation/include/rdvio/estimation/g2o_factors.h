#pragma once

#include <rdvio/estimation/reprojection_factor.h>
#include <rdvio/estimation/preintegration_factor.h>
#include <rdvio/estimation/rotation_factor.h>
#include <rdvio/estimation/marginalization_factor.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/track.h>
#include <rdvio/map/map.h>
#include <g2o/core/sparse_optimizer.h>

namespace rdvio {

// G2O factor class declarations for external use
class G2OReprojectionErrorFactor : public ReprojectionErrorFactor {
public:
    G2OReprojectionErrorFactor(Frame* frame, Track* track, g2o::SparseOptimizer* optimizer);
};

class G2OReprojectionPriorFactor : public ReprojectionPriorFactor {
public:
    G2OReprojectionPriorFactor(Frame* frame, Track* track, g2o::SparseOptimizer* optimizer);
};

class G2ORotationPriorFactor : public RotationPriorFactor {
public:
    G2ORotationPriorFactor(Frame* frame, Track* track, g2o::SparseOptimizer* optimizer);
};

class G2OPreIntegrationErrorFactor : public PreIntegrationErrorFactor {
public:
    G2OPreIntegrationErrorFactor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration, g2o::SparseOptimizer* optimizer);
};

class G2OPreIntegrationPriorFactor : public PreIntegrationPriorFactor {
public:
    G2OPreIntegrationPriorFactor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration, g2o::SparseOptimizer* optimizer);
};

class G2OMarginalizationFactor : public MarginalizationFactor {
public:
    G2OMarginalizationFactor(Map* map, g2o::SparseOptimizer* optimizer);
    void marginalize(size_t index) override;
};

// Global optimizer instance for factory access
extern g2o::SparseOptimizer* g_current_g2o_optimizer;

} // namespace rdvio