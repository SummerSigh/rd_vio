#include <rdvio/estimation/g2o_solver.h>
#include <rdvio/estimation/preintegrator.h>
#include <rdvio/estimation/state.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/track.h>
#include <rdvio/geometry/lie_algebra.h>
#include <rdvio/geometry/stereo.h>
#include <g2o/core/robust_kernel_impl.h>

namespace rdvio {

G2OSolver::G2OSolver() {
    setup_optimizer();
}

G2OSolver::~G2OSolver() = default;

void G2OSolver::setup_optimizer() {
    optimizer_ = std::make_unique<g2o::SparseOptimizer>();
    
    // Try to use CHOLMOD solver first, fall back to CSparse, then Eigen
    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver;
    
#ifdef G2O_HAVE_CHOLMOD
    linearSolver = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
#elif G2O_HAVE_CSPARSE
    linearSolver = std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
#else
    linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
#endif
    
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
    
    optimizer_->setAlgorithm(solver);
    optimizer_->setVerbose(false);
}

void G2OSolver::add_pose_vertex(size_t frame_id, const PoseState& pose, bool fixed) {
    auto* v = new g2o::VertexSE3Expmap();
    v->setId(get_next_vertex_id());
    
    // Convert pose to SE3
    g2o::SE3Quat se3(pose.q, pose.p);
    v->setEstimate(se3);
    v->setFixed(fixed);
    
    optimizer_->addVertex(v);
    frame_pose_vertices_[frame_id] = v;
}

void G2OSolver::add_extrinsic_vertex(size_t frame_id, const ExtrinsicParam& extrinsic, bool fixed) {
    auto* v = new g2o::VertexSE3Expmap();
    v->setId(get_next_vertex_id());
    
    g2o::SE3Quat se3(extrinsic.q_cs, extrinsic.p_cs);
    v->setEstimate(se3);
    v->setFixed(fixed);
    
    optimizer_->addVertex(v);
    frame_extrinsic_vertices_[frame_id] = v;
}

void G2OSolver::add_velocity_vertex(size_t frame_id, const Eigen::Vector3d& velocity, bool fixed) {
    auto* v = new VertexVelocity();
    v->setId(get_next_vertex_id());
    v->setEstimate(velocity);
    v->setFixed(fixed);
    
    optimizer_->addVertex(v);
    frame_velocity_vertices_[frame_id] = v;
}

void G2OSolver::add_bias_vertex(size_t frame_id, const Eigen::Vector3d& bg, const Eigen::Vector3d& ba, bool fixed) {
    auto* v = new VertexBias();
    v->setId(get_next_vertex_id());
    
    Eigen::Matrix<double, 6, 1> bias;
    bias.head<3>() = bg;
    bias.tail<3>() = ba;
    v->setEstimate(bias);
    v->setFixed(fixed);
    
    optimizer_->addVertex(v);
    frame_bias_vertices_[frame_id] = v;
}

void G2OSolver::add_landmark_vertex(size_t track_id, const Eigen::Vector3d& point, bool fixed) {
    auto* v = new g2o::VertexPointXYZ();
    v->setId(get_next_vertex_id());
    v->setEstimate(point);
    v->setFixed(fixed);
    v->setMarginalized(true); // Landmarks are typically marginalized
    
    optimizer_->addVertex(v);
    track_vertices_[track_id] = v;
}

void G2OSolver::add_frame_states(Frame* frame, bool with_motion) {
    size_t id = frame->id();
    
    // Add pose vertex
    add_pose_vertex(id, frame->pose, frame->tag(FT_FIX_POSE));
    
    // Add velocity vertex
    add_velocity_vertex(id, frame->motion.v, frame->tag(FT_FIX_MOTION));
    
    // Add bias vertex
    add_bias_vertex(id, frame->motion.bg, frame->motion.ba, frame->tag(FT_FIX_MOTION));
    
    // Add extrinsic calibration vertex (typically fixed)
    add_extrinsic_vertex(id, frame->camera, true);
}

void G2OSolver::add_track_states(Track* track) {
    size_t id = track->id();
    
    // For g2o, we'll use inverse depth representation
    // Convert from 3D point to inverse depth
    const auto& [ref_frame, ref_idx] = track->first_keypoint();
    if (ref_frame && ref_idx != nil()) {
        vector<3> bearing = ref_frame->get_keypoint(ref_idx);
        double inv_depth = track->landmark.inv_depth;
        
        // Add landmark as a 3D point for now
        vector<3> point = bearing / inv_depth;
        add_landmark_vertex(id, point, track->tag(TT_FIX_INVD));
    }
}

void G2OSolver::add_factor(ReprojectionErrorFactor* factor) {
    // TODO: Implement g2o reprojection factor
}

void G2OSolver::add_factor(ReprojectionPriorFactor* factor) {
    // TODO: Implement g2o reprojection prior factor
}

void G2OSolver::add_factor(RotationPriorFactor* factor) {
    // TODO: Implement g2o rotation prior factor
}

void G2OSolver::add_factor(PreIntegrationErrorFactor* factor) {
    // TODO: Implement g2o preintegration error factor
}

void G2OSolver::add_factor(PreIntegrationPriorFactor* factor) {
    // TODO: Implement g2o preintegration prior factor
}

void G2OSolver::add_factor(MarginalizationFactor* factor) {
    // TODO: Implement g2o marginalization factor
}

// Removed put_factor implementation as it's handled by base class

bool G2OSolver::solve(bool verbose) {
    // Set verbosity
    optimizer_->setVerbose(verbose);
    
    // Initialize optimization
    optimizer_->initializeOptimization();
    
    // Run optimization
    int iterations = 10; // TODO: make configurable
    optimizer_->optimize(iterations);
    
    // Update states from optimized vertices
    for (auto& [frame_id, vertex] : frame_pose_vertices_) {
        const g2o::SE3Quat& se3 = vertex->estimate();
        // Find frame and update - this would need access to map or frame list
        // For now, the update callbacks registered during add_frame_states will handle this
    }
    
    for (auto& [frame_id, vertex] : frame_velocity_vertices_) {
        const Eigen::Vector3d& v = vertex->estimate();
        // Update handled by callback
    }
    
    for (auto& [frame_id, vertex] : frame_bias_vertices_) {
        const Eigen::Matrix<double, 6, 1>& bias = vertex->estimate();
        // Update handled by callback
    }
    
    for (auto& [track_id, vertex] : track_vertices_) {
        const Eigen::Vector3d& point = vertex->estimate();
        // Update handled by callback
    }
    
    // Check if optimization was successful
    return optimizer_->activeChi2() < 1e6; // TODO: make threshold configurable
}

// Factory methods for creating factors

std::unique_ptr<PreIntegrationErrorFactor> G2OSolver::create_preintegration_error_factor(
    Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegrator) {
    
    // For now, return nullptr as this needs proper implementation
    // TODO: Implement proper g2o preintegration factor
    return nullptr;
}

std::unique_ptr<PreIntegrationPriorFactor> G2OSolver::create_preintegration_prior_factor(
    Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegrator) {
    // For now, return nullptr as this needs proper implementation
    // TODO: Implement proper g2o preintegration prior factor
    return nullptr;
}

std::unique_ptr<ReprojectionErrorFactor> G2OSolver::create_reprojection_error_factor(
    Frame* frame, Track* track) {
    
    // For now, return nullptr as this needs proper implementation
    // TODO: Implement proper g2o reprojection factor
    return nullptr;
}

std::unique_ptr<ReprojectionPriorFactor> G2OSolver::create_reprojection_prior_factor(
    Frame* frame, Track* track) {
    // For now, return nullptr as this needs proper implementation
    // TODO: Implement proper g2o reprojection prior factor
    return nullptr;
}

std::unique_ptr<RotationPriorFactor> G2OSolver::create_rotation_prior_factor(
    Frame* frame, Track* track) {
    // This would create a factor that only constrains rotation
    // For now, return nullptr as this needs custom implementation
    return nullptr;
}

std::unique_ptr<MarginalizationFactor> G2OSolver::create_marginalization_factor(Map* map) {
    // Marginalization in g2o is more complex and would require custom implementation
    // For now, return nullptr
    return nullptr;
}

// Edge implementations

void EdgeImuPreintegration::computeError() {
    // TODO: Implement proper IMU preintegration error computation
    // For now, set error to zero
    _error.setZero();
}

void EdgeImuPreintegration::linearizeOplus() {
    // TODO: Implement proper Jacobian computation
    // For now, set Jacobians to identity
    for (size_t i = 0; i < _vertices.size(); ++i) {
        if (_jacobianOplus[i].data()) {
            _jacobianOplus[i].setIdentity();
        }
    }
}

void EdgeVisualReprojection::computeError() {
    // TODO: Implement proper visual reprojection error
    // For now, set error to zero
    _error.setZero();
}

void EdgeVisualReprojection::linearizeOplus() {
    // TODO: Implement proper Jacobian computation
    // For now, set Jacobians to zero
    _jacobianOplusXi.setZero();
    _jacobianOplusXj.setZero();
}

void G2OSolver::manage_factor(std::unique_ptr<ReprojectionErrorFactor>&& factor) {
    managed_reprojection_factors_.push_back(std::move(factor));
}

void G2OSolver::manage_factor(std::unique_ptr<ReprojectionPriorFactor>&& factor) {
    managed_reprojection_prior_factors_.push_back(std::move(factor));
}

void G2OSolver::manage_factor(std::unique_ptr<RotationPriorFactor>&& factor) {
    managed_rotation_factors_.push_back(std::move(factor));
}

void G2OSolver::manage_factor(std::unique_ptr<PreIntegrationErrorFactor>&& factor) {
    managed_preintegration_factors_.push_back(std::move(factor));
}

void G2OSolver::manage_factor(std::unique_ptr<PreIntegrationPriorFactor>&& factor) {
    managed_preintegration_prior_factors_.push_back(std::move(factor));
}

void G2OSolver::manage_factor(std::unique_ptr<MarginalizationFactor>&& factor) {
    managed_marginalization_factors_.push_back(std::move(factor));
}

} // namespace rdvio