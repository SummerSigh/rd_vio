#pragma once

#include <rdvio/estimation/solver.h>
#include <rdvio/geometry/stereo.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/edge_project_xyz.h>
#include <memory>
#include <unordered_map>

namespace rdvio {

class G2OSolver : public Solver {
public:
    G2OSolver();
    ~G2OSolver() override;

    void add_frame_states(Frame* frame, bool with_motion = true) override;
    void add_track_states(Track* track) override;
    
    void add_factor(ReprojectionErrorFactor* factor) override;
    void add_factor(ReprojectionPriorFactor* factor) override;
    void add_factor(RotationPriorFactor* factor) override;
    void add_factor(PreIntegrationErrorFactor* factor) override;
    void add_factor(PreIntegrationPriorFactor* factor) override;
    void add_factor(MarginalizationFactor* factor) override;
    
    bool solve(bool verbose = false) override;
    
    static std::unique_ptr<PreIntegrationErrorFactor> create_preintegration_error_factor(
        Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegrator);
    static std::unique_ptr<PreIntegrationPriorFactor> create_preintegration_prior_factor(
        Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegrator);
    static std::unique_ptr<ReprojectionErrorFactor> create_reprojection_error_factor(
        Frame* frame, Track* track);
    static std::unique_ptr<ReprojectionPriorFactor> create_reprojection_prior_factor(
        Frame* frame, Track* track);
    static std::unique_ptr<RotationPriorFactor> create_rotation_prior_factor(
        Frame* frame, Track* track);
    static std::unique_ptr<MarginalizationFactor> create_marginalization_factor(
        Map* map);

protected:
    void manage_factor(std::unique_ptr<ReprojectionErrorFactor>&& factor) override;
    void manage_factor(std::unique_ptr<ReprojectionPriorFactor>&& factor) override;
    void manage_factor(std::unique_ptr<RotationPriorFactor>&& factor) override;
    void manage_factor(std::unique_ptr<PreIntegrationErrorFactor>&& factor) override;
    void manage_factor(std::unique_ptr<PreIntegrationPriorFactor>&& factor) override;
    void manage_factor(std::unique_ptr<MarginalizationFactor>&& factor) override;

private:
    struct G2OFactorWrapper {
        g2o::OptimizableGraph::Edge* edge = nullptr;
        
        G2OFactorWrapper(g2o::OptimizableGraph::Edge* e) : edge(e) {}
        ~G2OFactorWrapper() {
            if (edge && !edge->graph()) {
                delete edge;
            }
        }
    };

    std::unique_ptr<g2o::SparseOptimizer> optimizer_;
    std::unordered_map<size_t, g2o::VertexSE3Expmap*> frame_pose_vertices_;
    std::unordered_map<size_t, g2o::VertexSE3Expmap*> frame_extrinsic_vertices_;
    std::unordered_map<size_t, g2o::BaseVertex<3, Eigen::Vector3d>*> frame_velocity_vertices_;
    std::unordered_map<size_t, g2o::BaseVertex<6, Eigen::Matrix<double, 6, 1>>*> frame_bias_vertices_;
    std::unordered_map<size_t, g2o::VertexPointXYZ*> track_vertices_;
    
    // Managed factors
    std::vector<std::unique_ptr<ReprojectionErrorFactor>> managed_reprojection_factors_;
    std::vector<std::unique_ptr<ReprojectionPriorFactor>> managed_reprojection_prior_factors_;
    std::vector<std::unique_ptr<RotationPriorFactor>> managed_rotation_factors_;
    std::vector<std::unique_ptr<PreIntegrationErrorFactor>> managed_preintegration_factors_;
    std::vector<std::unique_ptr<PreIntegrationPriorFactor>> managed_preintegration_prior_factors_;
    std::vector<std::unique_ptr<MarginalizationFactor>> managed_marginalization_factors_;

    int next_vertex_id_ = 0;
    int get_next_vertex_id() { return next_vertex_id_++; }
    
    void setup_optimizer();
    void add_pose_vertex(size_t frame_id, const PoseState& pose, bool fixed = false);
    void add_extrinsic_vertex(size_t frame_id, const ExtrinsicParam& extrinsic, bool fixed = true);
    void add_velocity_vertex(size_t frame_id, const Eigen::Vector3d& velocity, bool fixed = false);
    void add_bias_vertex(size_t frame_id, const Eigen::Vector3d& bg, const Eigen::Vector3d& ba, bool fixed = false);
    void add_landmark_vertex(size_t track_id, const Eigen::Vector3d& point, bool fixed = false);
};

// Custom vertex types for VIO
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    VertexVelocity() = default;
    
    virtual void setToOriginImpl() override {
        _estimate.setZero();
    }
    
    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Map<const Eigen::Vector3d>(update);
    }
    
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }
};

class VertexBias : public g2o::BaseVertex<6, Eigen::Matrix<double, 6, 1>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    VertexBias() = default;
    
    virtual void setToOriginImpl() override {
        _estimate.setZero();
    }
    
    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Map<const Eigen::Matrix<double, 6, 1>>(update);
    }
    
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }
};

// Custom edge for IMU preintegration
class EdgeImuPreintegration : public g2o::BaseMultiEdge<15, Eigen::Matrix<double, 15, 1>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeImuPreintegration() {
        resize(6); // pose_i, vel_i, bias_i, pose_j, vel_j, bias_j
    }
    
    void setPreintegrator(std::shared_ptr<PreIntegrator> preint) {
        preintegrator_ = preint;
    }
    
    virtual void computeError() override;
    virtual void linearizeOplus() override;
    
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }
    
private:
    std::shared_ptr<PreIntegrator> preintegrator_;
};

// Custom edge for visual reprojection
class EdgeVisualReprojection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeVisualReprojection() = default;
    
    void setCamera(const matrix<3>& K, const ExtrinsicParams& extrinsic) {
        K_ = K;
        extrinsic_ = extrinsic;
    }
    
    void setMeasurement(const Eigen::Vector3d& bearing) {
        bearing_ = bearing;
        _measurement = apply_k(bearing, K_);
    }
    
    virtual void computeError() override;
    virtual void linearizeOplus() override;
    
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }
    
private:
    matrix<3> K_;
    ExtrinsicParams extrinsic_;
    Eigen::Vector3d bearing_;
};

} // namespace rdvio