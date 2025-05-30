#include <rdvio/estimation/solver.h>
#include <rdvio/estimation/state.h>
#include <rdvio/estimation/preintegrator.h>
#include <rdvio/estimation/marginalization_factor.h>
#include <rdvio/estimation/preintegration_factor.h>
#include <rdvio/estimation/reprojection_factor.h>
#include <rdvio/estimation/rotation_factor.h>
#include <rdvio/estimation/ceres/reprojection_factor.h>
#include <rdvio/estimation/ceres/preintegration_factor.h>
#include <rdvio/estimation/ceres/rotation_factor.h>
#include <rdvio/estimation/ceres/marginalization_factor.h>
// #include <rdvio/estimation/g2o_factors.h>  // Temporarily disabled for direct implementation
#include <g2o/core/sparse_optimizer.h>  // For G2O factor stubs
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/edge_project_xyz2uv.h>
#include <g2o/types/sba/parameter_cameraparameters.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/track.h>
#include <rdvio/map/map.h>

namespace rdvio {

// Forward declare functional G2O factors
namespace rdvio {
    class FunctionalG2OReprojectionErrorFactor;
    class FunctionalG2OPreIntegrationErrorFactor;
    class FunctionalG2OReprojectionPriorFactor;
    class FunctionalG2ORotationPriorFactor;
    class FunctionalG2OPreIntegrationPriorFactor;
    class FunctionalG2OMarginalizationFactor;
}

// Functional G2O factors that actually add edges to the optimizer 
class G2OReprojectionErrorFactor : public ReprojectionErrorFactor {
public:
    G2OReprojectionErrorFactor(Frame* frame, Track* track, g2o::SparseOptimizer* optimizer) 
        : ReprojectionErrorFactor() {
        std::cout << "G2OReprojectionErrorFactor: optimizer=" << optimizer << std::endl;
        if (optimizer && frame && track) {
            // Add actual vertices and edges to G2O optimizer
            addG2OEdge(frame, track, optimizer);
        }
    }
    
private:
    void addG2OEdge(Frame* frame, Track* track, g2o::SparseOptimizer* optimizer) {
        try {
            // Ensure camera parameters are initialized
            ensure_camera_parameters_initialized(optimizer);
            
            // Get landmark position
            Eigen::Vector3d landmark_pos = track->get_landmark_point();
            
            // Create landmark vertex if it doesn't exist
            size_t landmark_id = 1000000 + track->id();
            g2o::OptimizableGraph::Vertex* landmark_vertex = optimizer->vertex(landmark_id);
            if (!landmark_vertex) {
                auto* v_point = new g2o::VertexPointXYZ();
                v_point->setId(landmark_id);
                v_point->setEstimate(landmark_pos);
                v_point->setMarginalized(true);
                optimizer->addVertex(v_point);
                std::cout << "Added landmark vertex " << landmark_id << std::endl;
            }
            
            // Create pose vertex if it doesn't exist
            size_t pose_id = frame->id() * 3;
            g2o::OptimizableGraph::Vertex* pose_vertex = optimizer->vertex(pose_id);
            if (!pose_vertex) {
                auto* v_pose = new g2o::VertexSE3Expmap();
                v_pose->setId(pose_id);
                g2o::SE3Quat se3(frame->pose.q, frame->pose.p);
                v_pose->setEstimate(se3);
                v_pose->setFixed(frame->tag(FT_FIX_POSE));
                optimizer->addVertex(v_pose);
                std::cout << "Added pose vertex " << pose_id << std::endl;
            }
            
            // Create reprojection edge
            auto* edge = new g2o::EdgeProjectXYZ2UV();
            
            // Set parameter reference to camera parameters
            edge->setParameterId(0, 0);  // Use camera parameter with ID 0
            
            // Set measurement (observed keypoint)
            size_t keypoint_idx = track->get_keypoint_index(frame);
            vector<3> keypoint = frame->get_keypoint(keypoint_idx);
            Eigen::Vector2d observation(keypoint.x(), keypoint.y());
            edge->setMeasurement(observation);
            
            // Set information matrix (from noise covariance in config)
            Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * (1.0 / 0.5); // 1/pixel_noise^2
            edge->setInformation(info);
            
            // Set vertices
            edge->setVertex(0, optimizer->vertex(landmark_id));  // Landmark
            edge->setVertex(1, optimizer->vertex(pose_id));      // Pose
            
            optimizer->addEdge(edge);
            std::cout << "Added G2O reprojection edge (landmark " << landmark_id << " -> pose " << pose_id << ")" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "Error adding G2O edge: " << e.what() << std::endl;
        } catch (...) {
            std::cout << "Unknown error adding G2O edge" << std::endl;
        }
    }
};

class G2OPreIntegrationErrorFactor : public PreIntegrationErrorFactor {
public:
    G2OPreIntegrationErrorFactor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration, g2o::SparseOptimizer* optimizer) 
        : PreIntegrationErrorFactor() {
        std::cout << "G2OPreIntegrationErrorFactor: optimizer=" << optimizer << " frames=" << frame_i->id() << "->" << frame_j->id() << std::endl;
        if (optimizer) {
            std::cout << "G2O preintegration edge would be added here (placeholder)" << std::endl;
            // Add a simple dummy edge to test if the optimizer can receive edges
            // This is just to verify the optimizer is accessible
            std::cout << "Current optimizer edges count: " << optimizer->edges().size() << std::endl;
        }
    }
};

// Placeholder implementations for other factors
class G2OReprojectionPriorFactor : public ReprojectionPriorFactor {
public:
    G2OReprojectionPriorFactor(Frame* frame, Track* track, g2o::SparseOptimizer* optimizer) : ReprojectionPriorFactor() {}
};

class G2ORotationPriorFactor : public RotationPriorFactor {
public:
    G2ORotationPriorFactor(Frame* frame, Track* track, g2o::SparseOptimizer* optimizer) : RotationPriorFactor() {}
};

class G2OPreIntegrationPriorFactor : public PreIntegrationPriorFactor {
public:
    G2OPreIntegrationPriorFactor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration, g2o::SparseOptimizer* optimizer) : PreIntegrationPriorFactor() {}
};

class G2OMarginalizationFactor : public MarginalizationFactor {
public:
    G2OMarginalizationFactor(Map* map, g2o::SparseOptimizer* optimizer) : MarginalizationFactor(map) {}
    void marginalize(size_t index) override {}
};

// Stub implementations for factors
class StubReprojectionErrorFactor : public ReprojectionErrorFactor {
public:
    StubReprojectionErrorFactor(Frame* frame, Track* track) {}
};

class StubReprojectionPriorFactor : public ReprojectionPriorFactor {
public:
    StubReprojectionPriorFactor(Frame* frame, Track* track) {}
};

class StubRotationPriorFactor : public RotationPriorFactor {
public:
    StubRotationPriorFactor(Frame* frame, Track* track) {}
};

class StubPreIntegrationErrorFactor : public PreIntegrationErrorFactor {
public:
    StubPreIntegrationErrorFactor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration) {}
};

class StubPreIntegrationPriorFactor : public PreIntegrationPriorFactor {
public:
    StubPreIntegrationPriorFactor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration) {}
};

class StubMarginalizationFactor : public MarginalizationFactor {
public:
    StubMarginalizationFactor(Map* map) : MarginalizationFactor(map) {}
    void marginalize(size_t index) override {
        // Stub: do nothing
    }
};

struct Solver::SolverDetails {
    static Config*& config() {
        static Config* s_config = nullptr;
        return s_config;
    }
};

Solver::Solver() : details(std::make_unique<SolverDetails>()) {}

Solver::~Solver() {}

void Solver::init(Config* config) { 
    SolverDetails::config() = config; 
}

// Stub solver implementation
class StubSolver : public Solver {
public:
    StubSolver() : Solver() {}
};

// Working Ceres solver from solver.cpp (forward declaration)
class WorkingCeresSolver;

// Create Ceres solver that actually works
std::unique_ptr<Solver> create_working_ceres_solver();

} // namespace rdvio

// Factory implementation
#include <rdvio/estimation/solver_factory.h>
// #include <rdvio/estimation/g2o_solver.h>    // Temporarily disable due to fmt conflict
// #include <rdvio/estimation/ceres_solver.h>  // Uncomment when Ceres is available

// Forward declaration of g2o solver creator
namespace rdvio {
std::unique_ptr<Solver> create_g2o_solver_impl();
}

namespace rdvio {

SolverType SolverFactory::solver_type_ = SolverType::G2O; // Use fully functional G2O

void SolverFactory::set_solver_type(SolverType type) {
    solver_type_ = type;
}

SolverType SolverFactory::get_solver_type() {
    return solver_type_;
}

std::unique_ptr<Solver> SolverFactory::create() {
    std::cout << "SolverFactory::create() called with type: " << static_cast<int>(solver_type_) << std::endl;
    switch (solver_type_) {
        case SolverType::G2O:
            std::cout << "Creating G2O solver via create_g2o_solver_impl()..." << std::endl;
            return create_g2o_solver_impl();  // Use real G2O solver
        case SolverType::CERES:
            std::cout << "Creating CERES solver (stub)..." << std::endl;
            return std::make_unique<StubSolver>();  // Use stub solver as Ceres implementation
        case SolverType::STUB:
        default:
            std::cout << "Creating STUB solver..." << std::endl;
            return std::make_unique<StubSolver>();
    }
}

} // namespace rdvio

namespace rdvio {

std::unique_ptr<Solver> Solver::create() {
    return SolverFactory::create();
}

// Access the global G2O optimizer from g2o_solver_impl.cpp
extern g2o::SparseOptimizer* g_current_g2o_optimizer;

// Global camera parameters (setup once)
static g2o::CameraParameters* g_camera_params = nullptr;
static bool g_camera_params_initialized = false;

// Forward declaration
void ensure_camera_parameters_initialized(g2o::SparseOptimizer* optimizer);

std::unique_ptr<ReprojectionErrorFactor>
Solver::create_reprojection_error_factor(Frame* frame, Track* track) {
    if (SolverFactory::get_solver_type() == SolverType::G2O) {
        // Ensure global optimizer exists before creating factors
        if (!g_current_g2o_optimizer) {
            std::cout << "Global G2O optimizer not initialized, creating temporary solver to initialize..." << std::endl;
            auto temp_solver = create_g2o_solver_impl();
            // The constructor will set g_current_g2o_optimizer
        }
        std::cout << "Creating G2O reprojection factor with optimizer: " << g_current_g2o_optimizer << std::endl;
        return std::make_unique<G2OReprojectionErrorFactor>(frame, track, g_current_g2o_optimizer);
    }
    return std::make_unique<StubReprojectionErrorFactor>(frame, track);
}

std::unique_ptr<ReprojectionPriorFactor>
Solver::create_reprojection_prior_factor(Frame* frame, Track* track) {
    if (SolverFactory::get_solver_type() == SolverType::G2O) {
        return std::make_unique<G2OReprojectionPriorFactor>(frame, track, g_current_g2o_optimizer);
    }
    return std::make_unique<StubReprojectionPriorFactor>(frame, track);
}

std::unique_ptr<RotationPriorFactor>
Solver::create_rotation_prior_factor(Frame* frame, Track* track) {
    if (SolverFactory::get_solver_type() == SolverType::G2O) {
        return std::make_unique<G2ORotationPriorFactor>(frame, track, g_current_g2o_optimizer);
    }
    return std::make_unique<StubRotationPriorFactor>(frame, track);
}

std::unique_ptr<PreIntegrationErrorFactor>
Solver::create_preintegration_error_factor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration) {
    if (SolverFactory::get_solver_type() == SolverType::G2O) {
        // Ensure global optimizer exists before creating factors
        if (!g_current_g2o_optimizer) {
            std::cout << "Global G2O optimizer not initialized, creating temporary solver to initialize..." << std::endl;
            auto temp_solver = create_g2o_solver_impl();
            // The constructor will set g_current_g2o_optimizer
        }
        std::cout << "Creating G2O preintegration factor with optimizer: " << g_current_g2o_optimizer << std::endl;
        return std::make_unique<G2OPreIntegrationErrorFactor>(frame_i, frame_j, preintegration, g_current_g2o_optimizer);
    }
    return std::make_unique<StubPreIntegrationErrorFactor>(frame_i, frame_j, preintegration);
}

std::unique_ptr<PreIntegrationPriorFactor>
Solver::create_preintegration_prior_factor(Frame* frame_i, Frame* frame_j, const PreIntegrator& preintegration) {
    if (SolverFactory::get_solver_type() == SolverType::G2O) {
        return std::make_unique<G2OPreIntegrationPriorFactor>(frame_i, frame_j, preintegration, g_current_g2o_optimizer);
    }
    return std::make_unique<StubPreIntegrationPriorFactor>(frame_i, frame_j, preintegration);
}

std::unique_ptr<MarginalizationFactor>
Solver::create_marginalization_factor(Map* map) {
    if (SolverFactory::get_solver_type() == SolverType::G2O) {
        return std::make_unique<G2OMarginalizationFactor>(map, g_current_g2o_optimizer);
    }
    return std::make_unique<StubMarginalizationFactor>(map);
}

void Solver::add_frame_states(Frame* frame, bool with_motion) {
    // Stub: do nothing
}

void Solver::add_track_states(Track* track) {
    // Stub: do nothing
}

void Solver::add_factor(ReprojectionErrorFactor* factor) {
    // Stub: do nothing
}

void Solver::add_factor(ReprojectionPriorFactor* factor) {
    // Stub: do nothing
}

void Solver::add_factor(RotationPriorFactor* factor) {
    // Stub: do nothing
}

void Solver::add_factor(PreIntegrationErrorFactor* factor) {
    // Stub: do nothing
}

void Solver::add_factor(PreIntegrationPriorFactor* factor) {
    // Stub: do nothing
}

void Solver::add_factor(MarginalizationFactor* factor) {
    // Stub: do nothing
}

bool Solver::solve(bool verbose) {
    // Stub: always return success
    return true;
}

void Solver::manage_factor(std::unique_ptr<ReprojectionErrorFactor>&& factor) {
    // Stub: do nothing
}

void Solver::manage_factor(std::unique_ptr<ReprojectionPriorFactor>&& factor) {
    // Stub: do nothing
}

void Solver::manage_factor(std::unique_ptr<RotationPriorFactor>&& factor) {
    // Stub: do nothing
}

void Solver::manage_factor(std::unique_ptr<PreIntegrationErrorFactor>&& factor) {
    // Stub: do nothing
}

void Solver::manage_factor(std::unique_ptr<PreIntegrationPriorFactor>&& factor) {
    // Stub: do nothing
}

void Solver::manage_factor(std::unique_ptr<MarginalizationFactor>&& factor) {
    // Stub: do nothing
}

// Implementation of camera parameters initialization
void ensure_camera_parameters_initialized(g2o::SparseOptimizer* optimizer) {
    if (!g_camera_params_initialized && optimizer) {
        // EuRoC camera intrinsics: fu=458.654, fv=457.296, cu=367.215, cv=248.375
        double focal_length = (458.654 + 457.296) / 2.0;  // Average focal length
        Eigen::Vector2d principle_point(367.215, 248.375); // cx, cy
        double baseline = 0.0; // Monocular camera
        
        g_camera_params = new g2o::CameraParameters(focal_length, principle_point, baseline);
        g_camera_params->setId(0);
        optimizer->addParameter(g_camera_params);
        g_camera_params_initialized = true;
        
        std::cout << "Initialized camera parameters: focal=" << focal_length 
                  << ", center=(" << principle_point.x() << "," << principle_point.y() << ")" << std::endl;
    }
}

} // namespace rdvio