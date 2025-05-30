#pragma once

#include <rdvio/types.h>
#include <memory>

namespace rdvio {

class Map;
class Frame;
class Track;
class PreIntegrator;

// Base class for all factors
class Factor {
public:
    virtual ~Factor() = default;
};

// Specific factor types
class ReprojectionErrorFactor : public Factor {
public:
    ReprojectionErrorFactor(std::unique_ptr<Factor> impl, Frame* frame, Track* track)
        : impl_(std::move(impl)), frame_(frame), track_(track) {}
    
    Frame* frame() const { return frame_; }
    Track* track() const { return track_; }
    Factor* implementation() { return impl_.get(); }

private:
    std::unique_ptr<Factor> impl_;
    Frame* frame_;
    Track* track_;
};

class ReprojectionPriorFactor : public ReprojectionErrorFactor {
public:
    using ReprojectionErrorFactor::ReprojectionErrorFactor;
};

class RotationPriorFactor : public Factor {
public:
    RotationPriorFactor(std::unique_ptr<Factor> impl, Frame* frame, Track* track)
        : impl_(std::move(impl)), frame_(frame), track_(track) {}
    
    Frame* frame() const { return frame_; }
    Track* track() const { return track_; }
    Factor* implementation() { return impl_.get(); }

private:
    std::unique_ptr<Factor> impl_;
    Frame* frame_;
    Track* track_;
};

class PreIntegrationErrorFactor : public Factor {
public:
    PreIntegrationErrorFactor(std::unique_ptr<Factor> impl, Frame* frame_i, Frame* frame_j)
        : impl_(std::move(impl)), frame_i_(frame_i), frame_j_(frame_j) {}
    
    Frame* frame_i() const { return frame_i_; }
    Frame* frame_j() const { return frame_j_; }
    Factor* implementation() { return impl_.get(); }

private:
    std::unique_ptr<Factor> impl_;
    Frame* frame_i_;
    Frame* frame_j_;
};

class PreIntegrationPriorFactor : public PreIntegrationErrorFactor {
public:
    using PreIntegrationErrorFactor::PreIntegrationErrorFactor;
};

class MarginalizationFactor : public Factor {
public:
    MarginalizationFactor(std::unique_ptr<Factor> impl, Map* map)
        : impl_(std::move(impl)), map_(map) {}
    
    Map* map() const { return map_; }
    Factor* implementation() { return impl_.get(); }

private:
    std::unique_ptr<Factor> impl_;
    Map* map_;
};

// Abstract solver interface
class Solver {
public:
    virtual ~Solver() = default;

    static void init(Config* config);
    static std::unique_ptr<Solver> create();

    // Factory methods for creating factors
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

    // State management
    virtual void add_frame_states(Frame* frame) = 0;
    virtual void add_track_states(Track* track) = 0;

    // Factor management
    virtual void add_factor(Factor* factor) = 0;
    virtual void put_factor(std::unique_ptr<Factor>&& factor) = 0;

    // Solve the optimization problem
    virtual double solve() = 0;

protected:
    static Config* config_;
};

} // namespace rdvio