///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh, IRI: CSIC-UPC
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_FACTORY_COST_HPP_
#define EAGLE_MPC_FACTORY_COST_HPP_

#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>

#include "pinocchio/spatial/se3.hpp"

#include "crocoddyl/core/cost-base.hpp"
#include "crocoddyl/core/costs/residual.hpp"

#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/core/residuals/control.hpp"

#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/frame-velocity.hpp"
#include "crocoddyl/multibody/residuals/frame-translation.hpp"
#include "crocoddyl/multibody/residuals/frame-rotation.hpp"
#include "crocoddyl/multibody/residuals/contact-friction-cone.hpp"

#include "eagle_mpc/stage.hpp"
#include "eagle_mpc/utils/params_server.hpp"
#include "eagle_mpc/factory/activation.hpp"

namespace eagle_mpc
{
// enum class CostModelTypes {
//     CostModelState,
//     CostModelControl,
//     CostModelFramePlacement,
//     CostModelFrameTranslation,
//     CostModelFrameRotation,
//     CostModelFrameVelocity,
//     CostModelContactFrictionCone,
//     NbCostModelTypes
// };

// struct ResidualModelTypes {
//     enum Type {
//         ResidualModelState,
//         ResidualModelControl,
//         ResidualModelCoMPosition,
//         ResidualModelCentroidalMomentum,
//         ResidualModelFramePlacement,
//         ResidualModelFrameRotation,
//         ResidualModelFrameTranslation,
//         ResidualModelFrameVelocity,
//         ResidualModelControlGrav,
// #ifdef PINOCCHIO_WITH_HPP_FCL
//         ResidualModelPairCollision,
// #endif  // PINOCCHIO_WITH_HPP_FCL
//         NbResidualModelTypes
//     };
//     static std::vector<Type> init_all()
//     {
//         std::vector<Type> v;
//         for (int i = 0; i < NbResidualModelTypes; ++i) {
//             v.push_back((Type)i);
//         }
//         return v;
//     }
//     static const std::vector<Type> all;
// };

enum class ResidualModelTypes {
    ResidualModelState,
    ResidualModelControl,
    ResidualModelCoMPosition,
    ResidualModelCentroidalMomentum,
    ResidualModelFramePlacement,
    ResidualModelFrameRotation,
    ResidualModelFrameTranslation,
    ResidualModelFrameVelocity,
    ResidualModelControlGrav,
    ResidualModelPairCollision,
    ResidualModelContactFrictionCone,
    NbResidualModelTypes
};

static std::map<std::string, ResidualModelTypes> ResidualModelTypes_init_map()
{
    std::map<std::string, ResidualModelTypes> m;
    m.clear();
    m.insert({"ResidualModelState", ResidualModelTypes::ResidualModelState});
    m.insert({"ResidualModelControl", ResidualModelTypes::ResidualModelControl});
    m.insert({"ResidualModelFramePlacement", ResidualModelTypes::ResidualModelFramePlacement});
    m.insert({"ResidualModelFrameTranslation", ResidualModelTypes::ResidualModelFrameTranslation});
    m.insert({"ResidualModelFrameRotation", ResidualModelTypes::ResidualModelFrameRotation});
    m.insert({"ResidualModelFrameVelocity", ResidualModelTypes::ResidualModelFrameVelocity});
    m.insert({"ResidualModelContactFrictionCone", ResidualModelTypes::ResidualModelContactFrictionCone});
    return m;
}

static const std::map<std::string, ResidualModelTypes> ResidualModelTypes_map = ResidualModelTypes_init_map();

class Stage;
class ResidualModelFactory
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ResidualModelFactory();
    ~ResidualModelFactory();

    boost::shared_ptr<crocoddyl::CostModelResidual> create(const std::string&                     path_to_cost,
                                                           const boost::shared_ptr<ParamsServer>& server,
                                                           const boost::shared_ptr<crocoddyl::StateMultibody>& state,
                                                           const std::size_t&                                  nu,
                                                           ResidualModelTypes& cost_type) const;

    boost::shared_ptr<ActivationModelFactory> activation_factory_;
};

}  // namespace eagle_mpc

#endif