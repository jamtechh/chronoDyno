// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Front and Rear generic vehicle suspension subsystems (reduced double A-arm)
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishboneReduced) and origins at the midpoint
// between the lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef GENERIC_DOUBLEWISHBONE_REDUCED_H
#define GENERIC_DOUBLEWISHBONE_REDUCED_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishboneReduced.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Reduced double wishbone front suspension for the generic vehicle.
/// The control arms are modeled using distance constraints.
class CH_MODELS_API Generic_DoubleWishboneReducedFront : public ChDoubleWishboneReduced {
  public:
    Generic_DoubleWishboneReducedFront(const std::string& name);
    ~Generic_DoubleWishboneReducedFront();

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector3d getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;

    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_uprightInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

// -----------------------------------------------------------------------------

/// Reduced double wishbone rear suspension for the generic vehicle.
/// The control arms are modeled using distance constraints.
class CH_MODELS_API Generic_DoubleWishboneReducedRear : public ChDoubleWishboneReduced {
  public:
    Generic_DoubleWishboneReducedRear(const std::string& name);
    ~Generic_DoubleWishboneReducedRear();

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector3d getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;

    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_uprightInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
