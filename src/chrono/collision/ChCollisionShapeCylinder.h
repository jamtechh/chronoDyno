// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COLLISION_SHAPE_CYLINDER_H
#define CH_COLLISION_SHAPE_CYLINDER_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChCylinder.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision cylinder shape.
/// When added to a collision model, the cylinder is defined with its axis along the Z direction of the shape frame.
class ChApi ChCollisionShapeCylinder : public ChCollisionShape {
  public:
    ChCollisionShapeCylinder();
    ChCollisionShapeCylinder(std::shared_ptr<ChContactMaterial> material, double radius, double height);
    ChCollisionShapeCylinder(std::shared_ptr<ChContactMaterial> material, const ChCylinder& cyl);

    ~ChCollisionShapeCylinder() {}

    // Access the cylinder geometry.
    ChCylinder& GetGeometry() { return gcylinder; }

    /// Get the cylinder radius.
    double GetRadius() const { return gcylinder.GetRadius(); }

    /// Get the cylinder height.
    double GetHeight() const { return gcylinder.GetHeight(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChCylinder gcylinder;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
