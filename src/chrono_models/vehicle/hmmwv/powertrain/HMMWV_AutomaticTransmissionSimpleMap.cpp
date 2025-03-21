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
// Automatic transmssion model for the HMMWV vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

const double rpm2rads = CH_PI / 30;

HMMWV_AutomaticTransmissionSimpleMap::HMMWV_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void HMMWV_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.2;

    fwd.push_back(0.1708);
    fwd.push_back(0.2791);
    fwd.push_back(0.4218);
    fwd.push_back(0.6223);
    fwd.push_back(1.0173);
    fwd.push_back(1.5361);
}

void HMMWV_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2700 * rpm2rads));
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
