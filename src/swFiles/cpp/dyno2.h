#ifndef DYNO2_H
#define DYNO2_H

#include <vector>
#include <unordered_map>
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "dyno2.h"
#include "chrono/physics/ChSystemNSC.h"

void ImportSolidworksSystemCpp(chrono::ChSystemNSC& system);
void printSom();

#endif