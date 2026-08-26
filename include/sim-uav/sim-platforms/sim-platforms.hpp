/***********************************************************************************************************************
 * Copyright (c) 2025 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        sim-platforms.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header only file to include all the platforms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_PLATFORMS_HPP_
#define SIM_PLATFORMS_HPP_

#include "sim-uav.hpp"

/* =====================================================================================================================
  sim-platforms.hpp

  Purpose:
    - Central registry for all UAV platform class includes.
    - This ensures that `sim-bridge.hpp` (and related simulation code) 
      can discover and use all available UAV platform definitions.

  How to add a new platform:
    1. Include your derived UAV class header in the "Includes" section below.
    2. Add a new `bool <platform_name>{};` member to the `platforms` struct.
       !! The member name **must exactly match** the name used in your YAML config. !!
    3. Update the `members()` helper inside the struct with the new entry  
       so that `asMap()` / `asVector()` can iterate it automatically.
    4. Update `sim-bridge.hpp` and `sim-bridge.cpp` if any platform‑specific
       logic is needed for initialization.
    5. Verify that the platform is listed in the YAML configuration file and  
       that it exists in the ACSL Physics Sim database with matching `.hpp`/`.cpp`
       implementation.

  Notes:
    - Keep includes grouped and sorted alphabetically for maintainability.
    - This file intentionally only contains header includes and the **platforms** 
      struct — no functional logic belongs here.
==================================================================================================================== */

// -----------------------------------------------------------------------------
// Includes for the different UAV platforms (add more as needed)
// -----------------------------------------------------------------------------
#include "sim-qrbp.hpp"
// #include "sim-x8copter.hpp"
#include "sim-tailsitter.hpp"
#include "sim-quadm.hpp"
#include "sim-x8.hpp"

namespace _acsl_
{

namespace _bridge_
{

// -----------------------------------------------------------------------------
// Structure: platforms
//
// Holds boolean flags indicating which UAV platforms are enabled (true) or
// disabled (false) for the current simulation run.
//
// How to add a new platform:
//   1) Include the header for the new platform in sim-platforms.hpp
//   2) Add a bool member to this struct:
//        bool mynewdrone{};   // My cool new drone
//   3) Add it to the `asVectorRef()` helper below:
//        {"mynewdrone", mynewdrone}
//   4) Add it to the createSelectedUAV function appropriately
//   5) Add it to your `sim-config.yaml`:
//        platform:
//          mynewdrone: true 
//   6) Implement platform-specific logic in sim-bridge.cpp if required.
//
// How to remove a platform:
//   1) Remove the bool from this struct.
//   2) Remove it from `asVectorRef()`.
//   3) Remove it from the createSelectedUAV function appropriately.
//   4) Remove it from sim-config.yaml and any related headers/logic.
// -----------------------------------------------------------------------------
struct platforms {
    bool qrbp{};         // Quadrotor Biplane
    bool tailsitter{};   // Tailsitter (scaled/lighter version of the QRBP)
    bool quadm{};        // Medium Quadcopter
    bool x8{};           // X8 coaxial drone


    // -------------------------------------------------------------------------
    // Returns all platform members as {name, reference} pairs
    // for dynamic iteration and assignment.
    //
    // NOTE:
    //   Use `auto&` for writable access:
    //      for (auto& [name, val] : uavs.asVectorRef()) { val = true; }
    //   Use `const auto&` for read-only access:
    //      for (const auto& [name, val] : uavs.asVectorRef()) { if (val) {...} }
    // -------------------------------------------------------------------------
    std::vector<std::pair<std::string, bool&>> asVectorRef() {
        return {
            {"qrbp", qrbp},
            {"tailsitter", tailsitter},
            {"quadm", quadm},
            {"x8", x8},
        };
    }

    // -------------------------------------------------------------------------
    // Factory: createSelectedUAV()
    //
    // Purpose:
    //   Instantiate the UAV object corresponding to the active platform and
    //   return it as a polymorphic std::unique_ptr<simuavbase>.
    //
    // Parameters:
    //   sys - Reference to the Chrono physics system (ChSystemNSC) to be passed
    //         into the UAV constructor.
    //
    // Returns:
    //   std::unique_ptr<_acsl_::_uav_::simuavbase> pointing to the constructed
    //   platform instance.
    //
    // Workflow:
    //   1. Validate that exactly one platform is selected.
    //   2. Based on which boolean is true, construct the matching platform class.
    //   3. Return the base-class pointer (simuavbase) to the caller.
    //
    // Notes:
    //   - Requires that the derived platform class headers are included
    //     in sim-platforms.hpp.
    //   - Exits or throws an error if zero or more than one platforms
    //     are selected.
    // -------------------------------------------------------------------------
    std::unique_ptr< ::_acsl_::_uav_::simuavbase >
    createSelectedUAV(chrono::ChSystemNSC& sys) const {
        std::string active = validateExclusiveSelection();

        if (active == "qrbp") {
            _message_::SIMULATOR_INFO("[SIMBRG]: ATTACHING QRBP TO THE PHYSICS SYSTEM");
            return std::unique_ptr< ::_acsl_::_uav_::simuavbase >( new ::_acsl_::_qrbp_::simqrbp(sys) );
        }
        else if (active == "tailsitter") {
            _message_::SIMULATOR_INFO("[SIMBRG]: ATTACHING TAILSITTER TO THE PHYSICS SYSTEM");
            return std::unique_ptr< ::_acsl_::_uav_::simuavbase >( new ::_acsl_::_tailsitter_::simtailsitter(sys) );
        }
        else if (active == "quadm") {
            _message_::SIMULATOR_INFO("[SIMBRG]: ATTACHING MEDIUM QUADCOPTER TO THE PHYSICS SYSTEM");
            return std::unique_ptr< ::_acsl_::_uav_::simuavbase >( new ::_acsl_::_quadm_::simquadm(sys) );
        }
        else if (active == "x8") {
            _message_::SIMULATOR_INFO("ATTACHING X8COPTER TO THE PHYSICS SYSTEM");
            return std::unique_ptr< ::_acsl_::_uav_::simuavbase >(
                new ::_acsl_::_x8_::simx8(sys)
            );
        }

        _message_::SIMULATOR_ERROR("[SIMBRG]: ACTIVE PLATFORM NAME NOT RECOGNIZED: " + active);
        return nullptr;
    }


    // -------------------------------------------------------------------------
    // Validates that exactly ONE platform is set to true.
    // Returns the active platform name.
    // Throws/logs if zero or more than one are true.
    // -------------------------------------------------------------------------
    std::string validateExclusiveSelection() const {
        std::string active;
        int count = 0;
        for (const auto& [name, val] : const_cast<platforms*>(this)->asVectorRef()) {
            if (val) {
                active = name;
                count++;
            }
        }
        if (count == 0) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: NO UAV PLATFORM SELECTED — EXACTLY ONE MUST BE SET TO TRUE.");
        } else if (count > 1) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: MULTIPLE UAV PLATFORMS SELECTED — EXACTLY ONE MUST BE SET TO TRUE.");
        }
        return active;
    }
    
};




}   // namespace _bridge_

}   // namespace _acsl_

#endif // SIM_PLATFORMS_HPP_