// clang-format off
/*
** Copyright 2021 Robotic Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// clang-format on

#pragma once

#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include "maxon_epos_ethercat_sdk/ModeOfOperationEnum.hpp"
#include "maxon_epos_ethercat_sdk/PdoTypeEnum.hpp"

namespace maxon {
class Configuration {
 public:
  /*
  Angles: [rad]           --> Cinverted to inc (via Encoder resolution and sensor placement) when directly interfacing hardware
  Velocities: [rad/s]     --> Converted to microrpm/s when directly interfacing hardware
  Accelerations: [rpm/s]  --> Same unit when interfacing hardware
  */


  std::vector<ModeOfOperationEnum> modesOfOperation = {ModeOfOperationEnum::NA};
  unsigned int configRunSdoVerifyTimeout{20000};
  bool printDebugMessages{true};
  unsigned int driveStateChangeMinTimeout{20000};
  unsigned int minNumberOfSuccessfulTargetStateReadings{10};
  unsigned int driveStateChangeMaxTimeout{300000};
  bool forceAppendEqualError{true};
  bool forceAppendEqualFault{false};
  unsigned int errorStorageCapacity{100};
  unsigned int faultStorageCapacity{100};

  bool useRawCommands{false};
  bool useControllerParams{false};

  int32_t positionEncoderResolution{1};     // [inc/rev]
  double gearRatio{1};
  double sensorPositionCorrection{1}; // either 1 or gearRatio (depending on sensor positioning)


  double motorConstant{1};
  double workVoltage{48.0};
  double speedConstant{0};
  uint32_t polePairs{11};
  double nominalCurrentA{0};                // [A]
  double torqueConstantNmA{0};
  double maxCurrentA{0};                    // [A]

  double minPosition{0};                    // [rad]
  double maxPosition{0};                    // [rad]

  double maxProfileVelocity{0};             // [rad/s]
  uint32_t quickStopDecel{10000};           // [rpm/s]
  uint32_t profileDecel{10000};             // [rpm/s]
  uint32_t followErrorWindow{2000};         // [inc]
  double currentPGainSI{1.171880};
  double currentIGainSI{3906.250};
  double positionPGainSI{1.5};
  double positionIGainSI{0.78};
  double positionDGainSI{0.016};
  double velocityPGainSI{0.02};
  double velocityIGainSI{0.5};


  int8_t homingMethod {-3};
  uint16_t currentThreshold{0};
  uint32_t homingSpeeds{10};
  uint32_t homingAccel{1};
  uint32_t homeOffsetMoveDistance{0};
  uint32_t homePosition{0};


 public:
  // stream operator
  friend std::ostream& operator<<(std::ostream& os,
                                  const Configuration& configuration);

  /*!
   * @brief Check whether the parameters are sane.
   * Prints a list of the checks and whether they failed or passed.
   * @param[in] silent If true: Do not print. Only return the success of the
   * test.
   * @return true if the checks are successful.
   */
  bool sanityCheck(bool silent = false) const;

  std::pair<RxPdoTypeEnum, TxPdoTypeEnum> getPdoTypeSolution() const;
};

}  // namespace maxon
