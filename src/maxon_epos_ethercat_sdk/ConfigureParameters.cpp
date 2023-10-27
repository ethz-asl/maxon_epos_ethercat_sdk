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

#include <array>
#include <thread>

#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"

#include <soem_interface/EthercatBusBase.hpp>

#include <math.h>

#define RADTOMICROREV 60.0*1e6/(2*M_PI)

namespace maxon {
bool Maxon::mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum) {
  uint8_t subIndex;

  bool rxSuccess = true;
  switch (rxPdoTypeEnum) {
    case RxPdoTypeEnum::RxPdoStandard: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Standard Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Position Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCST: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Troque Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSV: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Veloctity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Toruqe/Position Mixed Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 6> objects{
          (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSPCSV: {
      MELO_INFO_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
          << "Cyclic Synchronous Toruqe/Position/Velocity Mixed Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 8> objects{
          (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoPVM: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_PROFILE_ACCELERATION << 16) | (0x00 << 8) |
              sizeof(uint32_t) * 8,
          (OD_INDEX_PROFILE_DECELERATION << 16) | (0x00 << 8) |
              sizeof(uint32_t) * 8,
          (OD_INDEX_MOTION_PROFILE_TYPE << 16) | (0x00 << 8) |
              sizeof(int16_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoPPM: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Profile Position Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 7> objects{
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_PROFILE_VELOCITY << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
          (OD_INDEX_PROFILE_ACCELERATION << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
          (OD_INDEX_PROFILE_DECELERATION << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
          (OD_INDEX_MOTION_PROFILE_TYPE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };



      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoPVMPPM:{
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Profile Position Mode/ Profile Velocity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 8> objects{
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_PROFILE_VELOCITY << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
          (OD_INDEX_PROFILE_ACCELERATION << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
          (OD_INDEX_PROFILE_DECELERATION << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
          (OD_INDEX_MOTION_PROFILE_TYPE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };



      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;      
    }
    case RxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
          "RxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
    default:  // Non-implemented type
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map unimplemented "
          "RxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
  }

  bool txSuccess = true;
  switch (txPdoTypeEnum) {
    case TxPdoTypeEnum::TxPdoStandard: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Standard Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Position Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCST: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSV: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque/Position Mixed Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSPCSV: {
      MELO_INFO_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
          << "Cyclic Synchronous Torque/Position/Velocity Mixed Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoPVM: {
      // (OD_INDEX_TORQUE_ACTUAL << 16) | (0x01 << 8) | sizeof(int16_t) * 8

      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 2> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoPPM: {
      // (OD_INDEX_TORQUE_ACTUAL << 16) | (0x01 << 8) | sizeof(int16_t) * 8

      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Profile Position Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 6> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_POSITION_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION_DISPLAY<< 16) | (0x00 << 8) | sizeof(int8_t) * 8,
          (OD_INDEX_DIG_IN_LOGIC_STATE<<16) | (0x01 << 8) | sizeof(int16_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoPVMPPM: {
      // (OD_INDEX_TORQUE_ACTUAL << 16) | (0x01 << 8) | sizeof(int16_t) * 8

      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                        << "Profile Position Mode/ Profile Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 8> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_POSITION_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CURRENT_ACTUAL << 16) | (0x02 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_DIGITAL_INPUTS << 16)  | (0x00 << 8) | sizeof(uint32_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION_DISPLAY<< 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                          configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
          "TxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
    default:  // if any case was forgotten
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map undefined "
          "TxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
  }

  return (txSuccess && rxSuccess);
}

bool Maxon::configParam() {
  bool configSuccess = true;
  uint32_t maxMotorSpeed;
  uint32_t maxProfileVelocity;
  uint32_t maxGearSpeed;
  uint32_t nominalCurrent;
  uint32_t maxCurrent;
  uint32_t torqueConstant;
  uint32_t currentPGain;
  uint32_t currentIGain;
  uint32_t positionPGain;
  uint32_t positionIGain;
  uint32_t positionDGain;
  uint32_t velocityPGain;
  uint32_t velocityIGain;


  int8_t   homingMethod;
  uint16_t currentThreshold;
  uint32_t homingSpeeds;
  uint32_t homingAccel;
  uint32_t homeOffsetMoveDistance;
  uint32_t homePosition;
  // Set velocity unit to milli revs per minute
  uint32_t velocity_unit = 0xFDB44700;
  
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_SI_UNIT_VELOCITY, 0x00, false, velocity_unit,
                     configuration_.configRunSdoVerifyTimeout);

  // ############################################################## //
  // Homing parameters
  // ############################################################## //
  homingMethod = static_cast<int8_t>(configuration_.homingMethod);
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_METHOD, 0x00, false, homingMethod, 
                      configuration_.configRunSdoVerifyTimeout);

  //Used for homing methods «−1», «−2», «−3», and «−4». A mechanical border will 
  //be detected when the measured motor current rises above the specified threshold [mA].
  currentThreshold = static_cast<uint16_t>(configuration_.currentThreshold);
  configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_THRESHOLD, 0x00, false,currentThreshold, configuration_.configRunSdoVerifyTimeout);
  //homing speeds Speed for switch search
  homingSpeeds = static_cast<uint32_t>(configuration_.homingSpeeds);
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_SPEEDS, 0x01, false,homingSpeeds, configuration_.configRunSdoVerifyTimeout);
  //Specifies the acceleration during Homing
  homingAccel = static_cast<uint32_t>(configuration_.homingAccel);
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_ACCEL, 0x00, false,homingAccel, configuration_.configRunSdoVerifyTimeout);
  // it is useful to move away from a detected position (for example mechanical limit stop or limit switch)
  homeOffsetMoveDistance = static_cast<uint32_t>(configuration_.homeOffsetMoveDistance);
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOME_OFFSET_MOVE_DISTANCE, 0x00, false,homeOffsetMoveDistance, configuration_.configRunSdoVerifyTimeout);
  // Defines the position that will be set as zero position of the absolute position counte
  homePosition = static_cast<uint32_t>(configuration_.homePosition);
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOME_POSITON, 0x00, false,homePosition, configuration_.configRunSdoVerifyTimeout);

  if (!configuration_.useControllerParams) {

    MELO_WARN_STREAM("[maxon_epos_ethercat_sdk] Writing config file specs to controller.");

    // ############################################################## //
    // Speeds
    // ############################################################## //

    maxMotorSpeed = static_cast<uint32_t>(configuration_.workVoltage * configuration_.speedConstant);
    configSuccess &=  sdoVerifyWrite(OD_INDEX_MAX_MOTOR_SPEED, 0x00, false, maxMotorSpeed,
                                     configuration_.configRunSdoVerifyTimeout);

    maxGearSpeed = static_cast<uint32_t>(maxMotorSpeed / configuration_.gearRatio);
    configSuccess &= sdoVerifyWrite(OD_INDEX_GEAR_DATA, 0x03, false, maxGearSpeed,
                                    configuration_.configRunSdoVerifyTimeout);

    maxProfileVelocity = static_cast<uint32_t>(configuration_.maxProfileVelocity * RADTOMICROREV);
    configSuccess &= sdoVerifyWrite(OD_INDEX_MAX_PROFILE_VELOCITY, 0x00, false, maxProfileVelocity,
                                    configuration_.configRunSdoVerifyTimeout);
    
    // ############################################################## //
    // Accelerations
    // ############################################################## //    

    configSuccess &= sdoVerifyWrite(OD_INDEX_QUICKSTOP_DECELERATION, 0x00, false,
                                    configuration_.quickStopDecel,
                                    configuration_.configRunSdoVerifyTimeout);

    configSuccess &= sdoVerifyWrite(OD_INDEX_PROFILE_DECELERATION, 0x00, false,
                                    configuration_.profileDecel,
                                    configuration_.configRunSdoVerifyTimeout);

    // ############################################################## //
    // Position Limits
    // ############################################################## //

    configSuccess &= sdoVerifyWrite(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x01, false,
                                    configuration_.minPosition);

    configSuccess &= sdoVerifyWrite(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x02, false,
                                    configuration_.maxPosition);

    // ############################################################## //
    // Currents and Torque Constant
    // ############################################################## //

    nominalCurrent = static_cast<uint32_t>(round(1000.0 * configuration_.nominalCurrentA));
    configSuccess &=
        sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x01, false, nominalCurrent,
                      configuration_.configRunSdoVerifyTimeout);

    maxCurrent =
        static_cast<uint32_t>(round(1000.0 * configuration_.maxCurrentA));
    configSuccess &= sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x02, false, maxCurrent,
                                    configuration_.configRunSdoVerifyTimeout);

    torqueConstant =
        static_cast<uint32_t>(1000000.0 * configuration_.torqueConstantNmA);
    configSuccess &=
        sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x05, false, torqueConstant,
                      configuration_.configRunSdoVerifyTimeout);

    // ############################################################## //
    // PID-Gains of Controller loops
    // ############################################################## //

    currentPGain = static_cast<uint32_t>(1000000 * configuration_.currentPGainSI);
    configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x01, false,
                                    static_cast<uint32_t>(currentPGain),
                                    configuration_.configRunSdoVerifyTimeout);

    currentIGain = static_cast<uint32_t>(1000 * configuration_.currentIGainSI);
    configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x02, false,
                                    static_cast<uint32_t>(currentIGain),
                                    configuration_.configRunSdoVerifyTimeout);

    positionPGain =
        static_cast<uint32_t>(1000000 * configuration_.positionPGainSI);
    configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x01, false,
                                    static_cast<uint32_t>(positionPGain),
                                    configuration_.configRunSdoVerifyTimeout);

    positionIGain =
        static_cast<uint32_t>(1000000 * configuration_.positionIGainSI);
    configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x02, false,
                                    static_cast<uint32_t>(positionIGain),
                                    configuration_.configRunSdoVerifyTimeout);

    positionDGain =
        static_cast<uint32_t>(1000000 * configuration_.positionDGainSI);
    configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x03, false,
                                    static_cast<uint32_t>(positionDGain),
                                    configuration_.configRunSdoVerifyTimeout);


    configSuccess &= sdoVerifyWrite(OD_INDEX_FOLLOW_ERROR_WINDOW, 0x00, false,
                                    configuration_.followErrorWindow,
                                    configuration_.configRunSdoVerifyTimeout);

    velocityPGain =
        static_cast<uint32_t>(1000000 * configuration_.velocityPGainSI);
    configSuccess &= sdoVerifyWrite(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x01, false,
                                    static_cast<uint32_t>(velocityPGain),
                                    configuration_.configRunSdoVerifyTimeout);

    velocityIGain =
        static_cast<uint32_t>(1000000 * configuration_.velocityIGainSI);
    configSuccess &= sdoVerifyWrite(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x02, false,
                                    static_cast<uint32_t>(velocityIGain),
                                    configuration_.configRunSdoVerifyTimeout);
  
  } else {
    // Read Parameters to config from Controllers
    MELO_WARN_STREAM("[maxon_epos_ethercat_sdk] Ignoring motor specs from config file. Reading from controller instead");

    uint32_t maxProfileVelocity;
    configSuccess &= sendSdoRead(OD_INDEX_MAX_PROFILE_VELOCITY, 0x00, false,
                                    maxProfileVelocity);

    configuration_.maxProfileVelocity = maxProfileVelocity * 2 * M_PI / 60 / 1e6;


    uint8_t polePairs;
    configSuccess &= sendSdoRead(OD_INDEX_MOTOR_DATA, 0x03, false,
                                    polePairs);
    configuration_.polePairs = static_cast<double>(polePairs);


    uint32_t numerator, denominator;
    configSuccess &= sendSdoRead(OD_INDEX_GEAR_DATA, 0x01, false, numerator);
    configSuccess &= sendSdoRead(OD_INDEX_GEAR_DATA, 0x02, false, denominator);
    configuration_.gearRatio = static_cast<double>(numerator) / static_cast<double>(denominator);

    int32_t min,max;
    configSuccess &= sendSdoRead(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x01, false,
                                    min);

    configSuccess &= sendSdoRead(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x02, false,
                                    max);
    configuration_.minPosition = min;
    configuration_.maxPosition = max;

    uint32_t nominalCurrent;
    configSuccess &= sendSdoRead(OD_INDEX_MOTOR_DATA, 0x01, false, nominalCurrent);
    configuration_.nominalCurrentA = static_cast<double>(nominalCurrent) / 1000;

    uint32_t maxCurrent;
    configSuccess &= sendSdoRead(OD_INDEX_MOTOR_DATA, 0x02, false, maxCurrent);
    configuration_.maxCurrentA = static_cast<double> (maxCurrent) / 1000;    

    uint32_t torqueConstant;
    configSuccess &= sendSdoRead(OD_INDEX_MOTOR_DATA, 0x05, false, torqueConstant); 
    configuration_.torqueConstantNmA = static_cast<double> (torqueConstant) / 1e6;

    configSuccess &= sendSdoRead(OD_INDEX_QUICKSTOP_DECELERATION, 0x00, false,
                                    configuration_.quickStopDecel);

    configSuccess &= sendSdoRead(OD_INDEX_PROFILE_DECELERATION, 0x00, false,
                                    configuration_.profileDecel);

    configSuccess &= sendSdoRead(OD_INDEX_FOLLOW_ERROR_WINDOW, 0x00, false,
                                    configuration_.followErrorWindow);


    // Retrieve sensor configuration from device and set scaling (position raw <-> actual) correspondingly
    
    uint32_t encoderResolution = 0;   

    configSuccess &= sendSdoRead(OD_INDEX_SENSOR_CONFIG, 0x05, false,
                                    encoderResolution);
    configuration_.positionEncoderResolution = encoderResolution;

    uint32_t sensorConfig = 0, mountingPosition = 0, gearMounted = 0;
    configSuccess &= sendSdoRead(OD_INDEX_SENSOR_CONFIG, 0x02, false,
                                sensorConfig);

    uint32_t mainSensor = (sensorConfig >> 16) & 0xF;

    gearMounted = (sensorConfig >> 12) & 0x1;


    switch (mainSensor) {
      case 1:
        mountingPosition = (sensorConfig >> 24) & 0x1;
        break;
      case 2:
        mountingPosition = (sensorConfig >> 26) & 0x1;
        break;
      case 3:
        mountingPosition = (sensorConfig >> 28) & 0x1;
        break;
      default:
        break;
    }

    MELO_WARN_STREAM("Gear mounted: " + std::to_string(gearMounted) + ", Main Sensor: " + std::to_string(mainSensor) + ", Sensor Position: " + std::to_string(mountingPosition) + "\n");

    if (mountingPosition == 0 && gearMounted) { // Sensor on motor axis with gear mounted.
      configuration_.sensorPositionCorrection = configuration_.gearRatio;
    } else {
      configuration_.sensorPositionCorrection = 1;
    }


    configuration_.sanityCheck();
  }



  if (configSuccess) {
    MELO_INFO("Setting configuration parameters succeeded.");
  } else {
    MELO_ERROR("Setting configuration parameters failed.");
  }

  return configSuccess;
}
}  // namespace maxon
