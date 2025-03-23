#include "dbc/dbc_parser.hpp"
#include "v2c/v2c_transcoder.hpp"
#include <gtest/gtest.h>
#include <string>
#include <utility>

// https://www.csselectronics.com/pages/can-dbc-file-database-intro
// https://github.com/commaai/opendbc/tree/master/opendbc/dbc
// https://github.com/joshwardell/model3dbc
// https://github.com/famouspotatoes1/can-dbc/tree/main/src/__tests__/testFiles
// https://www.csselectronics.com/pages/dbc-editor-can-bus-database

static const std::string dbc_0 = R"(
BO_ 2364540158 EEC1: 8 Vector__XXX
	SG_ EngineSpeed : 24|16@1+ (0.125,0) [0|8031.875] "rpm" Vector__XXX
)";

static const std::string dbc_1 =  R"(VERSION "0.1"

NS_ :

BS_:

BU_: Receiver ChassisBus VehicleBus V2C

BO_ 2 GPSLatLong: 8 ChassisBus
 SG_ GPSAccuracy : 57|7@1+ (0.2,0) [0|25] "m" Receiver
 SG_ GPSLongitude : 28|28@1- (1e-06,0) [-134.22|134.22] "Deg" Receiver
 SG_ GPSLatitude : 0|28@1- (1e-06,0) [-134.22|134.22] "Deg" Receiver

BO_ 3 GPSAltitude: 2 VehicleBus
 SG_ GPSAltitude : 0|14@1- (1,0) [-1000|7000] "M" Receiver

BO_ 4 SOC: 8 VehicleBus
 SG_ SOCavg : 30|10@1+ (0.1,0) [0|100] "%" Receiver

BO_ 5 GPSSpeed: 8 VehicleBus
 SG_ GPSSpeed : 12|12@1+ (0.2,-10) [-50|300] "kph" Receiver

BO_ 6 BatteryCurrent: 8 VehicleBus
 SG_ RawBattCurrent : 32|16@1- (0.05,-800) [-1000|1000] "A" Receiver
 SG_ SmoothBattCurrent : 16|16@1- (-0.5,0) [-1000|1000] "A" Receiver
 SG_ BattVoltage : 0|16@1+ (0.1,0) [0|1000] "V" Receiver

BO_ 7 PowerState: 2 VehicleBus
 SG_ PowerState : 14|2@1+ (1,0) [0|3] "" Receiver

EV_ V2CTxTime: 0 [0|60000] "ms" 2000 1 DUMMY_NODE_VECTOR1 V2C;
EV_ GPSGroupTxFreq: 0 [0|60000] "ms" 600 11 DUMMY_NODE_VECTOR1 V2C;
EV_ EnergyGroupTxFreq: 0 [0|60000] "ms" 500 12 DUMMY_NODE_VECTOR1 V2C;

BA_DEF_ BO_ "TxGroupFreq" STRING ;
BA_DEF_ SG_ "AggType" STRING ;

BA_ "AggType" SG_ 2 GPSAccuracy "LAST";
BA_ "AggType" SG_ 2 GPSLongitude "LAST";
BA_ "AggType" SG_ 2 GPSLatitude "LAST";
BA_ "AggType" SG_ 3 GPSAltitude "LAST";
BA_ "AggType" SG_ 5 GPSSpeed "LAST";
BA_ "AggType" SG_ 7 PowerState "LAST";
BA_ "AggType" SG_ 4 SOCavg "LAST";
BA_ "AggType" SG_ 6 RawBattCurrent "AVG";
BA_ "AggType" SG_ 6 SmoothBattCurrent "AVG";
BA_ "AggType" SG_ 6 BattVoltage "AVG";

BA_ "TxGroupFreq" BO_ 2 "GPSGroupTxFreq";
BA_ "TxGroupFreq" BO_ 3 "GPSGroupTxFreq";
BA_ "TxGroupFreq" BO_ 5 "GPSGroupTxFreq";
BA_ "TxGroupFreq" BO_ 7 "GPSGroupTxFreq";

BA_ "TxGroupFreq" BO_ 4 "EnergyGroupTxFreq";
BA_ "TxGroupFreq" BO_ 6 "EnergyGroupTxFreq";
)";

// https://gist.github.com/aphrx
static const std::string dbc_2 = R"(VERSION "0.1"

NS_ :

BS_:

BU_: VSA EPS EON V2C

BO_ 399 STEER_STATUS: 7 EPS
 SG_ STEER_TORQUE_SENSOR : 7|16@0- (-1,0) [-31000|31000] "tbd" EON
 SG_ STEER_ANGLE_RATE : 23|16@0- (-0.1,0) [-31000|31000] "deg/s" EON
 SG_ STEER_STATUS : 39|4@0+ (1,0) [0|15] "" EON
 SG_ STEER_CONTROL_ACTIVE : 35|1@0+ (1,0) [0|1] "" EON
 SG_ STEER_CONFIG_INDEX : 43|4@0+ (1,0) [0|15] "" EON
 SG_ COUNTER : 53|2@0+ (1,0) [0|3] "" EON
 SG_ CHECKSUM : 51|4@0+ (1,0) [0|15] "" EON

BO_ 420 VSA_STATUS: 8 VSA
 SG_ ESP_DISABLED : 28|1@0+ (1,0) [0|1] "" EON
 SG_ USER_BRAKE : 7|16@0+ (0.015625,-1.609375) [0|1000] "" EON
 SG_ BRAKE_HOLD_ACTIVE : 46|1@0+ (1,0) [0|1] "" EON
 SG_ BRAKE_HOLD_ENABLED : 45|1@0+ (1,0) [0|1] "" EON
 SG_ COUNTER : 61|2@0+ (1,0) [0|3] "" EON
 SG_ CHECKSUM : 59|4@0+ (1,0) [0|15] "" EON

BO_ 464 WHEEL_SPEEDS: 8 VSA
 SG_ WHEEL_SPEED_FL : 7|15@0+ (0.01,0) [0|250] "kph" EON
 SG_ WHEEL_SPEED_FR : 8|15@0+ (0.01,0) [0|250] "kph" EON
 SG_ WHEEL_SPEED_RL : 25|15@0+ (0.01,0) [0|250] "kph" EON
 SG_ WHEEL_SPEED_RR : 42|15@0+ (0.01,0) [0|250] "kph" EON
 SG_ CHECKSUM : 59|4@0+ (1,0) [0|3] "" EON

EV_ V2CTxTime: 0 [0|60000] "ms" 2000 1 DUMMY_NODE_VECTOR1 V2C;
EV_ WheelGroupTxFreq: 0 [0|60000] "ms" 600 11 DUMMY_NODE_VECTOR1 V2C;
EV_ SteerGroupTxFreq: 0 [0|60000] "ms" 500 12 DUMMY_NODE_VECTOR1 V2C;

BA_DEF_ BO_ "TxGroupFreq" STRING ;
BA_DEF_ SG_ "AggType" STRING ;

BA_ "AggType" SG_ 464 WHEEL_SPEED_FL "LAST";
BA_ "AggType" SG_ 464 WHEEL_SPEED_FR "LAST";
BA_ "AggType" SG_ 464 WHEEL_SPEED_RL "LAST";
BA_ "AggType" SG_ 464 WHEEL_SPEED_RR "LAST";

BA_ "AggType" SG_ 399 STEER_TORQUE_SENSOR "LAST";
BA_ "AggType" SG_ 399 STEER_ANGLE_RATE "LAST";
BA_ "AggType" SG_ 399 STEER_STATUS "LAST";
BA_ "AggType" SG_ 399 STEER_CONTROL_ACTIVE "LAST";
BA_ "AggType" SG_ 399 STEER_CONFIG_INDEX "LAST";

BA_ "TxGroupFreq" BO_ 464 "WheelGroupTxFreq";
BA_ "TxGroupFreq" BO_ 399 "SteerGroupTxFreq";

)";

static const std::string dbc_3 = R"(
VERSION ""


NS_ : 
	NS_DESC_
	CM_
	BA_DEF_
	BA_
	VAL_
	CAT_DEF_
	CAT_
	FILTER
	BA_DEF_DEF_
	EV_DATA_
	ENVVAR_DATA_
	SGTYPE_
	SGTYPE_VAL_
	BA_DEF_SGTYPE_
	BA_SGTYPE_
	SIG_TYPE_REF_
	VAL_TABLE_
	SIG_GROUP_
	SIG_VALTYPE_
	SIGTYPE_VALTYPE_
	BO_TX_BU_
	BA_DEF_REL_
	BA_REL_
	BA_DEF_DEF_REL_
	BU_SG_REL_
	BU_EV_REL_
	BU_BO_REL_
	SG_MUL_VAL_

BS_:

BU_:


BO_ 2147484160 TestMessageExtended: 8 Vector__XXX
 SG_ TestSignal4 : 24|8@1- (1,0) [0|0] "" Vector__XXX
 SG_ TestSignal3 : 16|8@1- (1,0) [0|0] "" Vector__XXX
 SG_ TestSignal2 : 8|8@1- (1,0) [0|0] "" Vector__XXX
 SG_ TestSignal1 : 0|8@1- (1,0) [0|0] "" Vector__XXX

BO_ 256 TestMessageStandard: 8 Vector__XXX
 SG_ TestSignal8 : 24|8@1- (1,0) [0|0] "" Vector__XXX
 SG_ TestSignal7 : 16|8@1- (1,0) [0|0] "" Vector__XXX
 SG_ TestSignal6 : 8|8@1- (1,0) [0|0] "" Vector__XXX
 SG_ TestSignal5 : 0|8@1- (1,0) [0|0] "" Vector__XXX



BA_DEF_ SG_  "GenSigSendType" ENUM  "Cyclic","OnWrite","OnWriteWithRepetition","OnChange","OnChangeWithRepetition","IfActive","IfActiveWithRepetition","NoSigSendType";
BA_DEF_ SG_  "GenSigInactiveValue" INT 0 0;
BA_DEF_ BO_  "GenMsgCycleTime" INT 0 0;
BA_DEF_ BO_  "GenMsgSendType" ENUM  "Cyclic","not_used","not_used","not_used","not_used","Cyclic","not_used","IfActive","NoMsgSendType";
BA_DEF_ BU_  "NmStationAddress" HEX 0 0;
BA_DEF_  "DBName" STRING ;
BA_DEF_  "BusType" STRING ;
BA_DEF_ BU_  "NodeLayerModules" STRING ;
BA_DEF_ BU_  "ECU" STRING ;
BA_DEF_ BU_  "CANoeJitterMax" INT 0 0;
BA_DEF_ BU_  "CANoeJitterMin" INT 0 0;
BA_DEF_ BU_  "CANoeDrift" INT 0 0;
BA_DEF_ BU_  "CANoeStartDelay" INT 0 0;
BA_DEF_DEF_  "GenSigSendType" "Cyclic";
BA_DEF_DEF_  "GenSigInactiveValue" 0;
BA_DEF_DEF_  "GenMsgCycleTime" 0;
BA_DEF_DEF_  "GenMsgSendType" "NoMsgSendType";
BA_DEF_DEF_  "NmStationAddress" 0;
BA_DEF_DEF_  "DBName" "";
BA_DEF_DEF_  "BusType" "CAN";
BA_DEF_DEF_  "NodeLayerModules" "";
BA_DEF_DEF_  "ECU" "";
BA_DEF_DEF_  "CANoeJitterMax" 0;
BA_DEF_DEF_  "CANoeJitterMin" 0;
BA_DEF_DEF_  "CANoeDrift" 0;
BA_DEF_DEF_  "CANoeStartDelay" 0;
BA_ "DBName" "SimpleDBC";
)";

static const std::string dbc_4 = R"(
VERSION ""


NS_ :
	NS_DESC_
	CM_
	BA_DEF_
	BA_
	VAL_
	CAT_DEF_
	CAT_
	FILTER
	BA_DEF_DEF_
	EV_DATA_
	ENVVAR_DATA_
	SGTYPE_
	SGTYPE_VAL_
	BA_DEF_SGTYPE_
	BA_SGTYPE_
	SIG_TYPE_REF_
	VAL_TABLE_
	SIG_GROUP_
	SIG_VALTYPE_
	SIGTYPE_VALTYPE_
	BO_TX_BU_
	BA_DEF_REL_
	BA_REL_
	BA_DEF_DEF_REL_
	BU_SG_REL_
	BU_EV_REL_
	BU_BO_REL_
	SG_MUL_VAL_

BS_:

BU_: NEO MCU GTW EPAS DI ESP SBW STW APP DAS XXX EPB APS PARK IC

VAL_TABLE_ StW_AnglHP_Spd 16383 "SNA" ;
VAL_TABLE_ DI_aebFaultReason 15 "DI_AEB_FAULT_DAS_REQ_DI_UNAVAIL" 14 "DI_AEB_FAULT_ACCEL_REQ_INVALID" 13 "DI_AEB_FAULT_MIN_TIME_BTWN_EVENTS" 12 "DI_AEB_FAULT_ESP_MIA" 11 "DI_AEB_FAULT_ESP_FAULT" 10 "DI_AEB_FAULT_EPB_NOT_PARKED" 9 "DI_AEB_FAULT_ACCEL_OUT_OF_BOUNDS" 8 "DI_AEB_FAULT_PM_REQUEST" 7 "DI_AEB_FAULT_VEL_EST_ABNORMAL" 6 "DI_AEB_FAULT_DAS_SNA" 5 "DI_AEB_FAULT_DAS_CONTROL_MIA" 4 "DI_AEB_FAULT_SPEED_DELTA" 3 "DI_AEB_FAULT_EBR_FAULT" 2 "DI_AEB_FAULT_PM_MIA" 1 "DI_AEB_FAULT_EPB_MIA" 0 "DI_AEB_FAULT_NONE" ;
VAL_TABLE_ DI_aebLockState 3 "AEB_LOCK_STATE_SNA" 2 "AEB_LOCK_STATE_UNUSED" 1 "AEB_LOCK_STATE_UNLOCKED" 0 "AEB_LOCK_STATE_LOCKED" ;
VAL_TABLE_ DI_aebSmState 7 "DI_AEB_STATE_FAULT" 6 "DI_AEB_STATE_EXIT" 5 "DI_AEB_STATE_STANDSTILL" 4 "DI_AEB_STATE_STOPPING" 3 "DI_AEB_STATE_ENABLE" 2 "DI_AEB_STATE_ENABLE_INIT" 1 "DI_AEB_STATE_STANDBY" 0 "DI_AEB_STATE_UNAVAILABLE" ;
VAL_TABLE_ DI_aebState 7 "AEB_CAN_STATE_SNA" 4 "AEB_CAN_STATE_FAULT" 3 "AEB_CAN_STATE_STANDSTILL" 2 "AEB_CAN_STATE_ENABLED" 1 "AEB_CAN_STATE_STANDBY" 0 "AEB_CAN_STATE_UNAVAILABLE" ;
VAL_TABLE_ DI_epbInterfaceReady 1 "EPB_INTERFACE_READY" 0 "EPB_INTERFACE_NOT_READY" ;
VAL_TABLE_ DI_gear 7 "DI_GEAR_SNA" 4 "DI_GEAR_D" 3 "DI_GEAR_N" 2 "DI_GEAR_R" 1 "DI_GEAR_P" 0 "DI_GEAR_INVALID" ;
VAL_TABLE_ DI_gpoReason 8 "DI_GPO_NUMREASONS" 7 "DI_GPO_CAPACITOR_OVERTEMP" 6 "DI_GPO_NOT_ENOUGH_12V" 5 "DI_GPO_NO_BATTERY_POWER" 4 "DI_GPO_AMBIENT_OVERTEMP" 3 "DI_GPO_FLUID_DELTAT" 2 "DI_GPO_STATOR_OVERTEMP" 1 "DI_GPO_HEATSINK_OVERTEMP" 0 "DI_GPO_OUTLET_OVERTEMP" ;
VAL_TABLE_ DI_immobilizerCondition 1 "DI_IMM_CONDITION_LEARNED" 0 "DI_IMM_CONDITION_VIRGIN_SNA" ;
VAL_TABLE_ DI_immobilizerState 7 "DI_IMM_STATE_FAULT" 6 "DI_IMM_STATE_FAULTRETRY" 5 "DI_IMM_STATE_RESET" 4 "DI_IMM_STATE_LEARN" 3 "DI_IMM_STATE_DISARMED" 2 "DI_IMM_STATE_AUTHENTICATING" 1 "DI_IMM_STATE_REQUEST" 0 "DI_IMM_STATE_INIT_SNA" ;
VAL_TABLE_ DI_limpReason 24 "DI_LIMP_NUMREASONS" 23 "DI_LIMP_CAPACITOR_OVERTEMP" 22 "DI_LIMP_GTW_MIA" 21 "DI_LIMP_TRQCMD_VALIDITY_UNKNOWN" 20 "DI_LIMP_DI_MIA" 19 "DI_LIMP_CONFIG_MISMATCH" 18 "DI_LIMP_HEATSINK_TEMP" 17 "DI_LIMP_PMREQUEST" 16 "DI_LIMP_PMHEARTBEAT" 15 "DI_LIMP_TRQ_CROSS_CHECK" 14 "DI_LIMP_EXTERNAL_COMMAND" 13 "DI_LIMP_WRONG_CS_CALIBRATION" 12 "DI_LIMP_STATOR_TEMP" 11 "DI_LIMP_DELTAT_TOO_NEGATIVE" 10 "DI_LIMP_DELTAT_TOO_POSITIVE" 9 "DI_LIMP_AMBIENT_TEMP" 8 "DI_LIMP_OUTLET_TEMP" 7 "DI_LIMP_LOW_FLOW" 6 "DI_LIMP_BMS_MIA" 5 "DI_LIMP_12V_SUPPLY_UNDERVOLTAGE" 4 "DI_LIMP_NO_FLUID" 3 "DI_LIMP_NO_FUNC_HEATSINK_SENSOR" 2 "DI_LIMP_NO_FUNC_STATORT_SENSOR" 1 "DI_LIMP_BUSV_SENSOR_IRRATIONAL" 0 "DI_LIMP_PHASE_IMBALANCE" ;
VAL_TABLE_ DI_mode 2 "DI_MODE_DYNO" 1 "DI_MODE_DRIVE" 0 "DI_MODE_UNDEF" ;
VAL_TABLE_ DI_motorType 14 "DI_MOTOR_F2AE" 13 "DI_MOTOR_F2AD" 12 "DI_MOTOR_F2AC" 11 "DI_MOTOR_F2AB" 10 "DI_MOTOR_F1AC" 9 "DI_MOTOR_SSR1A" 8 "DI_MOTOR_F1A" 7 "DI_MOTOR_M7M6" 6 "DI_MOTOR_M8A" 5 "DI_MOTOR_M7M5" 4 "DI_MOTOR_M7M4" 3 "DI_MOTOR_M7M3" 2 "DI_MOTOR_ROADSTER_SPORT" 1 "DI_MOTOR_ROADSTER_BASE" 0 "DI_MOTOR_SNA" ;
VAL_TABLE_ DI_speedUnits 1 "DI_SPEED_KPH" 0 "DI_SPEED_MPH" ;
VAL_TABLE_ DI_state 4 "DI_STATE_ENABLE" 3 "DI_STATE_FAULT" 2 "DI_STATE_CLEAR_FAULT" 1 "DI_STATE_STANDBY" 0 "DI_STATE_PREAUTH" ;
VAL_TABLE_ DI_velocityEstimatorState 4 "VE_STATE_BACKUP_MOTOR" 3 "VE_STATE_BACKUP_WHEELS_B" 2 "VE_STATE_BACKUP_WHEELS_A" 1 "VE_STATE_WHEELS_NORMAL" 0 "VE_STATE_NOT_INITIALIZED" ;


BO_ 1160 DAS_steeringControl: 4 NEO
 SG_ DAS_steeringControlType : 23|2@0+ (1,0) [0|0] "" EPAS
 SG_ DAS_steeringControlChecksum : 31|8@0+ (1,0) [0|0] "" EPAS
 SG_ DAS_steeringControlCounter : 19|4@0+ (1,0) [0|0] "" EPAS
 SG_ DAS_steeringAngleRequest : 6|15@0+ (0.1,-1638.35) [-1638.35|1638.35] "deg" EPAS
 SG_ DAS_steeringHapticRequest : 7|1@0+ (1,0) [0|0] "" EPAS

BO_ 697 DAS_control: 8 NEO
 SG_ DAS_setSpeed : 0|12@1+ (0.1,0) [0|409.4] "kph" NEO
 SG_ DAS_accState : 12|4@1+ (1,0) [0|0] "" NEO
 SG_ DAS_aebEvent : 16|2@1+ (1,0) [0|3] "" NEO
 SG_ DAS_jerkMin : 18|9@1+ (0.03,-15.232) [-15.232|0.098] "m/s^3" NEO
 SG_ DAS_jerkMax : 27|8@1+ (0.059,0) [0|15.045] "m/s^3" NEO
 SG_ DAS_accelMin : 35|9@1+ (0.04,-15) [-15|5.44] "m/s^2" NEO
 SG_ DAS_accelMax : 44|9@1+ (0.04,-15) [-15|5.44] "m/s^2" NEO
 SG_ DAS_controlCounter : 53|3@1+ (1,0) [0|0] "" NEO
 SG_ DAS_controlChecksum : 56|8@1+ (1,0) [0|0] "" NEO

BO_ 521 DAS_longControl: 8 NEO
 SG_ DAS_locMode : 0|2@1+ (1,0) [0|0] "" NEO
 SG_ DAS_locState : 2|3@1+ (1,0) [0|0] "" NEO
 SG_ DAS_locRequest : 5|3@1+ (1,0) [0|0] "" NEO
 SG_ DAS_locJerkMin : 8|8@1+ (0.034,-8.67) [-8.67|0] "m/s^3" NEO
 SG_ DAS_locJerkMax : 16|8@1+ (0.034,0) [0|8.67] "m/s^3" NEO
 SG_ DAS_locSpeed : 24|11@1+ (0.1,0) [0|204.7] "kph" NEO
 SG_ DAS_locAccelMin : 35|9@1+ (0.04,-15) [-15|5.44] "m/s^2" NEO
 SG_ DAS_locAccelMax : 44|9@1+ (0.04,-15) [-15|5.44] "m/s^2" NEO
 SG_ DAS_longControlCounter : 53|3@1+ (1,0) [0|0] "" NEO
 SG_ DAS_longControlChecksum : 56|8@1+ (1,0) [0|0] "" NEO

BO_ 569 DAS_lanes: 8 NEO
 SG_ DAS_leftLaneExists : 0|1@1+ (1,0) [0|0] "" NEO
 SG_ DAS_rightLaneExists : 1|1@1+ (1,0) [0|0] "" NEO
 SG_ DAS_virtualLaneWidth : 4|4@1+ (0.3125,2) [2|7] "m" NEO
 SG_ DAS_virtualLaneViewRange : 8|8@1+ (1,0) [0|160] "m" NEO
 SG_ DAS_virtualLaneC0 : 16|8@1+ (0.035,-3.5) [-3.5|3.5] "m" NEO
 SG_ DAS_virtualLaneC1 : 24|8@1+ (0.0016,-0.2) [-0.2|0.2] "rad" NEO
 SG_ DAS_virtualLaneC2 : 32|8@1+ (2E-05,-0.0025) [-0.0025|0.0025] "m-1" NEO
 SG_ DAS_virtualLaneC3 : 40|8@1+ (2.4E-07,-3E-05) [-3E-05|3E-05] "m-2" NEO
 SG_ DAS_leftLineUsage : 48|2@1+ (1,0) [0|3] "" NEO
 SG_ DAS_rightLineUsage : 50|2@1+ (1,0) [0|3] "" NEO
 SG_ DAS_leftFork : 52|2@1+ (1,0) [0|3] "" NEO
 SG_ DAS_rightFork : 54|2@1+ (1,0) [0|3] "" NEO
 SG_ DAS_lanesCounter : 60|4@1+ (1,0) [0|0] "" NEO

BO_ 257 GTW_epasControl: 3 NEO
 SG_ GTW_epasControlChecksum : 23|8@0+ (1,0) [0|255] ""  NEO
 SG_ GTW_epasControlCounter : 11|4@0+ (1,0) [0|15] ""  NEO
 SG_ GTW_epasControlType : 15|2@0+ (1,0) [-1|4] ""  NEO
 SG_ GTW_epasEmergencyOn : 7|1@0+ (1,0) [-1|2] ""  NEO
 SG_ GTW_epasLDWEnabled : 12|1@0+ (1,0) [-1|2] ""  NEO
 SG_ GTW_epasPowerMode : 6|4@0+ (1,0) [4|14] ""  NEO
 SG_ GTW_epasTuneRequest : 2|3@0+ (1,0) [-1|8] ""  NEO

BO_ 880 EPAS_sysStatus: 8 EPAS
 SG_ EPAS_currentTuneMode : 7|4@0+ (1,0) [8|15] ""  NEO
 SG_ EPAS_eacErrorCode : 23|4@0+ (1,0) [-1|16] ""  NEO
 SG_ EPAS_eacStatus : 55|3@0+ (1,0) [5|7] ""  NEO
 SG_ EPAS_handsOnLevel : 39|2@0+ (1,0) [-1|4] ""  NEO
 SG_ EPAS_internalSAS : 37|14@0+ (0.1,-819.200012) [0|0] "deg"  NEO
 SG_ EPAS_steeringFault : 2|1@0+ (1,0) [-1|2] ""  NEO
 SG_ EPAS_steeringRackForce : 1|10@0+ (50,-25575) [0|0] "N" NEO
 SG_ EPAS_steeringReduced : 3|1@0+ (1,0) [-1|2] ""  NEO
 SG_ EPAS_sysStatusChecksum : 63|8@0+ (1,0) [0|255] ""  NEO
 SG_ EPAS_sysStatusCounter : 51|4@0+ (1,0) [0|15] ""  NEO
 SG_ EPAS_torsionBarTorque : 19|12@0+ (0.01,-20.5) [0|0] "Nm"  NEO

BO_ 3 STW_ANGL_STAT: 8 STW
 SG_ StW_Angl : 5|14@0+ (0.5,-2048) [0|0] "deg"  NEO
 SG_ StW_AnglSpd : 21|14@0+ (0.5,-2048) [0|0] "/s"  NEO
 SG_ StW_AnglSens_Stat : 33|2@0+ (1,0) [-1|4] ""  NEO
 SG_ StW_AnglSens_Id : 35|2@0+ (1,0) [3|3] ""  NEO
 SG_ MC_STW_ANGL_STAT : 55|4@0+ (1,0) [0|15] ""  NEO
 SG_ CRC_STW_ANGL_STAT : 63|8@0+ (1,0) [0|255] ""  NEO

BO_ 14 STW_ANGLHP_STAT: 8 STW
 SG_ StW_AnglHP : 5|14@0+ (0.1,-819.2) [-819.2|819] "deg"  NEO
 SG_ StW_AnglHP_Spd : 21|14@0+ (0.5,-4096) [-4096|4095.5] "deg/s"  NEO
 SG_ StW_AnglHP_Sens_Stat : 33|2@0+ (1,0) [0|0] ""  NEO
 SG_ StW_AnglHP_Sens_Id : 35|2@0+ (1,0) [0|0] ""  NEO
 SG_ MC_STW_ANGLHP_STAT : 55|4@0+ (1,0) [0|15] ""  NEO
 SG_ CRC_STW_ANGLHP_STAT : 63|8@0+ (1,0) [0|0] ""  NEO

BO_ 264 DI_torque1: 8 DI
 SG_ DI_torqueDriver : 0|13@1- (0.25,0) [-750|750] "Nm"  NEO
 SG_ DI_torque1Counter : 13|3@1+ (1,0) [0|0] ""  NEO
 SG_ DI_torqueMotor : 16|13@1- (0.25,0) [-750|750] "Nm"  NEO
 SG_ DI_soptState : 29|3@1+ (1,0) [0|0] ""  NEO
 SG_ DI_motorRPM : 32|16@1- (1,0) [-17000|17000] "RPM"  NEO
 SG_ DI_pedalPos : 48|8@1+ (0.4,0) [0|100] "%"  NEO
 SG_ DI_torque1Checksum : 56|8@1+ (1,0) [0|0] ""  NEO

BO_ 280 DI_torque2: 6 DI
 SG_ DI_torqueEstimate : 0|12@1- (0.5,0) [-750|750] "Nm" NEO
 SG_ DI_gear : 12|3@1+ (1,0) [0|0] "" NEO
 SG_ DI_brakePedal : 15|1@1+ (1,0) [0|0] "" NEO
 SG_ DI_vehicleSpeed : 16|12@1+ (0.05,-25) [-25|179.75] "MPH" NEO
 SG_ DI_gearRequest : 28|3@1+ (1,0) [0|0] "" NEO
 SG_ DI_torqueInterfaceFailure : 31|1@1+ (1,0) [0|0] "" NEO
 SG_ DI_torque2Counter : 32|4@1+ (1,0) [0|0] "" NEO
 SG_ DI_brakePedalState : 36|2@1+ (1,0) [0|0] "" NEO
 SG_ DI_epbParkRequest : 38|1@1+ (1,0) [0|0] "" NEO
 SG_ DI_epbInterfaceReady : 39|1@1+ (1,0) [0|0] "" NEO
 SG_ DI_torque2Checksum : 40|8@1+ (1,0) [0|0] "" NEO

BO_ 309 ESP_135h: 5 ESP
 SG_ ESP_135hChecksum : 23|8@0+ (1,0) [0|255] ""  NEO
 SG_ ESP_135hCounter : 11|4@0+ (1,0) [0|15] ""  NEO
 SG_ ESP_absBrakeEvent : 2|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_brakeDiscWipingActive : 4|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_brakeLamp : 3|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_espFaultLamp : 6|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_espLampFlash : 7|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_hillStartAssistActive : 1|2@0+ (1,0) [-1|4] ""  NEO
 SG_ ESP_messagePumpService : 24|1@0+ (1,0) [0|1] ""  NEO
 SG_ ESP_messagePumpFailure : 25|1@0+ (1,0) [0|1] ""  NEO
 SG_ ESP_messageEBDFailure : 26|1@0+ (1,0) [0|1] ""  NEO
 SG_ ESP_absFaultLamp : 27|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_tcDisabledByFault : 28|1@0+ (1,0) [0|1] ""  NEO
 SG_ ESP_messageDynoModeActive : 29|1@0+ (1,0) [0|1] ""  NEO
 SG_ ESP_hydraulicBoostEnabled : 30|1@0+ (1,0) [0|1] ""  NEO
 SG_ ESP_espOffLamp : 31|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_stabilityControlSts : 14|3@0+ (1,0) [6|7] ""  NEO
 SG_ ESP_tcLampFlash : 5|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ESP_tcOffLamp : 15|1@0+ (1,0) [0|1] ""  NEO

BO_ 341 ESP_B: 8 ESP
 SG_ ESP_BChecksum : 39|8@0+ (1,0) [0|255] ""  NEO,EPAS
 SG_ ESP_BCounter : 62|4@0+ (1,0) [1|15] ""  NEO,EPAS
 SG_ ESP_vehicleSpeed : 47|16@0+ (0.00999999978,0) [0|0] "kph"  NEO,EPAS
 SG_ ESP_vehicleSpeedQF : 57|2@0+ (1,0) [1|2] ""  NEO,EPAS
 SG_ ESP_wheelPulseCountFrL : 7|8@0+ (1,0) [0|254] ""  NEO,EPAS
 SG_ ESP_wheelPulseCountFrR : 15|8@0+ (1,0) [0|254] ""  NEO,EPAS
 SG_ ESP_wheelPulseCountReL : 23|8@0+ (1,0) [0|254] ""  NEO,EPAS
 SG_ ESP_wheelPulseCountReR : 31|8@0+ (1,0) [0|254] ""  NEO,EPAS

BO_ 513 SDM1: 5 GTW
 SG_ SDM_bcklPassStatus : 3|2@0+ (1,0) [0|3] "" NEO
 SG_ SDM_bcklDrivStatus : 5|2@0+ (1,0) [0|3] "" NEO

BO_ 532 EPB_epasControl: 3 EPB
 SG_ EPB_epasControlChecksum : 23|8@0+ (1,0) [0|255] ""  NEO,EPAS
 SG_ EPB_epasControlCounter : 11|4@0+ (1,0) [0|15] ""  NEO,EPAS
 SG_ EPB_epasEACAllow : 2|3@0+ (1,0) [4|7] ""  NEO,EPAS

BO_ 792 GTW_carState: 8 GTW
 SG_ YEAR : 0|7@1+ (1,2000) [2000|2127] "Year" NEO
 SG_ CERRD : 7|1@1+ (1,0) [0|1] "" NEO
 SG_ MONTH : 8|4@1+ (1,0) [1|12] "Month" NEO
 SG_ DOOR_STATE_FL : 12|2@1+ (1,0) [0|3] "" NEO
 SG_ DOOR_STATE_FR : 14|2@1+ (1,0) [0|3] "" NEO
 SG_ SECOND : 16|6@1+ (1,0) [0|59] "s" NEO
 SG_ DOOR_STATE_RL : 22|2@1+ (1,0) [0|3] "" NEO
 SG_ Hour : 24|5@1+ (1,0) [0|23] "h" NEO
 SG_ DOOR_STATE_RR : 29|2@1+ (1,0) [0|3] "" NEO
 SG_ DAY : 32|5@1+ (1,0) [0|31] "" NEO
 SG_ MINUTE : 40|6@1+ (1,0) [0|59] "min" NEO
 SG_ BOOT_STATE : 46|2@1+ (1,0) [0|3] "" NEO
 SG_ GTW_updateInProgress : 48|2@1+ (1,0) [0|3] "" NEO
 SG_ DOOR_STATE_FrontTrunk : 50|2@1+ (1,0) [0|3] "" NEO
 SG_ MCU_factoryMode : 52|1@1+ (1,0) [0|1] "" NEO
 SG_ MCU_transportModeOn : 53|1@0+ (1,0) [0|1] "" NEO
 SG_ BC_headLightLStatus : 55|2@0+ (1,0) [0|3] "" NEO
 SG_ BC_headLightRStatus : 57|2@0+ (1,0) [0|3] "" NEO
 SG_ BC_indicatorLStatus : 59|2@0+ (1,0) [0|3] "" NEO
 SG_ BC_indicatorRStatus : 61|2@0+ (1,0) [0|3] "" NEO

BO_ 872 DI_state: 8 DI
 SG_ DI_systemState : 0|3@1+ (1,0) [0|0] ""  NEO
 SG_ DI_vehicleHoldState : 3|3@1+ (1,0) [0|0] ""  NEO
 SG_ DI_proximity : 6|1@1+ (1,0) [0|0] ""  NEO
 SG_ DI_driveReady : 7|1@1+ (1,0) [0|0] ""  NEO
 SG_ DI_regenLight : 8|1@1+ (1,0) [0|0] ""  NEO
 SG_ DI_state : 9|3@1+ (1,0) [0|0] ""  NEO
 SG_ DI_cruiseState : 12|4@1+ (1,0) [0|0] ""  NEO
 SG_ DI_analogSpeed : 16|12@1+ (0.1,0) [0|150] "speed"  NEO
 SG_ DI_immobilizerState : 28|3@1+ (1,0) [0|0] ""  NEO
 SG_ DI_speedUnits : 31|1@1+ (1,0) [0|1] ""  NEO
 SG_ DI_cruiseSet : 32|9@1+ (0.5,0) [0|255.5] "speed"  NEO
 SG_ DI_aebState : 41|3@1+ (1,0) [0|0] ""  NEO
 SG_ DI_stateCounter : 44|4@1+ (1,0) [0|0] ""  NEO
 SG_ DI_digitalSpeed : 48|8@1+ (1,0) [0|250] ""  NEO
 SG_ DI_stateChecksum : 56|8@1+ (1,0) [0|0] ""  NEO

BO_ 109 SBW_RQ_SCCM: 4 STW
 SG_ StW_Sw_Stat3 : 0|3@1+ (1,0) [0|0] "" NEO
 SG_ MsgTxmtId : 6|2@1+ (1,0) [0|0] "" NEO
 SG_ TSL_RND_Posn_StW : 8|4@1+ (1,0) [0|0] "" NEO
 SG_ TSL_P_Psd_StW : 12|2@1+ (1,0) [0|0] "" NEO
 SG_ MC_SBW_RQ_SCCM : 20|4@1+ (1,0) [0|15] "" NEO
 SG_ CRC_SBW_RQ_SCCM : 24|8@1+ (1,0) [0|0] "" NEO

BO_ 69 STW_ACTN_RQ: 8 STW
 SG_ SpdCtrlLvr_Stat : 0|6@1+ (1,0) [0|0] "" NEO
 SG_ VSL_Enbl_Rq : 6|1@1+ (1,0) [0|0] "" NEO
 SG_ SpdCtrlLvrStat_Inv : 7|1@1+ (1,0) [0|0] "" NEO
 SG_ DTR_Dist_Rq : 8|8@1+ (1,0) [0|200] "" NEO
 SG_ TurnIndLvr_Stat : 16|2@1+ (1,0) [0|0] "" NEO
 SG_ HiBmLvr_Stat : 18|2@1+ (1,0) [0|0] "" NEO
 SG_ WprWashSw_Psd : 20|2@1+ (1,0) [0|0] "" NEO
 SG_ WprWash_R_Sw_Posn_V2 : 22|2@1+ (1,0) [0|0] "" NEO
 SG_ StW_Lvr_Stat : 24|3@1+ (1,0) [0|0] "" NEO
 SG_ StW_Cond_Flt : 27|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Cond_Psd : 28|2@1+ (1,0) [0|0] "" NEO
 SG_ HrnSw_Psd : 30|2@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw00_Psd : 32|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw01_Psd : 33|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw02_Psd : 34|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw03_Psd : 35|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw04_Psd : 36|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw05_Psd : 37|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw06_Psd : 38|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw07_Psd : 39|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw08_Psd : 40|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw09_Psd : 41|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw10_Psd : 42|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw11_Psd : 43|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw12_Psd : 44|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw13_Psd : 45|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw14_Psd : 46|1@1+ (1,0) [0|0] "" NEO
 SG_ StW_Sw15_Psd : 47|1@1+ (1,0) [0|0] "" NEO
 SG_ WprSw6Posn : 48|3@1+ (1,0) [0|0] "" NEO
 SG_ MC_STW_ACTN_RQ : 52|4@1+ (1,0) [0|15] "" NEO
 SG_ CRC_STW_ACTN_RQ : 56|8@1+ (1,0) [0|0] "" NEO

BO_ 643 BODY_R1: 8 GTW
 SG_ AirTemp_Insd : 47|8@0+ (0.25,0) [0|63.5] "C"  NEO
 SG_ AirTemp_Outsd : 63|8@0+ (0.5,-40) [-40|86.5] "C"  NEO
 SG_ Bckl_Sw_RL_Stat_SAM_R : 49|2@0+ (1,0) [-1|4] ""  NEO
 SG_ Bckl_Sw_RM_Stat_SAM_R : 53|2@0+ (1,0) [-1|4] ""  NEO
 SG_ Bckl_Sw_RR_Stat_SAM_R : 51|2@0+ (1,0) [-1|4] ""  NEO
 SG_ DL_RLtch_Stat : 9|2@0+ (1,0) [-1|4] ""  NEO
 SG_ DrRLtch_FL_Stat : 1|2@0+ (1,0) [-1|4] ""  NEO
 SG_ DrRLtch_FR_Stat : 3|2@0+ (1,0) [-1|4] ""  NEO
 SG_ DrRLtch_RL_Stat : 5|2@0+ (1,0) [-1|4] ""  NEO
 SG_ DrRLtch_RR_Stat : 7|2@0+ (1,0) [-1|4] ""  NEO
 SG_ EngHd_Stat : 11|2@0+ (1,0) [-1|4] ""  NEO
 SG_ LoBm_On_Rq : 32|1@0+ (1,0) [0|1] ""  NEO
 SG_ HiBm_On : 33|1@0+ (1,0) [0|1] ""  NEO
 SG_ Hrn_On : 26|1@0+ (1,0) [0|1] ""  NEO
 SG_ IrLmp_D_Lt_Flt : 34|1@0+ (1,0) [0|1] ""  NEO
 SG_ IrLmp_P_Rt_Flt : 35|1@0+ (1,0) [0|1] ""  NEO
 SG_ LgtSens_Twlgt : 18|3@0+ (1,0) [0|7] "Steps"  NEO
 SG_ LgtSens_SNA : 19|1@0+ (1,0) [0|1] ""  NEO
 SG_ LgtSens_Tunnel : 20|1@0+ (1,0) [0|1] ""  NEO
 SG_ LgtSens_Flt : 21|1@0+ (1,0) [0|1] ""  NEO
 SG_ LgtSens_Night : 22|1@0+ (1,0) [-1|2] ""  NEO
 SG_ ADL_LoBm_On_Rq : 23|1@0+ (1,0) [0|1] ""  NEO
 SG_ LoBm_D_Lt_Flt : 36|1@0+ (1,0) [0|1] ""  NEO
 SG_ LoBm_P_Rt_Flt : 37|1@0+ (1,0) [0|1] ""  NEO
 SG_ MPkBrk_Stat : 28|1@0+ (1,0) [-1|2] ""  NEO
 SG_ RevGr_Engg : 39|2@0+ (1,0) [-1|4] ""  NEO
 SG_ StW_Cond_Stat : 55|2@0+ (1,0) [-1|4] ""  NEO
 SG_ Term54_Actv : 27|1@0+ (1,0) [0|1] ""  NEO
 SG_ Trlr_Stat : 25|2@0+ (1,0) [-1|4] ""  NEO
 SG_ VTA_Alm_Actv : 13|1@0+ (1,0) [0|1] ""  NEO
 SG_ WprOutsdPkPosn : 29|1@0+ (1,0) [0|1] ""  NEO

BO_ 760 UI_gpsVehicleSpeed: 8 GTW
 SG_ UI_gpsHDOP : 0|8@1+ (0.1,0) [0|25.5] "1" DAS
 SG_ UI_gpsVehicleHeading : 8|16@1+ (0.0078125,0) [0|511.9921875] "deg" DAS
 SG_ UI_gpsVehicleSpeed : 24|16@1+ (0.00390625,0) [0|250.996] "km/hr" Vector__XXX
 SG_ UI_userSpeedOffset : 40|6@1+ (1,-30) [-30|33] "kph/mph" DAS
 SG_ UI_mapSpeedLimitUnits : 46|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_userSpeedOffsetUnits : 47|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_mppSpeedLimit : 48|5@1+ (5,0) [0|155] "kph/mph" DAS
 SG_ UI_gpsNmeaMIA : 53|1@1+ (1,0) [0|0] "" DAS

BO_ 536 MCU_chassisControl: 8 GTW
 SG_ MCU_dasDebugEnable : 0|1@1+ (1,0) [0|0] "" NEO
 SG_ MCU_parkBrakeRequest : 1|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_trailerModeCH : 3|1@1+ (1,0) [0|0] "" NEO
 SG_ MCU_fcwSensitivity : 4|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_fcwEnable : 6|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_latControlEnable : 8|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_accOvertakeEnable : 10|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_ldwEnable : 12|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_aebEnable : 14|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_bsdEnable : 16|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_ahlbEnable : 18|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_parkSetting : 20|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_pedalSafetyEnable : 22|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_frontDefrostReq_das : 24|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_autoParkRequest : 26|4@1+ (1,0) [0|0] "" NEO
 SG_ MCU_redLightStopSignEnable : 30|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_enableCreepTorqueCH : 32|1@1+ (1,0) [0|0] "" NEO
 SG_ MCU_narrowGarages : 33|1@1+ (1,0) [0|0] "" NEO
 SG_ MCU_rebootAutopilot : 34|1@1+ (1,0) [0|0] "" NEO
 SG_ MCU_enableAutowipers : 35|1@1+ (1,0) [0|0] "" NEO
 SG_ MCU_overPaintedUSS : 38|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_selfParkTune : 40|4@1+ (1,0) [0|15] "" NEO
 SG_ MCU_towModeEnable : 44|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_zeroSpeedConfirmed : 46|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_aesEnable : 48|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_autoLaneChangeEnable : 50|2@1+ (1,0) [0|0] "" NEO
 SG_ MCU_chassisControlCounter : 52|4@1+ (1,0) [0|0] "" NEO
 SG_ MCU_chassisControlChecksum : 56|8@1+ (1,0) [0|0] "" NEO

BO_ 904 MCU_clusterBacklightRequest: 3 NEO
 SG_ MCU_clusterBacklightOn : 7|1@1+ (1,0) [0|1] "" NEO
 SG_ MCU_clusterBrightnessLevel : 8|8@1+ (0.5,0) [0|127.5] "%" NEO
 SG_ MCU_clusterReadyForDrive : 6|1@1+ (1,0) [-1|2] ""  NEO
 SG_ MCU_clusterReadyForPowerOff : 5|1@1+ (1,0) [0|1] "" NEO

BO_ 984 MCU_locationStatus: 8 MCU
 SG_ MCU_gpsAccuracy : 57|7@1+ (0.2,0) [0|0] "m" NEO
 SG_ MCU_latitude : 0|28@1- (1E-06,0) [0|0] "deg" NEO
 SG_ MCU_longitude : 28|29@1- (1E-06,0) [0|0] "deg" NEO

BO_ 104 MCU_locationStatus2: 8 MCU
 SG_ MCU_elevation : 0|32@1- (0.1,0) [0|0] "m" GTW
 SG_ MCU_navigonExpectedSpeed : 32|7@1+ (1,0) [0|126] "mph" GTW

BO_ 840 GTW_status: 8 GTW
 SG_ GTW_accGoingDown : 6|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_accRailReq : 8|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_brakePressed : 1|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_driveGoingDown : 7|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_driveRailReq : 0|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_driverIsLeaving : 5|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_driverPresent : 2|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_hvacGoingDown : 11|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_hvacRailReq : 9|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_icPowerOff : 4|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_notEnough12VForDrive : 3|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_preconditionRequest : 10|1@0+ (1,0) [0|1] ""  NEO
 SG_ GTW_statusChecksum : 63|8@0+ (1,0) [0|255] ""  NEO
 SG_ GTW_statusCounter : 51|4@0+ (1,0) [0|15] ""  NEO

BO_ 920 GTW_carConfig: 8 GTW
 SG_ GTW_performanceConfig : 2|3@0+ (1,0) [0|0] "" NEO
 SG_ GTW_fourWheelDrive : 4|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_unknown1 : 5|1@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_dasHw : 7|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_parkAssistInstalled : 9|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_forwardRadarHw : 11|2@0+ (1,0) [0|0] "" NEO
 SG_ GTW_airSuspensionInstalled : 14|3@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_unknown2 : 15|1@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_country : 23|16@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_parkSensorGeometryType : 33|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_rhd : 34|1@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_bodyControlsType : 35|1@0+ (1,0) [0|0] "" NEO
 SG_ GTW_radarPosition : 39|4@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_rearCornerRadarHw : 41|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_frontCornerRadarHw : 43|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_epasType : 45|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_chassisType : 47|2@0+ (1,0) [0|2] ""  NEO
 SG_ GTW_wheelType : 52|5@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_rearSeatControllerMask : 55|3@0+ (1,0) [0|7] ""  NEO
 SG_ GTW_euVehicle : 56|1@0+ (1,0) [0|0] "" NEO
 SG_ GTW_foldingMirrorsInstalled : 57|1@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_brakeHwType : 59|2@0+ (1,0) [0|2] ""  NEO
 SG_ GTW_autopilot : 61|2@0+ (1,0) [0|0] ""  NEO
 SG_ GTW_unknown3 : 63|2@0+ (1,0) [0|0] ""  NEO

BO_ 1006 UI_autopilotControl: 8 GTW
 SG_ UI_autopilotControlIndex M : 0|3@1+ (1,0) [0|7] "" APP,APS
 SG_ UI_hovEnabled m0 : 3|1@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_donDisableAutoWiperDuration m0 : 4|3@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_donDisableOnAutoWiperSpeed m0 : 7|4@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_blindspotMinSpeed m0 : 11|4@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_blindspotDistance m0 : 15|3@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_blindspotTTC m0 : 18|3@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_donStopEndOfRampBuffer m0 : 21|3@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_donDisableCutin m0 : 24|1@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_donMinGoreWidthForAbortMap m0 : 25|4@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_donAlcProgGoreAbortThres m0 : 29|4@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_donMinGoreWidthForAbortNotMap m0 : 33|4@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcDisableUltrasonicCheck m0 : 37|1@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcUltrasonicDistance m0 : 38|4@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcUltrasonicWaitTime m0 : 42|3@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcEgoLeadingReactionAccel m0 : 48|2@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcMergIntervalRearDHyst m0 : 50|2@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcMergingIntervalHeadwayHyst m0 : 52|2@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcAssertivenessRate m0 : 54|2@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_alcViewRangeSensitivity m0 : 56|2@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_camBlockLaneCheckDisable m1 : 3|1@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_camBlockLaneCheckThreshold m1 : 4|6@1+ (0.01587,0) [0|1] "%" APP,APS
 SG_ UI_camBlockBlurDisable m1 : 10|1@1+ (1,0) [0|0] "" APP,APS
 SG_ UI_camBlockBlurThreshold m1 : 11|6@1+ (0.01587,0) [0|1] "%" APP,APS

BO_ 728 UI_csaOfframpCurvature: 8 GTW
 SG_ UI_csaOfframpCurvC2 : 0|16@1- (1E-06,0) [-0.032768|0.032767] "1/m" DAS
 SG_ UI_csaOfframpCurvC3 : 16|16@1- (4E-09,0) [-0.000131072|0.000131068] "1/m2" DAS
 SG_ UI_csaOfframpCurvRange : 32|8@1+ (2,0) [0|510] "m" DAS
 SG_ UI_csaOfframpCurvCounter : 40|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ UI_csaOfframpCurvUsingTspline : 48|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_csaOfframpCurvReserved : 49|7@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_csaOfframpCurvChecksum : 56|8@1+ (1,0) [0|0] "" Vector__XXX

BO_ 744 UI_csaRoadCurvature: 8 GTW
 SG_ UI_csaRoadCurvC2 : 0|16@1- (1E-06,0) [-0.032768|0.032767] "1/m" DAS
 SG_ UI_csaRoadCurvC3 : 16|16@1- (4E-09,0) [-0.000131072|0.000131068] "1/m2" DAS
 SG_ UI_csaRoadCurvRange : 32|8@1+ (2,0) [0|510] "m" DAS
 SG_ UI_csaRoadCurvCounter : 40|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ UI_csaRoadCurvUsingTspline : 48|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_csaRoadCurvReserved : 49|7@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_csaRoadCurvChecksum : 56|8@1+ (1,0) [0|0] "" Vector__XXX

BO_ 1080 UI_driverAssistAnonDebugParams: 8 GTW
 SG_ UI_anonDebugParam1 : 0|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_anonDebugFlag1 : 7|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_anonDebugParam2 : 8|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_anonDebugFlag2 : 15|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_anonDebugParam3 : 16|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_anonDebugFlag3 : 23|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_anonDebugParam4 : 24|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_anonDebugFlag4 : 31|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_anonDebugParam5 : 32|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_anonDebugParam6 : 40|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_anonDebugParam7 : 48|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_visionSpeedSlider : 56|7@1+ (1,0) [0|100] "" DAS

BO_ 1000 UI_driverAssistControl: 8 GTW
 SG_ UI_autopilotControlRequest : 0|1@1+ (1,0) [1|0] "" DAS
 SG_ UI_ulcStalkConfirm : 1|1@1+ (1,0) [1|0] "" DAS
 SG_ UI_summonHeartbeat : 2|2@1+ (1,0) [0|0] "" DAS
 SG_ UI_curvSpeedAdaptDisable : 4|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_dasDeveloper : 5|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_enableVinAssociation : 6|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_lssLkaEnabled : 7|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_lssLdwEnabled : 8|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_autoSummonEnable : 10|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_exceptionListEnable : 11|1@1+ (1,0) [0|1] "" APP
 SG_ UI_roadCheckDisable : 12|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_driveOnMapsEnable : 13|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_handsOnRequirementDisable : 14|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_forksEnable : 15|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_fuseLanesDisable : 16|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_fuseHPPDisable : 17|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_fuseVehiclesDisable : 18|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_enableNextGenACC : 19|1@1+ (1,0) [0|1] "" APP
 SG_ UI_visionSpeedType : 20|2@1+ (1,0) [0|0] "" APP
 SG_ UI_curvatureDatabaseOnly : 22|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_lssElkEnabled : 23|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_summonExitType : 24|2@1+ (1,0) [0|3] "" DAS
 SG_ UI_summonEntryType : 26|2@1+ (1,0) [0|3] "" DAS
 SG_ UI_selfParkRequest : 28|4@1+ (1,0) [0|15] "" DAS,PARK
 SG_ UI_summonReverseDist : 32|6@1+ (1,0) [0|63] "" DAS
 SG_ UI_undertakeAssistEnable : 38|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_adaptiveSetSpeedEnable : 39|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_drivingSide : 40|2@1+ (1,0) [0|3] "" DAS
 SG_ UI_enableClipTelemetry : 42|1@1+ (1,0) [0|0] "" APP
 SG_ UI_enableTripTelemetry : 43|1@1+ (1,0) [0|0] "" APP
 SG_ UI_enableRoadSegmentTelemetry : 44|1@1+ (1,0) [0|0] "" APP
 SG_ UI_followNavRouteEnable : 46|1@1+ (1,0) [0|0] "" APP
 SG_ UI_ulcSpeedConfig : 48|2@1+ (1,0) [0|3] "" APP
 SG_ UI_ulcBlindSpotConfig : 50|2@1+ (1,0) [0|3] "" APP
 SG_ UI_autopilotAlwaysOn : 52|1@1+ (1,0) [0|1] "" APP
 SG_ UI_accFromZero : 53|1@1+ (1,0) [0|1] "" APP
 SG_ UI_alcOffHighwayEnable : 54|1@1+ (1,0) [0|1] "" APP
 SG_ UI_validationLoop : 55|1@1+ (1,0) [0|1] "" APP
 SG_ UI_ulcOffHighway : 56|1@1+ (1,0) [0|1] "" APP
 SG_ UI_enableNavRouteCSA : 57|1@1+ (1,0) [0|1] "" APP
 SG_ UI_enableCutinExperiments : 58|1@1+ (1,0) [0|1] "" APP
 SG_ UI_source3D : 60|3@1+ (1,0) [0|7] "" APP
 SG_ UI_enableVisionOnlyStops : 63|1@1+ (1,0) [0|1] "" APP

BO_ 968 UI_driverAssistMapData: 8 GTW
 SG_ UI_mapSpeedLimitDependency : 0|3@1+ (1,0) [0|0] "" DAS
 SG_ UI_roadClass : 3|3@1+ (1,0) [0|0] "" DAS
 SG_ UI_inSuperchargerGeofence : 6|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_mapSpeedUnits : 7|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_mapSpeedLimit : 8|5@1+ (1,0) [0|0] "" DAS
 SG_ UI_mapSpeedLimitType : 13|3@1+ (1,0) [0|0] "" DAS
 SG_ UI_countryCode : 16|10@1+ (1,0) [0|0] "" DAS
 SG_ UI_streetCount : 26|2@1+ (1,0) [0|0] "" DAS
 SG_ UI_gpsRoadMatch : 28|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_navRouteActive : 29|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_parallelAutoparkEnabled : 30|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_perpendicularAutoparkEnabled : 31|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_nextBranchDist : 32|5@1+ (10,0) [0|300] "m" DAS
 SG_ UI_controlledAccess : 37|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_nextBranchLeftOffRamp : 38|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_nextBranchRightOffRamp : 39|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectLeftLane : 40|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectRightLane : 41|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectHPP : 42|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectNav : 43|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectLeftFreeSpace : 44|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectRightFreeSpace : 45|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectAutosteer : 46|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_rejectHandsOn : 47|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_acceptBottsDots : 48|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_autosteerRestricted : 49|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_pmmEnabled : 50|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_scaEnabled : 51|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_mapDataCounter : 52|4@1+ (1,0) [0|0] "" DAS
 SG_ UI_mapDataChecksum : 56|8@1+ (1,0) [0|0] "" DAS

BO_ 568 UI_driverAssistRoadSign: 8 GTW
 SG_ UI_roadSign M : 0|8@1+ (1,0) [0|0] "" DAS
 SG_ UI_splineLocConfidence : 40|7@1+ (1,0) [0|100] "" DAS
 SG_ UI_splineID : 48|4@1+ (1,0) [0|15] "" Vector__XXX
 SG_ UI_roadSignCounter : 52|4@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_roadSignChecksum : 56|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_dummyData m0 : 8|1@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_stopSignStopLineDist m1 : 8|10@1+ (0.25,-8) [-8|247.5] "m" Vector__XXX
 SG_ UI_stopSignStopLineConf m1 : 18|7@1+ (1,0) [0|100] "" Vector__XXX
 SG_ UI_trafficLightStopLineDist m2 : 8|10@1+ (0.25,-8) [-8|247.5] "m" Vector__XXX
 SG_ UI_trafficLightStopLineConf m2 : 18|7@1+ (1,0) [0|100] "" Vector__XXX
 SG_ UI_baseMapSpeedLimitMPS m3 : 8|8@1+ (0.25,0) [0|63.75] "m/s" DAS
 SG_ UI_bottomQrtlFleetSpeedMPS m3 : 16|8@1+ (0.25,0) [0|63.75] "m/s" DAS
 SG_ UI_topQrtlFleetSpeedMPS m3 : 24|8@1+ (0.25,0) [0|63.75] "m/s" DAS
 SG_ UI_meanFleetSplineSpeedMPS m4 : 8|8@1+ (0.25,0) [0|63.75] "m/s" DAS
 SG_ UI_medianFleetSpeedMPS m4 : 16|8@1+ (0.25,0) [0|63.75] "m/s" DAS
 SG_ UI_meanFleetSplineAccelMPS2 m4 : 24|8@1+ (0.05,-6.35) [-6.35|6.4] "m/s^2" DAS
 SG_ UI_rampType m4 : 32|3@1+ (1,0) [0|7] "" DAS
 SG_ UI_currSplineIdFull m5 : 8|32@1+ (1,0) [0|1] "" APP


BO_ 696 UI_radarMapData: 8 GTW
 SG_ UI_radarTargetDx : 0|8@1+ (1,-95) [-95|160] "m" DAS
 SG_ UI_radarTargetDxEnd : 8|8@1+ (1,0) [0|255] "m" DAS
 SG_ UI_radarTargetTrustMap : 16|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_radarEnableBraking : 17|1@1+ (1,0) [0|1] "" DAS
 SG_ UI_radarMapDataCounter : 52|4@1+ (1,0) [0|0] "" DAS
 SG_ UI_radarMapDataChecksum : 56|8@1+ (1,0) [0|0] "" DAS

BO_ 712 UI_roadCurvature: 8 GTW
 SG_ UI_roadCurvC0 : 0|11@1- (0.02,0) [-20.48|20.46] "m" DAS
 SG_ UI_roadCurvC1 : 11|10@1- (0.00075,0) [-0.384|0.38325] "1" DAS
 SG_ UI_roadCurvC2 : 21|14@1- (7.5E-06,0) [-0.03072|0.03071625] "1/m" DAS
 SG_ UI_roadCurvC3 : 35|13@1- (3E-08,0) [-0.00012288|0.00012285] "1/m2" DAS
 SG_ UI_roadCurvRange : 48|6@1+ (4,0) [0|252] "m" DAS
 SG_ UI_roadCurvHealth : 54|2@1+ (1,0) [0|0] "" DAS
 SG_ UI_roadCurvChecksum : 56|8@1+ (1,0) [0|0] "" Vector__XXX

BO_ 582 UI_solarData: 5 GTW
 SG_ UI_solarAzimuthAngle : 0|16@1- (1,0) [0|360] "deg" APP
 SG_ UI_solarAzimuthAngleCarRef : 16|9@1- (1,0) [-180|180] "deg" APP
 SG_ UI_isSunUp : 25|1@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_solarElevationAngle : 32|8@1- (1,0) [-90|90] "deg" APP

BO_ 824 UI_status: 8 GTW
 SG_ UI_touchActive : 0|1@1+ (1,0) [0|0] "" IC
 SG_ UI_audioActive : 1|1@1+ (1,0) [0|0] "" IC
 SG_ UI_bluetoothActive : 2|1@1+ (1,0) [0|0] "" IC
 SG_ UI_cellActive : 3|1@1+ (1,0) [0|0] "" IC
 SG_ UI_displayReady : 4|1@1+ (1,0) [0|0] "" IC
 SG_ UI_gpsActive : 5|1@1+ (1,0) [0|0] "" IC
 SG_ UI_wifiConnected : 6|1@1+ (1,0) [0|0] "" IC,APP
 SG_ UI_systemActive : 7|1@1+ (1,0) [0|0] "" IC
 SG_ UI_xmActive : 8|1@1+ (1,0) [0|0] "" IC
 SG_ UI_displayOn : 9|1@1+ (1,0) [0|0] "" IC,APP
 SG_ UI_readyForDrive : 10|1@1+ (1,0) [0|0] "" IC
 SG_ UI_cellConnected : 11|1@1+ (1,0) [0|0] "" IC,APP
 SG_ UI_vpnActive : 12|1@1+ (1,0) [0|0] "" IC,APP
 SG_ UI_wifiActive : 13|1@1+ (1,0) [0|0] "" IC
 SG_ UI_cameraActive : 14|1@1+ (1,0) [0|0] "" IC,APP
 SG_ UI_usbActive : 15|1@1+ (1,0) [0|0] "" IC
 SG_ UI_screenshotActive : 16|1@1+ (1,0) [0|0] "" IC,APP
 SG_ UI_monitorModemPower : 17|1@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_factoryReset : 18|2@1+ (1,0) [0|3] "" Vector__XXX
 SG_ UI_cellNetworkTechnology : 20|4@1+ (1,0) [0|15] "" APP
 SG_ UI_tegraCoreTemperature : 24|8@1+ (1,-64) [0|0] "deg C" IC
 SG_ UI_tegraAmbientTemperature : 32|8@1+ (1,-64) [0|0] "deg C" IC
 SG_ UI_googleWifiUsages : 40|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_autopilotTrial : 48|2@1+ (1,0) [0|0] "" APP
 SG_ UI_cellSignalBars : 50|3@1+ (1,0) [0|7] "" APP
 SG_ UI_hardwareType : 53|2@1+ (1,0) [0|3] "" APP
 SG_ UI_developmentCar : 55|1@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_cellReceiverPower : 56|8@1+ (1,-128) [-128|127] "dB" APP

BO_ 1064 UI_telemetryControl: 8 GTW
 SG_ UI_TCR_enable : 0|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_moveStateStanding : 1|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_moveStateStopped : 2|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_moveStateMoving : 3|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_moveStateIndeterm : 4|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_classConstElem : 5|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_classMovingPed : 6|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_classMovingTwoWheel : 7|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_classMovingFourWheel : 8|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_classUnknown : 9|1@1+ (1,0) [0|0] "" DAS
 SG_ UI_TCR_downSampleFactor : 16|4@1+ (1,0) [0|15] "" Vector__XXX
 SG_ UI_TCR_wExist : 24|5@1+ (1,0) [0|31] "" Vector__XXX
 SG_ UI_TCR_vehSpeed : 32|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ UI_TCR_minRCS : 40|8@1+ (0.25,-14) [-14|49.75] "dB" Vector__XXX
 SG_ UI_TCR_maxDy : 48|5@1+ (0.5,0) [0|15.5] "m" Vector__XXX
 SG_ UI_TCR_maxObjects : 56|5@1+ (1,0) [0|31] "" Vector__XXX
 SG_ UI_TCR_maxRoadClass : 61|3@1+ (1,0) [0|7] "" Vector__XXX

BO_ 522 BrakeMessage: 8 XXX
 SG_ driverBrakeStatus : 2|2@1+ (1,0) [0|3] "" XXX

BO_ 921 AutopilotStatus: 8 XXX
 SG_ autopilotStatus : 0|4@1+ (1,0) [0|0] "" XXX
 SG_ DAS_blindSpotRearLeft : 4|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_blindSpotRearRight : 6|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_fusedSpeedLimit : 8|5@1+ (5,0) [0|150] "kph/mph" XXX
 SG_ DAS_suppressSpeedWarning : 13|1@1+ (1,0) [0|0] "" XXX
 SG_ DAS_summonObstacle : 14|1@1+ (1,0) [0|0] "" XXX
 SG_ DAS_summonClearedGate : 15|1@1+ (1,0) [0|0] "" XXX
 SG_ DAS_visionOnlySpeedLimit : 16|5@1+ (5,0) [0|150] "kph/mph" XXX
 SG_ DAS_heaterState : 21|1@1+ (1,0) [0|0] "" XXX
 SG_ DAS_forwardCollisionWarning : 22|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_autoparkReady : 24|1@1+ (1,0) [0|1] "" XXX
 SG_ DAS_autoParked : 25|1@1+ (1,0) [0|0] "" XXX
 SG_ DAS_autoparkWaitingForBrake : 26|1@1+ (1,0) [0|1] "" XXX
 SG_ DAS_summonFwdLeashReached : 28|1@1+ (1,0) [0|1] "" XXX
 SG_ DAS_summonRvsLeashReached : 29|1@1+ (1,0) [0|1] "" XXX
 SG_ DAS_sideCollisionAvoid : 30|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_sideCollisionWarning : 32|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_sideCollisionInhibit : 34|1@1+ (1,0) [0|0] "" XXX
 SG_ DAS_csaState : 35|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_laneDepartureWarning : 37|3@1+ (1,0) [0|0] "" XXX
 SG_ DAS_fleetSpeedState : 40|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_autopilotHandsOnState : 42|4@1+ (1,0) [0|0] "" XXX
 SG_ DAS_autoLaneChangeState : 46|5@1+ (1,0) [0|0] "" XXX
 SG_ DAS_summonAvailable : 51|1@1+ (1,0) [0|1] "" XXX
 SG_ DAS_statusCounter : 52|4@1+ (1,0) [0|0] "" XXX
 SG_ DAS_statusChecksum : 56|8@1+ (1,0) [0|0] "" XXX

BO_ 905 DAS_status2: 8 XXX
 SG_ DAS_accSpeedLimit : 0|10@1+ (0.2,0) [0|204.6] "mph" XXX
 SG_ DAS_pmmObstacleSeverity : 10|3@1+ (1,0) [0|7] "" XXX
 SG_ DAS_pmmLoggingRequest : 13|1@1+ (1,0) [0|1] "" XXX
 SG_ DAS_activationFailureStatus : 14|2@1+ (1,0) [0|1] "" XXX
 SG_ DAS_pmmUltrasonicsFaultReason : 16|3@1+ (1,0) [0|7] "" XXX
 SG_ DAS_pmmRadarFaultReason : 19|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_pmmSysFaultReason : 21|3@1+ (1,0) [0|7] "" XXX
 SG_ DAS_pmmCameraFaultReason : 24|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_ACC_report : 26|5@1+ (1,0) [0|0] "" XXX
 SG_ DAS_lssState : 31|3@1+ (1,0) [0|0] "" XXX
 SG_ DAS_radarTelemetry : 34|2@1+ (1,0) [0|0] "" XXX
 SG_ DAS_robState : 36|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_driverInteractionLevel : 38|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_ppOffsetDesiredRamp : 40|8@1+ (0.01,-1.28) [-1.28|1.27] "m" XXX
 SG_ DAS_longCollisionWarning : 48|4@1+ (1,0) [0|15] "" XXX
 SG_ DAS_status2Counter : 52|4@1+ (1,0) [0|0] "" XXX
 SG_ DAS_status2Checksum : 56|8@1+ (1,0) [0|0] "" XXX

BO_ 1001 DAS_bodyControls: 8 XXX
 SG_ DAS_headlightRequest : 0|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_hazardLightRequest : 2|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_wiperSpeed : 4|4@1+ (1,0) [0|15] "" XXX
 SG_ DAS_turnIndicatorRequest : 8|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_highLowBeamDecision : 10|2@1+ (1,0) [0|3] "" XXX
 SG_ DAS_highLowBeamOffReason : 12|3@1+ (1,0) [0|4] "" XXX
 SG_ DAS_turnIndicatorRequestReason : 16|4@1+ (1,0) [0|15] "" XXX
 SG_ DAS_bodyControlsCounter : 52|4@1+ (1,0) [0|15] "" XXX
 SG_ DAS_bodyControlsChecksum : 56|8@1+ (1,0) [0|255] "" XXX

VAL_ 3 StW_Angl 16383 "SNA" ;
VAL_ 3 StW_AnglSens_Id 2 "MUST" 0 "PSBL" 1 "SELF" ;
VAL_ 3 StW_AnglSens_Stat 2 "ERR" 3 "ERR_INI" 1 "INI" 0 "OK" ;
VAL_ 3 StW_AnglSpd 16383 "SNA" ;
VAL_ 14 StW_AnglHP 16383 "SNA" ;
VAL_ 14 StW_AnglHP_Spd 16383 "SNA" ;
VAL_ 14 StW_AnglHP_Sens_Stat 3 "SNA" 2 "ERR" 1 "INI" 0 "OK" ;
VAL_ 14 StW_AnglHP_Sens_Id 3 "SNA" 2 "KOSTAL" 1 "DELPHI" 0 "TEST" ;
VAL_ 69 SpdCtrlLvr_Stat 32 "DN_1ST" 16 "UP_1ST" 8 "DN_2ND" 4 "UP_2ND" 2 "RWD" 1 "FWD" 0 "IDLE" ;
VAL_ 69 DTR_Dist_Rq 255 "SNA" 200 "ACC_DIST_7" 166 "ACC_DIST_6" 133 "ACC_DIST_5" 100 "ACC_DIST_4" 66 "ACC_DIST_3" 33 "ACC_DIST_2" 0 "ACC_DIST_1" ;
VAL_ 69 TurnIndLvr_Stat 3 "SNA" 2 "RIGHT" 1 "LEFT" 0 "IDLE" ;
VAL_ 69 HiBmLvr_Stat 3 "SNA" 2 "HIBM_FLSH_ON_PSD" 1 "HIBM_ON_PSD" 0 "IDLE" ;
VAL_ 69 WprWashSw_Psd 3 "SNA" 2 "WASH" 1 "TIPWIPE" 0 "NPSD" ;
VAL_ 69 WprWash_R_Sw_Posn_V2 3 "SNA" 2 "WASH" 1 "INTERVAL" 0 "OFF" ;
VAL_ 69 StW_Lvr_Stat 4 "STW_BACK" 3 "STW_FWD" 2 "STW_DOWN" 1 "STW_UP" 0 "NPSD" ;
VAL_ 69 StW_Cond_Psd 3 "SNA" 2 "DOWN" 1 "UP" 0 "NPSD" ;
VAL_ 69 HrnSw_Psd 3 "SNA" 2 "NDEF2" 1 "PSD" 0 "NPSD" ;
VAL_ 69 StW_Sw00_Psd 1 "PRESSED" 0 "NOT_PRESSED_SNA" ;
VAL_ 69 StW_Sw01_Psd 1 "PRESSED" 0 "NOT_PRESSED_SNA" ;
VAL_ 69 StW_Sw03_Psd 1 "PRESSED" 0 "NOT_PRESSED_SNA" ;
VAL_ 69 StW_Sw04_Psd 1 "PRESSED" 0 "NOT_PRESSED_SNA" ;
VAL_ 69 WprSw6Posn 7 "SNA" 6 "STAGE2" 5 "STAGE1" 4 "INTERVAL4" 3 "INTERVAL3" 2 "INTERVAL2" 1 "INTERVAL1" 0 "OFF" ;
VAL_ 257 GTW_epasControlType 0 "WITHOUT" 1 "WITH_ANGLE" 3 "WITH_BOTH" 2 "WITH_TORQUE" ;
VAL_ 109 StW_Sw_Stat3 7 "SNA" 6 "NDEF6" 5 "NDEF5" 4 "NDEF4" 3 "PLUS_MINUS" 2 "MINUS" 1 "PLUS" 0 "NPSD" ;
VAL_ 109 MsgTxmtId 3 "NDEF3" 2 "NDEF2" 1 "SCCM" 0 "EWM" ;
VAL_ 109 TSL_RND_Posn_StW 15 "SNA" 8 "D" 6 "INI" 4 "N_DOWN" 2 "N_UP" 1 "R" 0 "IDLE" ;
VAL_ 109 TSL_P_Psd_StW 3 "SNA" 2 "INI" 1 "PSD" 0 "IDLE" ;
VAL_ 257 GTW_epasEmergencyOn 1 "EMERGENCY_POWER" 0 "NONE" ;
VAL_ 257 GTW_epasLDWEnabled 1 "ALLOWED" 0 "INHIBITED" ;
VAL_ 257 GTW_epasPowerMode 0 "DRIVE_OFF" 1 "DRIVE_ON" 3 "LOAD_SHED" 2 "SHUTTING_DOWN" 15 "SNA" ;
VAL_ 257 GTW_epasTuneRequest 1 "DM_COMFORT" 3 "DM_SPORT" 2 "DM_STANDARD" 0 "FAIL_SAFE_DEFAULT" 4 "RWD_COMFORT" 6 "RWD_SPORT" 5 "RWD_STANDARD" 7 "SNA" ;
VAL_ 264 DI_torqueDriver -4096 "SNA" ;
VAL_ 264 DI_torqueMotor -4096 "SNA" ;
VAL_ 264 DI_soptState 7 "SOPT_TEST_SNA" 4 "SOPT_TEST_NOT_RUN" 3 "SOPT_TEST_PASSED" 2 "SOPT_TEST_FAILED" 1 "SOPT_TEST_IN_PROGRESS" 0 "SOPT_PRE_TEST" ;
VAL_ 264 DI_motorRPM -32768 "SNA" ;
VAL_ 264 DI_pedalPos 255 "SNA" ;
VAL_ 280 DI_torqueEstimate -2048 "SNA" ;
VAL_ 280 DI_gear 7 "DI_GEAR_SNA" 4 "DI_GEAR_D" 3 "DI_GEAR_N" 2 "DI_GEAR_R" 1 "DI_GEAR_P" 0 "DI_GEAR_INVALID" ;
VAL_ 280 DI_brakePedal 1 "Applied" 0 "Not_applied" ;
VAL_ 280 DI_vehicleSpeed 4095 "SNA" ;
VAL_ 280 DI_gearRequest 7 "DI_GEAR_SNA" 4 "DI_GEAR_D" 3 "DI_GEAR_N" 2 "DI_GEAR_R" 1 "DI_GEAR_P" 0 "DI_GEAR_INVALID" ;
VAL_ 280 DI_torqueInterfaceFailure 1 "TORQUE_INTERFACE_FAILED" 0 "TORQUE_INTERFACE_NORMAL" ;
VAL_ 280 DI_brakePedalState 3 "SNA" 2 "INVALID" 1 "ON" 0 "OFF" ;
VAL_ 280 DI_epbParkRequest 1 "Park_requested" 0 "No_request" ;
VAL_ 280 DI_epbInterfaceReady 1 "EPB_INTERFACE_READY" 0 "EPB_INTERFACE_NOT_READY" ;
VAL_ 309 ESP_absBrakeEvent 1 "ACTIVE" 0 "NOT_ACTIVE" ;
VAL_ 309 ESP_brakeDiscWipingActive 1 "ACTIVE" 0 "INACTIVE" ;
VAL_ 309 ESP_brakeLamp 0 "OFF" 1 "ON" ;
VAL_ 309 ESP_espFaultLamp 0 "OFF" 1 "ON" ;
VAL_ 309 ESP_espLampFlash 1 "FLASH" 0 "OFF" ;
VAL_ 309 ESP_hillStartAssistActive 1 "ACTIVE" 0 "INACTIVE" 2 "NOT_AVAILABLE" 3 "SNA" ;
VAL_ 309 ESP_absFaultLamp 0 "OFF" 1 "ON" ;
VAL_ 309 ESP_espOffLamp 0 "OFF" 1 "ON" ;
VAL_ 309 ESP_stabilityControlSts 2 "ENGAGED" 3 "FAULTED" 5 "INIT" 4 "NOT_CONFIGURED" 0 "OFF" 1 "ON" ;
VAL_ 309 ESP_tcLampFlash 1 "FLASH" 0 "OFF" ;
VAL_ 568 UI_mapSpeedLimit 31 "SNA" 30 "UNLIMITED" 29 "LESS_OR_EQ_160" 28 "LESS_OR_EQ_150" 27 "LESS_OR_EQ_140" 26 "LESS_OR_EQ_130" 25 "LESS_OR_EQ_120" 24 "LESS_OR_EQ_115" 23 "LESS_OR_EQ_110" 22 "LESS_OR_EQ_105" 21 "LESS_OR_EQ_100" 20 "LESS_OR_EQ_95" 19 "LESS_OR_EQ_90" 18 "LESS_OR_EQ_85" 17 "LESS_OR_EQ_80" 16 "LESS_OR_EQ_75" 15 "LESS_OR_EQ_70" 14 "LESS_OR_EQ_65" 13 "LESS_OR_EQ_60" 12 "LESS_OR_EQ_55" 11 "LESS_OR_EQ_50" 10 "LESS_OR_EQ_45" 9 "LESS_OR_EQ_40" 8 "LESS_OR_EQ_35" 7 "LESS_OR_EQ_30" 6 "LESS_OR_EQ_25" 5 "LESS_OR_EQ_20" 4 "LESS_OR_EQ_15" 3 "LESS_OR_EQ_10" 2 "LESS_OR_EQ_7" 1 "LESS_OR_EQ_5" 0 "UNKNOWN" ;
VAL_ 569 DAS_leftLineUsage 3 "BLACKLISTED" 2 "FUSED" 1 "AVAILABLE" 0 "REJECTED_UNAVAILABLE" ;
VAL_ 569 DAS_rightLineUsage 3 "BLACKLISTED" 2 "FUSED" 1 "AVAILABLE" 0 "REJECTED_UNAVAILABLE" ;
VAL_ 569 DAS_leftFork 3 "LEFT_FORK_UNAVAILABLE" 2 "LEFT_FORK_SELECTED" 1 "LEFT_FORK_AVAILABLE" 0 "LEFT_FORK_NONE" ;
VAL_ 569 DAS_rightFork 3 "RIGHT_FORK_UNAVAILABLE" 2 "RIGHT_FORK_SELECTED" 1 "RIGHT_FORK_AVAILABLE" 0 "RIGHT_FORK_NONE" ;
VAL_ 521 DAS_locMode 3 "DAS_LOC_DRIVERLESS" 2 "DAS_LOC_RESTRICTED" 1 "DAS_LOC_NORMAL" 0 "DAS_LOC_OFF" ;
VAL_ 521 DAS_locState 7 "DAS_LOC_FAULT_SNA" 6 "DAS_LOC_AEB_ACTIVE" 2 "DAS_LOC_CANCEL_SILENT" 1 "DAS_LOC_CANCEL_GENERIC" 0 "DAS_LOC_HEALTHY" ;
VAL_ 521 DAS_locRequest 4 "DAS_RQ_PARK" 3 "DAS_RQ_HOLD" 2 "DAS_RQ_BACKWARD" 1 "DAS_RQ_FORWARD" 0 "DAS_RQ_IDLE" ;
VAL_ 521 DAS_locJerkMin 255 "SNA" ;
VAL_ 521 DAS_locJerkMax 255 "SNA" ;
VAL_ 521 DAS_locSpeed 2047 "SNA" ;
VAL_ 521 DAS_locAccelMin 511 "SNA" ;
VAL_ 521 DAS_locAccelMax 511 "SNA" ;
VAL_ 522 driverBrakeStatus 2 "APPLIED" 1 "NOT_APPLIED" ;
VAL_ 760 UI_mapSpeedLimitUnits 1 "KPH" 0 "MPH" ;
VAL_ 760 UI_userSpeedOffsetUnits 1 "KPH" 0 "MPH" ;
VAL_ 643 AirTemp_Insd 255 "SNA" ;
VAL_ 643 AirTemp_Outsd 254 "INIT" 255 "SNA" ;
VAL_ 643 Bckl_Sw_RL_Stat_SAM_R 2 "FLT" 1 "NOT" 0 "OK" 3 "SNA" ;
VAL_ 643 Bckl_Sw_RM_Stat_SAM_R 2 "FLT" 1 "NOT" 0 "OK" 3 "SNA" ;
VAL_ 643 Bckl_Sw_RR_Stat_SAM_R 2 "FLT" 1 "NOT" 0 "OK" 3 "SNA" ;
VAL_ 643 DL_RLtch_Stat 1 "CLS" 0 "NDEF0" 2 "OPN" 3 "SNA" ;
VAL_ 643 DrRLtch_FL_Stat 1 "CLS" 0 "NDEF0" 2 "OPN" 3 "SNA" ;
VAL_ 643 DrRLtch_FR_Stat 1 "CLS" 0 "NDEF0" 2 "OPN" 3 "SNA" ;
VAL_ 643 DrRLtch_RL_Stat 1 "CLS" 0 "NDEF0" 2 "OPN" 3 "SNA" ;
VAL_ 643 DrRLtch_RR_Stat 1 "CLS" 0 "NDEF0" 2 "OPN" 3 "SNA" ;
VAL_ 643 EngHd_Stat 1 "CLS" 0 "NDEF0" 2 "OPN" 3 "SNA" ;
VAL_ 643 LgtSens_Night 0 "DAY" 1 "NIGHT" ;
VAL_ 643 MPkBrk_Stat 1 "ENGG" 0 "RELS" ;
VAL_ 643 RevGr_Engg 0 "DISENGG" 1 "ENGG" 2 "NDEF2" 3 "SNA" ;
VAL_ 643 StW_Cond_Stat 3 "BLINK" 1 "NDEF1" 0 "OFF" 2 "ON" ;
VAL_ 643 Trlr_Stat 2 "NDEF2" 0 "NONE" 1 "OK" 3 "SNA" ;
VAL_ 697 DAS_setSpeed 4095 "SNA" ;
VAL_ 697 DAS_accState 15 "FAULT_SNA" 13 "ACC_CANCEL_GENERIC_SILENT" 11 "APC_SELFPARK_START" 10 "APC_UNPARK_COMPLETE" 9 "APC_PAUSE" 8 "APC_ABORT" 7 "APC_COMPLETE" 6 "APC_FORWARD" 5 "APC_BACKWARD" 4 "ACC_ON" 3 "ACC_HOLD" 0 "ACC_CANCEL_GENERIC" ;
VAL_ 697 DAS_aebEvent 3 "AEB_SNA" 2 "AEB_FAULT" 1 "AEB_ACTIVE" 0 "AEB_NOT_ACTIVE" ;
VAL_ 697 DAS_jerkMin 511 "SNA" ;
VAL_ 697 DAS_jerkMax 255 "SNA" ;
VAL_ 697 DAS_accelMin 511 "SNA" ;
VAL_ 697 DAS_accelMax 511 "SNA" ;
VAL_ 792 BOOT_STATE 2 "Init" 3 "SNA" 0 "closed" 1 "open" ;
VAL_ 792 CERRD 1 "CAN error detect" 0 "no Can error detected" ;
VAL_ 792 DAY 1 "Init" 0 "SNA" ;
VAL_ 792 DOOR_STATE_FL 2 "Init" 3 "SNA" 0 "closed" 1 "open" ;
VAL_ 792 DOOR_STATE_FR 2 "Init" 3 "SNA" 0 "closed" 1 "open" ;
VAL_ 792 DOOR_STATE_FrontTrunk 2 "Init" 3 "SNA" 0 "closed" 1 "open" ;
VAL_ 792 DOOR_STATE_RL 2 "Init" 3 "SNA" 0 "closed" 1 "open" ;
VAL_ 792 DOOR_STATE_RR 2 "Init" 3 "SNA" 0 "closed" 1 "open" ;
VAL_ 792 GTW_updateInProgress 1 "IN_PROGRESS" 2 "IN_PROGRESS_NOT_USED" 3 "IN_PROGRESS_SNA" 0 "NOT_IN_PROGRESS" ;
VAL_ 792 Hour 30 "Init" 31 "SNA" ;
VAL_ 792 MCU_factoryMode 1 "FACTORY_MODE" 0 "NORMAL_MODE" ;
VAL_ 792 MCU_transportModeOn 0 "NORMAL_MODE" ;
VAL_ 792 MINUTE 62 "Init" 63 "SNA" ;
VAL_ 792 MONTH 1 "Init" 15 "SNA" ;
VAL_ 792 SECOND 62 "Init" 63 "SNA" ;
VAL_ 792 YEAR 126 "Init" 127 "SNA" ;
VAL_ 872 DI_aebState 2 "ENABLED" 4 "FAULT" 7 "SNA" 1 "STANDBY" 3 "STANDSTILL" 0 "UNAVAILABLE" ;
VAL_ 872 DI_analogSpeed 4095 "SNA" ;
VAL_ 872 DI_cruiseState 2 "ENABLED" 5 "FAULT" 0 "OFF" 4 "OVERRIDE" 7 "PRE_CANCEL" 6 "PRE_FAULT" 1 "STANDBY" 3 "STANDSTILL" ;
VAL_ 872 DI_digitalSpeed 255 "SNA" ;
VAL_ 872 DI_immobilizerState 2 "AUTHENTICATING" 3 "DISARMED" 6 "FAULT" 4 "IDLE" 0 "INIT_SNA" 1 "REQUEST" 5 "RESET" ;
VAL_ 872 DI_speedUnits 1 "KPH" 0 "MPH" ;
VAL_ 872 DI_state 3 "ABORT" 4 "ENABLE" 2 "FAULT" 1 "STANDBY" 0 "UNAVAILABLE" ;
VAL_ 872 DI_systemState 3 "ABORT" 4 "ENABLE" 2 "FAULT" 1 "STANDBY" 0 "UNAVAILABLE" ;
VAL_ 872 DI_vehicleHoldState 2 "BLEND_IN" 4 "BLEND_OUT" 6 "FAULT" 7 "INIT" 5 "PARK" 1 "STANDBY" 3 "STANDSTILL" 0 "UNAVAILABLE" ;
VAL_ 880 EPAS_currentTuneMode 1 "DM_COMFORT" 3 "DM_SPORT" 2 "DM_STANDARD" 0 "FAIL_SAFE_DEFAULT" 4 "RWD_COMFORT" 6 "RWD_SPORT" 5 "RWD_STANDARD" 7 "UNAVAILABLE" ;
VAL_ 880 EPAS_eacErrorCode 14 "EAC_ERROR_EPB_INHIBIT" 3 "EAC_ERROR_HANDS_ON" 7 "EAC_ERROR_HIGH_ANGLE_RATE_REQ" 9 "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY" 6 "EAC_ERROR_HIGH_ANGLE_REQ" 8 "EAC_ERROR_HIGH_ANGLE_SAFETY" 10 "EAC_ERROR_HIGH_MMOT_SAFETY" 11 "EAC_ERROR_HIGH_TORSION_SAFETY" 0 "EAC_ERROR_IDLE" 12 "EAC_ERROR_LOW_ASSIST" 2 "EAC_ERROR_MAX_SPEED" 1 "EAC_ERROR_MIN_SPEED" 13 "EAC_ERROR_PINION_VEL_DIFF" 4 "EAC_ERROR_TMP_FAULT" 5 "EAR_ERROR_MAX_STEER_DELTA" 15 "SNA" ;
VAL_ 880 EPAS_eacStatus 2 "EAC_ACTIVE" 1 "EAC_AVAILABLE" 3 "EAC_FAULT" 0 "EAC_INHIBITED" 4 "SNA" ;
VAL_ 880 EPAS_handsOnLevel 0 "0" 1 "1" 2 "2" 3 "3" ;
VAL_ 880 EPAS_steeringFault 1 "FAULT" 0 "NO_FAULT" ;
VAL_ 880 EPAS_steeringRackForce 1022 "NOT_IN_SPEC" 1023 "SNA" ;
VAL_ 880 EPAS_steeringReduced 0 "NORMAL_ASSIST" 1 "REDUCED_ASSIST" ;
VAL_ 880 EPAS_torsionBarTorque 0 "SEE_SPECIFICATION" 4095 "SNA" 4094 "UNDEFINABLE_DATA" ;
VAL_ 904 MCU_clusterReadyForDrive 0 "NO_SNA" 1 "YES" ;
VAL_ 905 DAS_accSpeedLimit 1023 "SNA" 0 "NONE" ;
VAL_ 905 DAS_pmmObstacleSeverity 7 "PMM_SNA" 6 "PMM_ACCEL_LIMIT" 5 "PMM_CRASH_FRONT" 4 "PMM_CRASH_REAR" 3 "PMM_BRAKE_REQUEST" 2 "PMM_IMMINENT_FRONT" 1 "PMM_IMMINENT_REAR" 0 "PMM_NONE" ;
VAL_ 905 DAS_pmmLoggingRequest 1 "TRUE" 0 "FALSE" ;
VAL_ 905 DAS_activationFailureStatus 2 "LC_ACTIVATION_FAILED_2" 1 "LC_ACTIVATION_FAILED_1" 0 "LC_ACTIVATION_IDLE" ;
VAL_ 905 DAS_pmmUltrasonicsFaultReason 4 "PMM_ULTRASONICS_INVALID_MIA" 3 "PMM_ULTRASONICS_BLOCKED_BOTH" 2 "PMM_ULTRASONICS_BLOCKED_REAR" 1 "PMM_ULTRASONICS_BLOCKED_FRONT" 0 "PMM_ULTRASONICS_NO_FAULT" ;
VAL_ 905 DAS_pmmRadarFaultReason 2 "PMM_RADAR_INVALID_MIA" 1 "PMM_RADAR_BLOCKED_FRONT" 0 "PMM_RADAR_NO_FAULT" ;
VAL_ 905 DAS_pmmSysFaultReason 7 "PMM_FAULT_BRAKE_PEDAL_INHIBIT" 6 "PMM_FAULT_ROAD_TYPE" 5 "PMM_FAULT_DISABLED_BY_USER" 4 "PMM_FAULT_STEERING_ANGLE_RATE" 3 "PMM_FAULT_DI_FAULT" 2 "PMM_FAULT_SPEED" 1 "PMM_FAULT_DAS_DISABLED" 0 "PMM_FAULT_NONE" ;
VAL_ 905 DAS_pmmCameraFaultReason 2 "PMM_CAMERA_INVALID_MIA" 1 "PMM_CAMERA_BLOCKED_FRONT" 0 "PMM_CAMERA_NO_FAULT" ;
VAL_ 905 DAS_ACC_report 24 "ACC_REPORT_BEHAVIOR_REPORT" 23 "ACC_REPORT_CAMERA_ONLY" 22 "ACC_REPORT_RADAR_OBJ_FIVE" 21 "ACC_REPORT_CIPV_CUTTING_OUT" 20 "ACC_REPORT_MCVLR_IN_PATH" 19 "ACC_REPORT_MCVLR_DPP" 18 "ACC_REPORT_FLEET_SPEEDS" 17 "ACC_REPORT_TARGET_MCP" 16 "ACC_REPORT_RADAR_OBJ_TWO" 15 "ACC_REPORT_RADAR_OBJ_ONE" 14 "ACC_REPORT_LC_EXTERNAL_STATE_ACTIVE_RESTRICTED" 13 "ACC_REPORT_LC_EXTERNAL_STATE_ABORTED" 12 "ACC_REPORT_LC_EXTERNAL_STATE_ABORTING" 11 "ACC_REPORT_LC_HANDS_ON_REQD_STRUCK_OUT" 10 "ACC_REPORT_CSA" 9 "ACC_REPORT_TARGET_TYPE_FAULT" 8 "ACC_REPORT_TARGET_TYPE_IPSO" 7 "ACC_REPORT_TARGET_TYPE_TRAFFIC_LIGHT" 6 "ACC_REPORT_TARGET_TYPE_STOP_SIGN" 5 "ACC_REPORT_TARGET_CUTIN" 4 "ACC_REPORT_TARGET_MCVR" 3 "ACC_REPORT_TARGET_MCVL" 2 "ACC_REPORT_TARGET_IN_FRONT_OF_CIPV" 1 "ACC_REPORT_TARGET_CIPV" 0 "ACC_REPORT_TARGET_NONE" ;
VAL_ 905 DAS_lssState 7 "LSS_STATE_OFF" 6 "LSS_STATE_ABORT" 5 "LSS_STATE_BLINDSPOT" 4 "LSS_STATE_MONITOR" 3 "LSS_STATE_ELK" 2 "LSS_STATE_LKA" 1 "LSS_STATE_LDW" 0 "LSS_STATE_FAULT" ;
VAL_ 905 DAS_radarTelemetry 2 "RADAR_TELEMETRY_URGENT" 1 "RADAR_TELEMETRY_NORMAL" 0 "RADAR_TELEMETRY_IDLE" ;
VAL_ 905 DAS_robState 3 "ROB_STATE_MAPLESS" 2 "ROB_STATE_ACTIVE" 1 "ROB_STATE_MEASURE" 0 "ROB_STATE_INHIBITED" ;
VAL_ 905 DAS_driverInteractionLevel 2 "CONTINUED_DRIVER_NOT_INTERACTING" 1 "DRIVER_NOT_INTERACTING" 0 "DRIVER_INTERACTING" ;
VAL_ 905 DAS_ppOffsetDesiredRamp 128 "PP_NO_OFFSET" ;
VAL_ 905 DAS_longCollisionWarning 15 "FCM_LONG_COLLISION_WARNING_SNA" 12 "FCM_LONG_COLLISION_WARNING_VEHICLE_CIPV2" 11 "FCM_LONG_COLLISION_WARNING_VEHICLE_MCVR2" 10 "FCM_LONG_COLLISION_WARNING_VEHICLE_MCVR" 9 "FCM_LONG_COLLISION_WARNING_VEHICLE_MCVL2" 8 "FCM_LONG_COLLISION_WARNING_VEHICLE_MCVL" 7 "FCM_LONG_COLLISION_WARNING_VEHICLE_CUTIN" 6 "FCM_LONG_COLLISION_WARNING_VEHICLE_CIPV" 5 "FCM_LONG_COLLISION_WARNING_TFL_STOPLINE" 4 "FCM_LONG_COLLISION_WARNING_STOPSIGN_STOPLINE" 3 "FCM_LONG_COLLISION_WARNING_IPSO" 2 "FCM_LONG_COLLISION_WARNING_PEDESTRIAN" 1 "FCM_LONG_COLLISION_WARNING_VEHICLE_UNKNOWN" 0 "FCM_LONG_COLLISION_WARNING_NONE" ;
VAL_ 921 autopilotStatus 5 "ACTIVE_NAVIGATE_ON_AUTOPILOT" 4 "ACTIVE_2" 3 "ACTIVE_1" 2 "AVAILABLE" 1 "UNAVAILABLE" 0 "DISABLED" ;
VAL_ 921 DAS_blindSpotRearLeft 3 "SNA" 2 "WARNING_LEVEL_2" 1 "WARNING_LEVEL_1" 0 "NO_WARNING" ;
VAL_ 921 DAS_blindSpotRearRight 3 "SNA" 2 "WARNING_LEVEL_2" 1 "WARNING_LEVEL_1" 0 "NO_WARNING" ;
VAL_ 921 DAS_fusedSpeedLimit 31 "NONE" 0 "UNKNOWN_SNA" ;
VAL_ 921 DAS_suppressSpeedWarning 1 "Suppress_Speed_Warning" 0 "Do_Not_Suppress" ;
VAL_ 921 DAS_visionOnlySpeedLimit 31 "NONE" 0 "UNKNOWN_SNA" ;
VAL_ 921 DAS_heaterState 1 "HEATER_ON" 0 "HEATER_OFF_SNA" ;
VAL_ 921 DAS_forwardCollisionWarning 3 "SNA" 1 "FORWARD_COLLISION_WARNING" 0 "NONE" ;
VAL_ 921 DAS_autoparkReady 1 "AUTOPARK_READY" 0 "AUTOPARK_UNAVAILABLE" ;
VAL_ 921 DAS_sideCollisionAvoid 3 "SNA" 2 "AVOID_RIGHT" 1 "AVOID_LEFT" 0 "NONE" ;
VAL_ 921 DAS_sideCollisionWarning 3 "WARN_LEFT_RIGHT" 2 "WARN_RIGHT" 1 "WARN_LEFT" 0 "NONE" ;
VAL_ 921 DAS_sideCollisionInhibit 1 "INHIBIT" 0 "NO_INHIBIT" ;
VAL_ 921 DAS_csaState 3 "CSA_EXTERNAL_STATE_HOLD" 2 "CSA_EXTERNAL_STATE_ENABLE" 1 "CSA_EXTERNAL_STATE_AVAILABLE" 0 "CSA_EXTERNAL_STATE_UNAVAILABLE" ;
VAL_ 921 DAS_laneDepartureWarning 5 "SNA" 4 "RIGHT_WARNING_SEVERE" 3 "LEFT_WARNING_SEVERE" 2 "RIGHT_WARNING" 1 "LEFT_WARNING" 0 "NONE" ;
VAL_ 921 DAS_fleetSpeedState 3 "FLEETSPEED_HOLD" 2 "FLEETSPEED_ACTIVE" 1 "FLEETSPEED_AVAILABLE" 0 "FLEETSPEED_UNAVAILABLE" ;
VAL_ 921 DAS_autopilotHandsOnState 15 "LC_HANDS_ON_SNA" 8 "LC_HANDS_ON_SUSPENDED" 7 "LC_HANDS_ON_REQD_STRUCK_OUT" 5 "LC_HANDS_ON_REQD_CHIME_2" 4 "LC_HANDS_ON_REQD_CHIME_1" 3 "LC_HANDS_ON_REQD_VISUAL" 2 "LC_HANDS_ON_REQD_NOT_DETECTED" 1 "LC_HANDS_ON_REQD_DETECTED" 0 "LC_HANDS_ON_NOT_REQD" ;
VAL_ 921 DAS_autoLaneChangeState 31 "ALC_SNA" 30 "ALC_ABORT_MISSION_PLAN_INVALID" 29 "ALC_ABORT_TIMEOUT" 28 "ALC_WAITING_HANDS_ON" 27 "ALC_BLOCKED_LANE_TYPE_R" 26 "ALC_BLOCKED_LANE_TYPE_L" 25 "ALC_BLOCKED_VEH_TTC_AND_USS_R" 24 "ALC_BLOCKED_VEH_TTC_R" 23 "ALC_BLOCKED_VEH_TTC_AND_USS_L" 22 "ALC_BLOCKED_VEH_TTC_L" 21 "ALC_UNAVAILABLE_SOLID_LANE_LINE" 20 "ALC_ABORT_OTHER_REASON" 19 "ALC_ABORT_BLINKER_TURNED_OFF" 18 "ALC_ABORT_LC_HEALTH_BAD" 17 "ALC_ABORT_POOR_VIEW_RANGE" 16 "ALC_ABORT_SIDE_OBSTACLE_PRESENT_R" 15 "ALC_ABORT_SIDE_OBSTACLE_PRESENT_L" 14 "ALC_WAITING_FOR_FWD_OBST_TO_PASS_R" 13 "ALC_WAITING_FOR_FWD_OBST_TO_PASS_L" 12 "ALC_WAITING_FOR_SIDE_OBST_TO_PASS_R" 11 "ALC_WAITING_FOR_SIDE_OBST_TO_PASS_L" 10 "ALC_IN_PROGRESS_R" 9 "ALC_IN_PROGRESS_L" 8 "ALC_AVAILABLE_BOTH" 7 "ALC_AVAILABLE_ONLY_R" 6 "ALC_AVAILABLE_ONLY_L" 5 "ALC_UNAVAILABLE_VEHICLE_SPEED" 4 "ALC_UNAVAILABLE_EXITING_HIGHWAY" 3 "ALC_UNAVAILABLE_TP_FOLLOW" 2 "ALC_UNAVAILABLE_SONICS_INVALID" 1 "ALC_UNAVAILABLE_NO_LANES" 0 "ALC_UNAVAILABLE_DISABLED" ;
VAL_ 1001 DAS_headlightRequest 3 "DAS_HEADLIGHT_REQUEST_INVALID" 1 "DAS_HEADLIGHT_REQUEST_ON" 0 "DAS_HEADLIGHT_REQUEST_OFF" ;
VAL_ 1001 DAS_hazardLightRequest 3 "DAS_REQUEST_HAZARDS_SNA" 2 "DAS_REQUEST_HAZARDS_UNUSED" 1 "DAS_REQUEST_HAZARDS_ON" 0 "DAS_REQUEST_HAZARDS_OFF" ;
VAL_ 1001 DAS_wiperSpeed 15 "DAS_WIPER_SPEED_INVALID" 14 "DAS_WIPER_SPEED_14" 13 "DAS_WIPER_SPEED_13" 12 "DAS_WIPER_SPEED_12" 11 "DAS_WIPER_SPEED_11" 10 "DAS_WIPER_SPEED_10" 9 "DAS_WIPER_SPEED_9" 8 "DAS_WIPER_SPEED_8" 7 "DAS_WIPER_SPEED_7" 6 "DAS_WIPER_SPEED_6" 5 "DAS_WIPER_SPEED_5" 4 "DAS_WIPER_SPEED_4" 3 "DAS_WIPER_SPEED_3" 2 "DAS_WIPER_SPEED_2" 1 "DAS_WIPER_SPEED_1" 0 "DAS_WIPER_SPEED_OFF" ;
VAL_ 1001 DAS_turnIndicatorRequest 3 "DAS_TURN_INDICATOR_CANCEL" 2 "DAS_TURN_INDICATOR_RIGHT" 1 "DAS_TURN_INDICATOR_LEFT" 0 "DAS_TURN_INDICATOR_NONE" ;
VAL_ 1001 DAS_highLowBeamDecision 3 "DAS_HIGH_BEAM_SNA" 2 "DAS_HIGH_BEAM_ON" 1 "DAS_HIGH_BEAM_OFF" 0 "DAS_HIGH_BEAM_UNDECIDED" ;
VAL_ 1001 DAS_highLowBeamOffReason 5 "HIGH_BEAM_OFF_REASON_SNA" 4 "HIGH_BEAM_OFF_REASON_HEAD_LIGHT" 3 "HIGH_BEAM_OFF_REASON_AMBIENT_LIGHT" 2 "HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET" 1 "HIGH_BEAM_OFF_REASON_MOVING_VISION_TARGET" 0 "HIGH_BEAM_ON" ;
VAL_ 1001 DAS_turnIndicatorRequestReason 6 "DAS_ACTIVE_COMMANDED_LANE_CHANGE" 5 "DAS_CANCEL_FORK" 4 "DAS_CANCEL_LANE_CHANGE" 3 "DAS_ACTIVE_FORK" 2 "DAS_ACTIVE_SPEED_LANE_CHANGE" 1 "DAS_ACTIVE_NAV_LANE_CHANGE" 0 "DAS_NONE" ;
VAL_ 1160 DAS_steeringAngleRequest 16384 "ZERO_ANGLE" ;
VAL_ 1160 DAS_steeringControlType 1 "ANGLE_CONTROL" 3 "DISABLED" 0 "NONE" 2 "RESERVED" ;
VAL_ 1160 DAS_steeringHapticRequest 1 "ACTIVE" 0 "IDLE" ;
)";

class DbcParserTest : public ::testing::Test
{
protected: 
	can::v2c_transcoder ipt;

    void SetUp() override
    {
        // Initialize interpreter or any other setup if needed
    }
};


TEST_F(DbcParserTest, AddMessage)
{
    canid_t message_id = 0x123;
    ipt.add_message(message_id, "TestMessage");

    auto* msg = ipt.find_message(message_id);
    ASSERT_NE(msg, nullptr);
}

TEST_F(DbcParserTest, SetEnvVar)
{
    ipt.set_env_var("TestVar", 42);
    // Assuming there's a way to verify the environment variable was set
}

TEST_F(DbcParserTest, Vin)
{
    std::string vin = ipt.vin();
    EXPECT_TRUE(vin.empty());
}

TEST_F(DbcParserTest, ParseDBC0)
{	
    EXPECT_TRUE(can::parse_dbc(dbc_1, std::ref(ipt)));
}


TEST_F(DbcParserTest, ParseDBC1)
{	
    EXPECT_TRUE(can::parse_dbc(dbc_1, std::ref(ipt)));
}

TEST_F(DbcParserTest, ParseDBC2)
{
    EXPECT_TRUE(can::parse_dbc(dbc_2, std::ref(ipt)));
}

TEST_F(DbcParserTest, ParseDBC3)
{
    EXPECT_TRUE(can::parse_dbc(dbc_3, std::ref(ipt)));
}

TEST_F(DbcParserTest, ParseDBC4)
{
	// Syntax error:
    // => VAL_ 264 DI_torqueDriver -4096 "SNA" ;
	// unsupported ?
    EXPECT_FALSE(can::parse_dbc(dbc_4, std::ref(ipt)));
}
