///////////////////////////////////////////////////////////////////////////////
// FILE:          ThorlabsKinesisBenchtop.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Control of Thorlabs stages using the Kinesis library
//
// COPYRIGHT:     Emilio J. Gualda, 2012
//                Egor Zindy, University of Manchester, 2013
//				  Hugo M.M.	Pereira, Instituto Gulbenkian de Ciencia, 2017
//
// LICENSE:       This file is distributed under the BSD license.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
//
// AUTHOR:        Hugo M.M. Pereira, Instituto Gulbenkian de Ciencia, 2017
//				  Emilio J. Gualda, IGC, 2012
//                Egor Zindy (egor.zindy@manchester.ac.uk)
//                
//
// NOTES:         Needs / Tested with Thorlabs.MotionControl.DeviceManager.dll
//                The following Kinesis controllers have been tested:
//
//                [ ] BSC001 - 1 Ch benchtop stepper driver
//                [X] BSC101 - 1 Ch benchtop stepper driver
//                [ ] BSC002 - 2 Ch benchtop stepper driver
//                [ ] BDC101 - 1 Ch benchtop DC servo driver
//                [ ] SCC001 - 1 Ch stepper driver card (used within BSC102,103 units)
//                [ ] DCC001 - 1 Ch DC servo driver card (used within BDC102,103 units)
//                [ ] ODC001 - 1 Ch DC servo driver cube
//                [ ] OST001 - 1 Ch stepper driver cube
//                [ ] MST601 - 2 Ch modular stepper driver module
//                [ ] TST001 - 1 Ch Stepper driver T-Cube
//                [ ] TDC001 - 1 Ch DC servo driver T-Cube
//				  [ ] KDC001 - 1 Ch DC servo driver K-Cube
//

#ifdef WIN32
#include <windows.h>
#define snprintf _snprintf
#endif

#include "ThorlabsKinesisBenchtop.h"
#include "Thorlabs.MotionControl.Benchtop.StepperMotor.h"
#include <cstdio>
#include <string>
#include <math.h>
#include <sstream>
#include <string.h>

//Short descriptions taken from the APTAPI.h
const char* g_ThorlabsDeviceNameBSC001 = "BSC001";
const char* g_ThorlabsDeviceDescBSC001 = "1 Ch benchtop stepper driver";
const char* g_ThorlabsDeviceNameBSC101 = "BSC101";
const char* g_ThorlabsDeviceDescBSC101 = "1 Ch benchtop stepper driver";
const char* g_ThorlabsDeviceNameBSC201 = "BSC201";
const char* g_ThorlabsDeviceDescBSC201 = "2 Ch benchtop stepper driver";
const char* g_ThorlabsDeviceNameBSC002 = "BSC002";
const char* g_ThorlabsDeviceDescBSC002 = "2 Ch benchtop stepper driver";
const char* g_ThorlabsDeviceNameBDC101 = "BDC101";
const char* g_ThorlabsDeviceDescBDC101 = "1 Ch benchtop DC servo driver";
const char* g_ThorlabsDeviceNameSCC001 = "SCC001";
const char* g_ThorlabsDeviceDescSCC001 = "1 Ch stepper driver card (used within BSC102,103 units)";
const char* g_ThorlabsDeviceNameDCC001 = "DCC001";
const char* g_ThorlabsDeviceDescDCC001 = "1 Ch DC servo driver card (used within BDC102,103 units)";
const char* g_ThorlabsDeviceNameODC001 = "ODC001";
const char* g_ThorlabsDeviceDescODC001 = "1 Ch DC servo driver cube";
const char* g_ThorlabsDeviceNameOST001 = "OST001";
const char* g_ThorlabsDeviceDescOST001 = "1 Ch stepper driver cube";
const char* g_ThorlabsDeviceNameMST601 = "MST601";
const char* g_ThorlabsDeviceDescMST601 = "2 Ch modular stepper driver module";
const char* g_ThorlabsDeviceNameTST001 = "TST001";
const char* g_ThorlabsDeviceDescTST001 = "1 Ch Stepper driver T-Cube";
const char* g_ThorlabsDeviceNameTDC001 = "TDC001";
const char* g_ThorlabsDeviceDescTDC001 = "1 Ch DC servo driver T-Cube";

const char* g_PositionProp = "Position";
const char* g_Keyword_Position = "Set position (um)";
const char* g_Keyword_Velocity = "Velocity (mm/s)";
const char* g_Keyword_Home = "Go Home";

const char* g_NumberUnitsProp = "Number of Units";
const char* g_SerialNumberProp = "Serial Number";
const char* g_ChannelProp = "Channel";
const char* g_MaxVelProp = "Maximum Velocity";
const char* g_MaxAccnProp = "Maximum Acceleration";
const char* g_MinPosProp = "Position Lower Limit (um)";
const char* g_MaxPosProp = "Position Upper Limit (um)";
const char* g_StepSizeProp = "Step Size";

const char* g_TrigModeProp = "Trigger Mode";
const char* g_TrigMoveProp = "Trigger Move";
const char* g_TrigDirProp = "Trigger Direction";
const char* g_MoveRelProp = "Trigger Step Size (um)";
const char* g_TrigModes[] = { "Trigger In", "Trigger Out", "Trigger In/Out" };
const char* g_TrigMoves[] = { "Relative", "Absolute", "Home" };
const char* g_TrigDir[] = { "Foward", "Backward" };

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
	
	RegisterDevice(g_ThorlabsDeviceNameBSC201, MM::StageDevice, g_ThorlabsDeviceDescBSC201);

}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
	int chNumber = 1;

	if (deviceName == 0)
		return 0;

	ThorlabsKinesisBenchtop* s = new ThorlabsKinesisBenchtop(deviceName, chNumber);
	return s;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
	delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////
// ThorlabsKinesisBenchtop class
// FIXME? as default initialises to TDC001 / 1 channel
//
ThorlabsKinesisBenchtop::ThorlabsKinesisBenchtop() :
	hwType_(0),
	deviceName_("ThorlabsKinesisBenchtop"),
	chNumber_(1),
	stepSizeUm_(0.1),
	initialized_(false),
	busy_(false),
	homed_(false),
	answerTimeoutMs_(1000),
	minTravelUm_(0.0),
	maxTravelUm_(50000.0),
	curPosUm_(0.0),
	ccdT_(0.0),
	mode_(0),
	mode_m(0),
	mode_d(0),
	trigmode(0),
	trigmove(0),
	trigdir(false),

	trigModeNumber_(2),
	trigMoveNumber_(2),
	trigDirNumber_(1),
	moveRelStep_(0.5)

{
	//use the TDC001 default
	int hwType = 31;//HWTYPE_TDC001;
	std::string deviceName = g_ThorlabsDeviceNameTDC001;
	long chNumber = 1;

	init(deviceName, chNumber);
}

ThorlabsKinesisBenchtop::ThorlabsKinesisBenchtop(std::string deviceName, long chNumber) :
	hwType_(0),
	deviceName_("ThorlabsKinesisBenchtop"),
	chNumber_(1),
	stepSizeUm_(0.1),
	initialized_(false),
	busy_(false),
	homed_(false),
	answerTimeoutMs_(1000),
	minTravelUm_(0.0),
	maxTravelUm_(50000.0),
	curPosUm_(0.0),
	ccdT_(0.0),
	mode_(0),
	mode_m(0),
	mode_d(0),
	trigmode(0),
	trigmove(0),
	trigdir(false),

	trigModeNumber_(2),
	trigMoveNumber_(2),
	trigDirNumber_(1),
	moveRelStep_(0.5)
{
	init(deviceName, chNumber);
}

ThorlabsKinesisBenchtop::~ThorlabsKinesisBenchtop()
{
	Shutdown();
}

void ThorlabsKinesisBenchtop::GetName(char* Name) const
{
	CDeviceUtils::CopyLimitedString(Name, deviceName_.c_str());
}

int ThorlabsKinesisBenchtop::Initialize()
{
	int ret(DEVICE_OK);
	int hola = 0;

	//needed to get the hardware info
	GetLimits(minTravelUm_, maxTravelUm_);

	//change the global flag

	aptInitialized = true;
	LogInit();

	tmpMessage << "InitHWDevice()";
	LogIt();

	/////////

	//MOT_GetVelParamLimits(serialNumber_, &pfMaxAccn, &pfMaxVel);
	SBC_GetMotorVelocityLimits(serialNumber_.c_str(), chNumber_, &pfMaxVel, &pfMaxAccn);
	posUm_ = SBC_GetPosition(serialNumber_.c_str(), chNumber_);
	tmpMessage << "pfMaxAccn:" << pfMaxAccn << " pfMaxVel:" << pfMaxVel;
	LogIt();

	// READ ONLY PROPERTIES
	CreateProperty(g_SerialNumberProp, serialNumber_.c_str(), MM::String, true);
	CreateProperty(g_MaxVelProp, CDeviceUtils::ConvertToString(pfMaxVel), MM::String, true);
	CreateProperty(g_MaxAccnProp, CDeviceUtils::ConvertToString(pfMaxAccn), MM::String, true);

	//Action Properties
	CPropertyAction* pAct = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnPosition);
	CPropertyAction* pAct2 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnVelocity);
	CPropertyAction* pAct3 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnHome);
	CPropertyAction* pAct4 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnMinPosUm);
	CPropertyAction* pAct5 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnMaxPosUm);

	CPropertyAction* pAct6 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnTrigMode);
	CPropertyAction* pAct7 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnTrigMove);
	CPropertyAction* pAct8 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnTrigDir);
	CPropertyAction* pAct9 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnMoveRel);


	CreateFloatProperty(g_Keyword_Position, 0, false, pAct);
	SetPropertyLimits(g_Keyword_Position, minTravelUm_, maxTravelUm_);

	CreateProperty(g_Keyword_Velocity, CDeviceUtils::ConvertToString(pfMaxVel), MM::Float, false, pAct2);

	CreateProperty(g_Keyword_Home, "0", MM::Integer, false, pAct3);
	SetPropertyLimits(g_Keyword_Home, 0, 1);

	//By now, we now more about the hardware. Lets set proper hardware limits for g_MinPosProp / g_MaxPosProp
	//pfMinPos*1000 / pfMaxPos*1000 are the absolute limits.
	//FIXME may run into problems if not (plUnits == STAGE_UNITS_MM || plUnits == STAGE_UNITS_DEG)

	CreateProperty(g_MinPosProp, CDeviceUtils::ConvertToString(minTravelUm_), MM::Float, false, pAct4);
	SetPropertyLimits(g_MinPosProp, pfMinPos * 1000, pfMaxPos * 1000);

	CreateProperty(g_MaxPosProp, CDeviceUtils::ConvertToString(maxTravelUm_), MM::Float, false, pAct5);
	SetPropertyLimits(g_MaxPosProp, pfMinPos * 1000, pfMaxPos * 1000);

	// Trigger properties
	CreateProperty(g_TrigModeProp, g_TrigModes[0], MM::Integer, false, pAct6);
	for (int i = 0; i <= trigModeNumber_; i++)
	{
		AddAllowedValue(g_TrigModeProp, g_TrigModes[i]);
	}
	trigModeNumber_ = 2;

	CreateProperty(g_TrigMoveProp, g_TrigMoves[0], MM::Integer, false, pAct7);

	for (int i = 0; i <= trigMoveNumber_; i++)
	{
		AddAllowedValue(g_TrigMoveProp, g_TrigMoves[i]);
	}
	trigMoveNumber_ = 2;

	CreateProperty(g_TrigDirProp, g_TrigDir[0], MM::String, false, pAct8);

	for (int i = 0; i <= trigDirNumber_; i++)
	{
		AddAllowedValue(g_TrigDirProp, g_TrigDir[i]);
	}
	trigDirNumber_ = 1;

	CreateProperty(g_MoveRelProp, CDeviceUtils::ConvertToString(moveRelStep_), MM::Float, false, pAct9);
	SetPropertyLimits(g_MoveRelProp, 1, 1000);

	ret = UpdateStatus();

	tmpMessage << "all done";
	LogIt();

	if (ret != DEVICE_OK)
		return ret;

	initialized_ = true;
	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::Shutdown()
{

	if (initialized_)
	{
		initialized_ = false;
	}
	SBC_Close(serialNumber_.c_str());
	return DEVICE_OK;
}

bool ThorlabsKinesisBenchtop::Busy()
{
	int resp = SBC_RequestStatusBits(serialNumber_.c_str(), chNumber_);
	Sleep(200);
	DWORD status = SBC_GetStatusBits(serialNumber_.c_str(), chNumber_);
	unsigned short isMovingCW = (status & 0x00000010);
	unsigned short isMovingCCW = (status & 0x00000020);
	bool isBusy = (bool)(isMovingCW || isMovingCCW);

	return (bool)(isBusy);
}

int ThorlabsKinesisBenchtop::GetPositionUm(double& posUm)
{

	pfPosition = SBC_GetPosition(serialNumber_.c_str(), chNumber_);

	curPosUm_ = (double)pfPosition/409600*1000;
	posUm = curPosUm_;

	tmpMessage << "GetPositionUm:" << curPosUm_;
	LogIt();

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::SetPositionUm(double posUm)
{
	return SetPositionUmFlag(posUm, 1);
}

int ThorlabsKinesisBenchtop::SetPositionUmContinuous(double posUm)
{
	return SetPositionUmFlag(posUm, 1);
}

int ThorlabsKinesisBenchtop::SetOrigin()
{
	return DEVICE_UNSUPPORTED_COMMAND;
}

int ThorlabsKinesisBenchtop::SetPositionSteps(long steps)
{
	steps = (long)0.1;
	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::GetPositionSteps(long& steps)
{
	stepSizeUm_ = steps;
	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::GetLimits(double& min, double& max)
{

	tmpMessage << "In GetLimits(). chNumber:" << chNumber_ << " pfMinPos:" << pfMinPos << " pfMaxPos:" << pfMaxPos; //<< " plUnits:" << plUnits << " pfPitch:" << pfPitch;
	LogIt();
	

	min = minTravelUm_;
	max = maxTravelUm_;

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::SetLimits(double min, double max)
{
	minTravelUm_ = min;
	maxTravelUm_ = max;
	return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////

void ThorlabsKinesisBenchtop::init(std::string deviceName, long chNumber)
{
	long plNumUnits, plSerialNum;
	int hola;
	char serialNO[9];
	deviceName_ = deviceName;
	chNumber_ = chNumber;
	InitializeDefaultErrorMessages();

	// set device specific error messages
	// ------------------------------------
	SetErrorText(ERR_UNRECOGNIZED_ANSWER, "Invalid response from the device.");
	SetErrorText(ERR_UNSPECIFIED_ERROR, "Unspecified error occured.");
	SetErrorText(ERR_RESPONSE_TIMEOUT, "Device timed-out: no response received withing expected time interval.");
	SetErrorText(ERR_BUSY, "Device busy.");
	SetErrorText(ERR_STAGE_NOT_ZEROED, "Zero sequence still in progress.\n"
		"Wait for few more seconds before trying again."
		"Zero sequence executes only once per power cycle.");

	// create pre-initialization properties
	// ------------------------------------
	
	// Name
	CreateProperty(MM::g_Keyword_Name, deviceName.c_str(), MM::String, true);

	// Description (will be properly updated later from MOT_GetStageAxisInfo)
	CreateProperty(MM::g_Keyword_Description, "Thorlabs Stage", MM::String, true);

	CPropertyAction* pAct = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnPosition);
	CreateFloatProperty(g_Keyword_Position, 0, false, pAct);
	SetPropertyLimits(g_Keyword_Position, minTravelUm_, maxTravelUm_);

	// Serial Number
	CPropertyAction* pAct1 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnSerialNumber);
	CreateProperty(g_SerialNumberProp, "", MM::Integer, false, pAct1, true);

	// Add this device to the drop-down menu, creating the property if this is the first one.
	short n = TLI_GetDeviceListSize();
	if (TLI_BuildDeviceList() == 0)
	{
		short n = TLI_GetDeviceListSize();

		char serialNos[100];
		TLI_GetDeviceListByTypeExt(serialNos, 100, 40); // we are only capturing Benchtop Stepper devices
		char *next_token1 = NULL;
		char *p = strtok_s(serialNos, ",", &next_token1);
		while (p != NULL)
		{
			// Get and parse this device's serial number and description
			TLI_DeviceInfo deviceInfo;
			TLI_GetDeviceInfo(p, &deviceInfo);
			char desc[65];
			strncpy_s(desc, deviceInfo.description, 64);
			desc[64] = '\0';

			// serialNo has type c-string.
			//char serialNo[9];
			strncpy_s(serialNO, deviceInfo.serialNo, 8);
			serialNO[8] = '\0';
			if (SBC_Open(serialNO) == 0)
			{
				// start the device polling at 200ms intervals
				SBC_StartPolling(serialNO, chNumber, 200);
				SBC_LoadSettings(serialNO, chNumber);
			}
			int i = 0;

			if (i == 0)
			{

				CPropertyAction* pAct1 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnSerialNumber);
				CreateProperty(g_SerialNumberProp, serialNO, MM::String, false, pAct1, true);
			}
			AddAllowedValue(g_SerialNumberProp, serialNO);
			i++;
			p = strtok_s(NULL, ",", &next_token1);

		}
	}

	//Populating the Channel drop-down menu (last one selected by default?)
	CPropertyAction* pAct2 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnChannelNumber);
	CreateProperty(g_ChannelProp, "1", MM::Integer, false, pAct2, true);

	for (int i = 0; i < chNumber_; i++)
	{
		AddAllowedValue(g_ChannelProp, CDeviceUtils::ConvertToString(i + 1));
	}
	chNumber_ = 1;

	//Populating the min and max values for min travel

	//At this point in time, we do not know the real min and max travel.
	//These depend on the serial number and channel and type of stage attached.
	//Widely, 0-500000 um should cover the rotation stages as well (0-360*1000?)
	//GetLimits(minTravelUm_, maxTravelUm_);
	pfMinPos = (float)(minTravelUm_);
	pfMaxPos = (float)(maxTravelUm_);

	CPropertyAction* pAct3 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnMinPosUm);
	CreateProperty(g_MinPosProp, CDeviceUtils::ConvertToString(minTravelUm_), MM::Float, false, pAct3, true);
	CreateProperty(g_MinPosProp, CDeviceUtils::ConvertToString(minTravelUm_), MM::Float, false, pAct3);
	//SetPropertyLimits(g_MinPosProp, minTravelUm_, maxTravelUm_);

	CPropertyAction* pAct4 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnMaxPosUm);
	CreateProperty(g_MaxPosProp, CDeviceUtils::ConvertToString(maxTravelUm_), MM::Float, false, pAct4, true);
	CreateProperty(g_MaxPosProp, CDeviceUtils::ConvertToString(maxTravelUm_), MM::Float, false, pAct4);
	//SetPropertyLimits(g_MaxPosProp, minTravelUm_, maxTravelUm_);

	//Populating the Trigger Mode drop-down menu
	CPropertyAction* pAct5 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnTrigMode);
	CreateProperty(g_TrigModeProp, g_TrigModes[0], MM::String, false, pAct5);

	for (int i = 0; i <= trigModeNumber_; i++)
	{
		AddAllowedValue(g_TrigModeProp, g_TrigModes[i]);
	}
	trigModeNumber_ = 2;

	//Populating the Trigger Move drop-down menu
	CPropertyAction* pAct6 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnTrigMove);
	CreateProperty(g_TrigMoveProp, g_TrigMoves[0], MM::String, false, pAct6);

	for (int i = 0; i <= trigMoveNumber_; i++)
	{
		AddAllowedValue(g_TrigMoveProp, g_TrigMoves[i]);
	}
	trigMoveNumber_ = 2;

	//Populating the Move relative Step Size menu
	CPropertyAction* pAct7 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnMoveRel);
	CreateProperty(g_MoveRelProp, CDeviceUtils::ConvertToString(moveRelStep_), MM::Float, false, pAct7);
	SetPropertyLimits(g_MoveRelProp ,1 , 1000);

	//Populating the Trigger Direction drop-down menu
	CPropertyAction* pAct8 = new CPropertyAction(this, &ThorlabsKinesisBenchtop::OnTrigDir);
	CreateProperty(g_TrigDirProp, g_TrigDir[0], MM::String, false, pAct8);

	for (int i = 0; i <= trigDirNumber_; i++)
	{
		AddAllowedValue(g_TrigDirProp, g_TrigDir[i]);
	}
	trigDirNumber_ = 1;
}

void ThorlabsKinesisBenchtop::LogInit()
{
	tmpMessage = std::stringstream();
	tmpMessage << deviceName_ << "-" << serialNumber_ << " ";
}

void ThorlabsKinesisBenchtop::LogIt()
{
	LogMessage(tmpMessage.str().c_str());
	LogInit();
}

int ThorlabsKinesisBenchtop::SetPositionUmFlag(double posUm, int continuousFlag)
{
	if (posUm < minTravelUm_)
		posUm = minTravelUm_;
	else if (posUm > maxTravelUm_)
		posUm = maxTravelUm_;
	curPosUm_ = posUm;
	newPosition = (float)curPosUm_/1000*409600;
	SBC_MoveToPosition(serialNumber_.c_str(), chNumber_, newPosition);
	OnStagePositionChanged(curPosUm_);

	tmpMessage << "SetPositionUm:" << posUm << " continuousFlag:" << continuousFlag;
	LogIt();

	return DEVICE_OK;
}


/**
* Send Home command to the stage
* If stage was already Homed, this command has no effect.
* If not, then the zero sequence will be initiated.
*/

int ThorlabsKinesisBenchtop::Home()
{
	int ret = DEVICE_OK;
	SBC_Home(serialNumber_.c_str(), chNumber_);

	if (homed_)
		return ret;

	if (SBC_CanHome(serialNumber_.c_str(), chNumber_))
	{
		homed_ = false;
	}
	else
	{
		homed_ = true;
	}

	return ret;
}

int ThorlabsKinesisBenchtop::GetVelParam(double& vel)
{
	SBC_GetMotorVelocityLimits(serialNumber_.c_str(), chNumber_, &pfMaxVel, &pfMaxAccn);

	vel = (double)pfMaxVel;

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::SetVelParam(double vel)
{
	SBC_SetVelParams(serialNumber_.c_str(), chNumber_, pfAccn, pfMaxVel);

	return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////

int ThorlabsKinesisBenchtop::OnSerialNumber(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		pProp->Set(serialNumber_.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		if (initialized_)
		{
			long serialNumber;
			pProp->Get(serialNumber);
			serialNumber_ = serialNumber;

		}
		string serialNumber;
		pProp->Get(serialNumber);
		serialNumber_ = serialNumber;
	}

	if (aptInitialized == true)
		//InitHWDevice(serialNumber_);

	tmpMessage << "Serial number set to " << serialNumber_;
	LogIt();

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnChannelNumber(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		pProp->Set(chNumber_);
	}
	else if (eAct == MM::AfterSet)
	{
		if (initialized_)
		{
			long chNumber;
			pProp->Get(chNumber);
			chNumber_ = chNumber;

		}

		pProp->Get(chNumber_);
	}

	tmpMessage << "Channel number set to " << chNumber_;
	LogIt();

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnMinPosUm(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		if (minTravelUm_ > pfMaxPos * 1000)
			minTravelUm_ = pfMaxPos * 1000;
		else if (minTravelUm_ < pfMinPos * 1000)
			minTravelUm_ = pfMinPos * 1000;
		pProp->Set(minTravelUm_);
	}
	else if (eAct == MM::AfterSet)
	{
		if (initialized_)
		{
			double minTravelUm;
			pProp->Get(minTravelUm);
			minTravelUm_ = minTravelUm;
		}

		pProp->Get(minTravelUm_);
		if (minTravelUm_ > pfMaxPos * 1000)
			minTravelUm_ = pfMaxPos * 1000;
		else if (minTravelUm_ < pfMinPos * 1000)
			minTravelUm_ = pfMinPos * 1000;
	}

	tmpMessage << "minTravelUm_ set to " << minTravelUm_ << " pfMinPos:" << pfMinPos << " pfMaxPos:" << pfMaxPos;
	LogIt();

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnMaxPosUm(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		if (maxTravelUm_ > pfMaxPos * 1000)
			maxTravelUm_ = pfMaxPos * 1000;
		else if (maxTravelUm_ < pfMinPos * 1000)
			maxTravelUm_ = pfMinPos * 1000;

		pProp->Set(maxTravelUm_);
	}
	else if (eAct == MM::AfterSet)
	{
		if (initialized_)
		{
			double maxTravelUm;
			pProp->Get(maxTravelUm);
			maxTravelUm_ = maxTravelUm;
		}

		pProp->Get(maxTravelUm_);

		if (maxTravelUm_ > pfMaxPos * 1000)
			maxTravelUm_ = pfMaxPos * 1000;
		else if (maxTravelUm_ < pfMinPos * 1000)
			maxTravelUm_ = pfMinPos * 1000;
	}

	tmpMessage << "maxTravelUm_ set to " << maxTravelUm_ << " pfMinPos:" << pfMinPos << " pfMaxPos:" << pfMaxPos;
	LogIt();

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		double pos = 0.0;
		int ret = GetPositionUm(pos);
		if (ret != DEVICE_OK)
			return ret;

		pProp->Set(pos);
	}
	else if (eAct == MM::AfterSet)
	{
		double pos = 0.0;
		pProp->Get(pos);
		int ret = SetPositionUm(pos);
		if (ret != DEVICE_OK)
			return ret;
	}

	return DEVICE_OK;
}


int ThorlabsKinesisBenchtop::OnVelocity(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		double vel;

		int ret = GetVelParam(vel);
		if (ret != DEVICE_OK)
			return ret;

		pProp->Set(vel);
	}
	else if (eAct == MM::AfterSet)
	{
		double vel;
		pProp->Get(vel);
		int ret = SetVelParam(vel);
		if (ret != DEVICE_OK)
			return ret;
	}

	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnHome(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		pProp->Set((long)homed_);
	}
	else if (eAct == MM::AfterSet)
	{
		long homed;
		pProp->Get(homed);

		int ret = Home();
		if (ret != DEVICE_OK)
			return ret;
	}
	return DEVICE_OK;
}

/*int ThorlabsKinesisBenchtop::OnStagePositionChanged(long totalSteps)
{
	ostringstream posStr;
	posStr << "Stage Position with serial number " << serialNumber_ << " changed to in steps " << totalSteps;
	this->LogMessage(posStr.str(), true);

	// int newPosition = floor(float(posUm)/1000); // in mm
	SBC_SetMoveAbsolutePosition(serialNo, chNumber_, totalSteps);
	SBC_MoveAbsolute(serialNo, chNumber_);
	return DEVICE_OK;
}*/
//////////////////////////////////////////////////trigger mode

int ThorlabsKinesisBenchtop::OnTrigMode(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	std::string val;
	if (eAct == MM::BeforeGet)
	{
		switch (mode_)
		{
		case 0:
			val = g_TrigModes[0];
			break;
		case 1:
			val = g_TrigModes[1];
			break;
		case 2:
			val = g_TrigModes[2];
			break;
		default:
			val = g_TrigModes[0];
			break;
		}
		pProp->Set(val.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(val);
		if (val == g_TrigModes[0])
		{
			mode_ = 0;
			trigmode = 1;
		}
		else if (val == g_TrigModes[1])
		{
			mode_ = 1;
			trigmode = 2;
		}
		else
		{
			mode_ = 2;
			trigmode = 4;
		}
	}
	SBC_SetTriggerSwitches(serialNumber_.c_str(), chNumber_, trigmode+trigmove);
	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnTrigMove(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	std::string val;
	if (eAct == MM::BeforeGet)
	{
		switch (mode_m)
		{
		case 0:
			val = g_TrigMoves[0];
			break;
		case 1:
			val = g_TrigMoves[1];
			break;
		case 2:
			val = g_TrigMoves[2];
			break;
		default:
			val = g_TrigMoves[0];
			break;
		}
		pProp->Set(val.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(val);
		if (val == g_TrigMoves[0])
		{
			mode_m = 0;
			trigmove = 16;
		}
		else if (val == g_TrigMoves[1])
		{
			mode_m = 1;
			trigmove = 32;
		}
		else
		{
			mode_m = 2;
			trigmove = 64;
		}
	}
	SBC_SetTriggerSwitches(serialNumber_.c_str(), chNumber_, trigmode + trigmove);
	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnTrigDir(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	std::string val;
	if (eAct == MM::BeforeGet)
	{
		switch (mode_d)
		{
		case 0:
			val = g_TrigDir[0];
			break;
		case 1:
			val = g_TrigDir[1];
			break;
		default:
			val = g_TrigDir[0];
			break;
		}
		pProp->Set(val.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(val);
		if (val == g_TrigDir[0])
		{
			mode_d = 0;
			trigdir = false;
		}
		else if (val == g_TrigDir[1])
		{
			mode_d = 1;
			trigdir = true;
		}
	}
	SBC_SetDirection(serialNumber_.c_str(), chNumber_, trigdir);
	return DEVICE_OK;
}

int ThorlabsKinesisBenchtop::OnMoveRel(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet)
	{
		pProp->Set(moveRelStep_);
	}
	else if (eAct == MM::AfterSet)
	{
		if (initialized_)
		{
			long moveRelStep;
			pProp->Get(moveRelStep);
			moveRelStep_ = (int)moveRelStep;

		}

		//pProp->Get(moveRelStep_);
	}

	tmpMessage << "Relative Step Size set to " << moveRelStep_;
	LogIt();
	SBC_SetMoveRelativeDistance(serialNumber_.c_str(), chNumber_, moveRelStep_*409600/1000);
	return DEVICE_OK;
}

// vim: set autoindent tabstop=4 softtabstop=4 shiftwidth=4 expandtab textwidth=78:

