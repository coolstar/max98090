#if !defined(_MAXM_H_)
#define _MAXM_H_

#pragma warning(disable:4200)  // suppress nameless struct/union warning
#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <initguid.h>
#include <wdm.h>

#pragma warning(default:4200)
#pragma warning(default:4201)
#pragma warning(default:4214)
#include <wdf.h>

#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <hidport.h>

#include "max98090.h"
#include "spb.h"
#include "gpiowrapper.h"

//
// String definitions
//

#define DRIVERNAME                 "max98090.sys: "

#define MAXM_POOL_TAG            (ULONG) 'mxaM'
#define MAXM_HARDWARE_IDS        L"CoolStar\\MAXM9890\0\0"
#define MAXM_HARDWARE_IDS_LENGTH sizeof(MAXM_HARDWARE_IDS)

#define NTDEVICE_NAME_STRING       L"\\Device\\MAXM9890"
#define SYMBOLIC_NAME_STRING       L"\\DosDevices\\MAXM9890"

#define true 1
#define false 0

typedef struct _MAXM_CONTEXT
{

	//
	// Handle back to the WDFDEVICE
	//

	WDFDEVICE FxDevice;

	WDFQUEUE ReportQueue;

	SPB_CONTEXT I2CContext;

	GPIO_CONTEXT GpioContext;

	GPIO_CONTEXT MicGpioContext;

	WDFTIMER Timer;

	BOOLEAN ConnectInterrupt;

	BOOLEAN HeadphonesConnected;

	BOOLEAN HeadsetMicConnected;

} MAXM_CONTEXT, *PMAXM_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(MAXM_CONTEXT, GetDeviceContext)

//
// Function definitions
//

DRIVER_INITIALIZE DriverEntry;

EVT_WDF_DRIVER_UNLOAD MaxmDriverUnload;

EVT_WDF_DRIVER_DEVICE_ADD MaxmEvtDeviceAdd;

EVT_WDFDEVICE_WDM_IRP_PREPROCESS MaxmEvtWdmPreprocessMnQueryId;

EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL MaxmEvtInternalDeviceControl;

//
// Helper macros
//

#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_INFO    2
#define DEBUG_LEVEL_VERBOSE 3

#define DBG_INIT  1
#define DBG_PNP   2
#define DBG_IOCTL 4

#if 0
#define MaxmPrint(dbglevel, dbgcatagory, fmt, ...) {          \
    if (MaxmDebugLevel >= dbglevel &&                         \
        (MaxmDebugCatagories && dbgcatagory))                 \
		    {                                                           \
        DbgPrint(DRIVERNAME);                                   \
        DbgPrint(fmt, __VA_ARGS__);                             \
		    }                                                           \
}
#else
#define MaxmPrint(dbglevel, fmt, ...) {                       \
}
#endif
#endif