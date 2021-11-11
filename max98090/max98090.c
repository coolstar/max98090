#define DESCRIPTOR_DEF
#include "driver.h"
#include "stdint.h"

#define bool int
#define MHz 1000000

static ULONG MaxmDebugLevel = 100;
static ULONG MaxmDebugCatagories = DBG_INIT || DBG_PNP || DBG_IOCTL;

NTSTATUS
DriverEntry(
__in PDRIVER_OBJECT  DriverObject,
__in PUNICODE_STRING RegistryPath
)
{
	NTSTATUS               status = STATUS_SUCCESS;
	WDF_DRIVER_CONFIG      config;
	WDF_OBJECT_ATTRIBUTES  attributes;

	MaxmPrint(DEBUG_LEVEL_INFO, DBG_INIT,
		"Driver Entry\n");

	WDF_DRIVER_CONFIG_INIT(&config, MaxmEvtDeviceAdd);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);

	//
	// Create a framework driver object to represent our driver.
	//

	status = WdfDriverCreate(DriverObject,
		RegistryPath,
		&attributes,
		&config,
		WDF_NO_HANDLE
		);

	if (!NT_SUCCESS(status))
	{
		MaxmPrint(DEBUG_LEVEL_ERROR, DBG_INIT,
			"WdfDriverCreate failed with status 0x%x\n", status);
	}

	return status;
}

static int max98090_i2c_write(PMAXM_CONTEXT pDevice, uint8_t reg, uint8_t data)
{
	uint8_t rawdata[1];
	rawdata[0] = data;
	SpbWriteDataSynchronously(&pDevice->I2CContext, reg, rawdata, 1);
	return 0;
}

static int max98090_i2c_read(PMAXM_CONTEXT pDevice, uint8_t reg, uint8_t *data)
{
	uint8_t rawdata[256];
	SpbReadDataSynchronously(&pDevice->I2CContext, 0, rawdata, 256);
	*data = rawdata[reg];
	return 0;
}

static int max98090_update_bits(PMAXM_CONTEXT pDevice, uint8_t reg,
	uint8_t mask, uint8_t value)
{
	uint8_t old;
	if (max98090_i2c_read(pDevice, reg, &old))
		return 1;
	uint8_t new_value = (old & ~mask) | (value & mask);
	if (old != new_value && max98090_i2c_write(pDevice, reg, new_value))
		return 1;

	return 0;
}

// Resets the audio codec.
static int max98090_reset(PMAXM_CONTEXT codec)
{
	// Gracefully reset the DSP core and the codec hardware in a proper
	// sequence.
	if (max98090_i2c_write(codec, M98090_REG_SOFTWARE_RESET,
		M98090_SWRESET_MASK))
		return 1;
	return 0;
}

static int max98090_hw_params(PMAXM_CONTEXT codec)
{
	unsigned char value;
	unsigned int sample_rate = 48000;
	unsigned int bits_per_sample = 24;

	switch (bits_per_sample) {
	case 16:
		max98090_i2c_read(codec, M98090_REG_INTERFACE_FORMAT, &value);
		if (max98090_update_bits(codec, M98090_REG_INTERFACE_FORMAT,
			M98090_WS_MASK, 0))
			return 1;
		max98090_i2c_read(codec, M98090_REG_INTERFACE_FORMAT, &value);
		break;
	case 24:
		break;
	default:
		MaxmPrint(DEBUG_LEVEL_ERROR,
			DBG_IOCTL, "Illegal bits per sample %d.\n",
			bits_per_sample);
		break;
	}

	max98090_i2c_write(codec, M98090_REG_FILTER_CONFIG, M98090_MODE_MASK | M98090_AHPF_MASK | M98090_DHPF_MASK);
	return 0;
}

// Configures audio interface system clock for the selected frequency.
static int max98090_set_sysclk(PMAXM_CONTEXT codec)
{
	int freq = 512 /*256*/ /*lr_frame_size*/ * 48000 /* sample rate */;
	// TODO(hungte) Should we handle codec->master_clock?
	uint8_t mclksel = 0;

	/*
	* Setup clocks for slave mode, and using the PLL
	* PSCLK = 0x01 (when master clk is 10MHz to 20MHz)
	*	0x02 (when master clk is 20MHz to 40MHz)..
	*	0x03 (when master clk is 40MHz to 60MHz)..
	*/
	if ((freq >= 10 * MHz) && (freq < 20 * MHz)) {
		mclksel |= M98090_PSCLK_DIV1;
	}
	else if ((freq >= 20 * MHz) && (freq < 40 * MHz)) {
		mclksel |= M98090_PSCLK_DIV2;
	}
	else if ((freq >= 40 * MHz) && (freq < 60 * MHz)) {
		mclksel |= M98090_PSCLK_DIV4;
	}
	else {
		MaxmPrint(DEBUG_LEVEL_ERROR,
			DBG_IOCTL, "Invalid master clock frequency\n");
		return 1;
	}

	if (max98090_i2c_write(codec, M98090_REG_SYSTEM_CLOCK, mclksel))
		return 1;

	MaxmPrint(DEBUG_LEVEL_ERROR,
		DBG_IOCTL, "Clock at %uHz\n", freq);
	return 0;
}

// Sets Max98090 I2S format.
static int max98090_set_fmt(PMAXM_CONTEXT codec)
{
	// Set to slave mode PLL - MAS mode off.
	if (max98090_i2c_write(codec, M98090_REG_CLOCK_RATIO_NI_MSB, 0x00) ||
		max98090_i2c_write(codec, M98090_REG_CLOCK_RATIO_NI_LSB, 0x00))
		return 1;

	if (max98090_update_bits(codec, M98090_REG_CLOCK_MODE,
		M98090_USE_M1_MASK, 0))
		return 1;

	if (max98090_i2c_write(codec, M98090_REG_MASTER_MODE, 0))
		return 1;

	// Format: I2S, IB_IF.
	if (max98090_i2c_write(codec, M98090_REG_INTERFACE_FORMAT,
		M98090_DLY_MASK))
		return 1;

	return 0;
}

int max98090_device_init(PMAXM_CONTEXT codec)
{
	// Reset the codec, the DSP core, and disable all interrupts.
	

	uint8_t id;
	if (max98090_i2c_read(codec, M98090_REG_REVISION_ID, &id))
		return 1;
	DbgPrint("Maxim Hardware revision: 0x%x\n", id);

	max98090_i2c_write(codec, M98090_REG_DEVICE_SHUTDOWN,
		0x00); //Shutdown the codec to allow setup

	/* Reading interrupt status to clear them */
	int res = 0;
	res = max98090_i2c_read(codec, M98090_REG_DEVICE_STATUS, &id);

	res |= max98090_i2c_write(codec, M98090_REG_DAC_CONTROL,
		M98090_DACHP_MASK | M98090_PERFMODE_MASK);
	res |= max98090_i2c_write(codec, M98090_REG_ADC_CONTROL, M98090_ADCHP_MASK | M98090_ADCDITHER_MASK);
	res |= max98090_i2c_write(codec, M98090_REG_BIAS_CONTROL,
		M98090_VCM_MODE_MASK);

	res |= max98090_i2c_write(codec, M98090_REG_CLOCK_RATIO_NI_MSB, 0x0);
	res |= max98090_i2c_write(codec, M98090_REG_CLOCK_RATIO_NI_LSB, 0x0);
	res |= max98090_i2c_write(codec, M98090_REG_MASTER_MODE, 0x0);
	res |= max98090_i2c_write(codec, M98090_REG_INTERFACE_FORMAT, 0x0);
	res |= max98090_i2c_write(codec, M98090_REG_IO_CONFIGURATION,
		M98090_SDIEN_MASK);
	if (codec->HeadphonesConnected) {
		res |= max98090_i2c_write(codec, M98090_REG_OUTPUT_ENABLE,
			M98090_HPREN_MASK | M98090_HPLEN_MASK |
			M98090_DAREN_MASK | M98090_DALEN_MASK);
	}
	else {
		res |= max98090_i2c_write(codec, M98090_REG_OUTPUT_ENABLE,
			M98090_SPREN_MASK | M98090_SPLEN_MASK |
			M98090_DAREN_MASK | M98090_DALEN_MASK);
	}

	//IO Config

	res |= max98090_i2c_write(codec, M98090_REG_IO_CONFIGURATION,
		M98090_SDOEN_MASK | M98090_SDIEN_MASK);
}

int max98090_set_output(PMAXM_CONTEXT codec) {
	int res = 0;
	//Headphone Mixer Configuration
	res |= max98090_i2c_write(codec, M98090_REG_LEFT_HP_VOLUME, 0x1F);
	res |= max98090_i2c_write(codec, M98090_REG_RIGHT_HP_VOLUME, 0x1F);

	res |= max98090_i2c_write(codec, M98090_REG_LEFT_HP_MIXER, M98090_MIXHPL_DACL_MASK);
	res |= max98090_i2c_write(codec, M98090_REG_RIGHT_HP_MIXER, M98090_MIXHPR_DACR_MASK);

	res |= max98090_i2c_write(codec, M98090_REG_HP_CONTROL, 0x30);

	//Speaker Mixer Configuration
	res |= max98090_i2c_write(codec, M98090_REG_LEFT_SPK_VOLUME, 0x3F);
	res |= max98090_i2c_write(codec, M98090_REG_RIGHT_SPK_VOLUME, 0x3F);

	res |= max98090_i2c_write(codec, M98090_REG_LEFT_SPK_MIXER, M98090_MIXSPL_DACL_MASK);
	res |= max98090_i2c_write(codec, M98090_REG_RIGHT_SPK_MIXER, M98090_MIXSPR_DACR_MASK);

	res |= max98090_i2c_write(codec, M98090_REG_SPK_CONTROL, 0x0f); //gain config -12 db


	//Input config

	MaxmPrint(DEBUG_LEVEL_ERROR,
		DBG_IOCTL, "Input Configuration");

	//Internal Mic

	res |= max98090_i2c_write(codec, M98090_REG_MIC1_INPUT_LEVEL, 0x14);

	res |= max98090_update_bits(codec, M98090_REG_MIC_BIAS_VOLTAGE, M98090_MBVSEL_MASK, M98090_MBVSEL_2V8);

	res |= max98090_i2c_write(codec, M98090_REG_DIGITAL_MIC_CONFIG, 0x60);

	res |= max98090_i2c_write(codec, M98090_REG_LEFT_ADC_MIXER, M98090_MIXADL_MIC2_MASK);
	res |= max98090_i2c_write(codec, M98090_REG_RIGHT_ADC_MIXER, M98090_MIXADR_MIC2_MASK);

	res |= max98090_i2c_write(codec, M98090_REG_LEFT_ADC_LEVEL, (0x4 << M98090_AVLG_SHIFT) | 0x4);
	res |= max98090_i2c_write(codec, M98090_REG_RIGHT_ADC_LEVEL, (0x4 << M98090_AVLG_SHIFT) | 0x4);

	if (codec->HeadsetMicConnected) {
		max98090_i2c_write(codec, M98090_REG_MIC2_INPUT_LEVEL, (1 << M98090_MIC_PA2EN_SHIFT) | 0x00); //Set PA2EN to 1 for Headset, 0 for internal mic
		max98090_i2c_write(codec, M98090_REG_DIGITAL_MIC_ENABLE, (3 << M98090_MICCLK_SHIFT) | 0);
		max98090_i2c_write(codec, M98090_REG_INPUT_ENABLE, M98090_ADLEN_MASK | M98090_ADREN_MASK | M98090_MBEN_MASK);
	}
	else {
		max98090_i2c_write(codec, M98090_REG_MIC2_INPUT_LEVEL, (0 << M98090_MIC_PA2EN_SHIFT) | 0x00); //Set PA2EN to 1 for Headset, 0 for internal mic
		max98090_i2c_write(codec, M98090_REG_DIGITAL_MIC_ENABLE, (3 << M98090_MICCLK_SHIFT) | M98090_DIGMICR_MASK | M98090_DIGMICL_MASK);
		max98090_i2c_write(codec, M98090_REG_INPUT_ENABLE, 0x0); //Set to M98090_ADLEN_MASK | M98090_ADREN_MASK | M98090_MBEN_MASK for headset, 0 for internal mic
	}

	return res;
}

void max98090_jack_detect(PMAXM_CONTEXT codec) {
	max98090_i2c_write(codec, M98090_REG_DEVICE_SHUTDOWN,
		M98090_SHDNN_MASK);

	max98090_i2c_write(codec, M98090_REG_JACK_DETECT,
		M98090_JDETEN_MASK | M98090_JDEB_25MS | M98090_JDWK_MASK);
}

VOID
MaxmBootWorkItem(
	IN WDFWORKITEM  WorkItem
)
{
	WDFDEVICE Device = (WDFDEVICE)WdfWorkItemGetParentObject(WorkItem);
	PMAXM_CONTEXT pDevice = GetDeviceContext(Device);

	max98090_device_init(pDevice);
	max98090_set_sysclk(pDevice);
	max98090_hw_params(pDevice);
	max98090_set_fmt(pDevice);
	max98090_set_output(pDevice);
	max98090_jack_detect(pDevice);

	pDevice->ConnectInterrupt = true;

	WdfObjectDelete(WorkItem);
}

void MaxmBootTimer(_In_ WDFTIMER hTimer) {
	WDFDEVICE Device = (WDFDEVICE)WdfTimerGetParentObject(hTimer);
	PMAXM_CONTEXT pDevice = GetDeviceContext(Device);

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_WORKITEM_CONFIG workitemConfig;
	WDFWORKITEM hWorkItem;

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(&attributes, MAXM_CONTEXT);
	attributes.ParentObject = Device;
	WDF_WORKITEM_CONFIG_INIT(&workitemConfig, MaxmBootWorkItem);

	WdfWorkItemCreate(&workitemConfig,
		&attributes,
		&hWorkItem);

	WdfWorkItemEnqueue(hWorkItem);
	WdfTimerStop(hTimer, FALSE);
}

NTSTATUS BOOTCODEC(
	_In_  PMAXM_CONTEXT  pDevice
	)
{
	NTSTATUS status = 0;

	pDevice->ConnectInterrupt = false;

	pDevice->HeadphonesConnected = false;

	if (max98090_reset(pDevice))
		return 1;

	WDF_TIMER_CONFIG              timerConfig;
	WDFTIMER                      hTimer;
	WDF_OBJECT_ATTRIBUTES         attributes;

	WDF_TIMER_CONFIG_INIT(&timerConfig, MaxmBootTimer);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	attributes.ParentObject = pDevice->FxDevice;
	status = WdfTimerCreate(&timerConfig, &attributes, &hTimer);

	WdfTimerStart(hTimer, WDF_REL_TIMEOUT_IN_MS(20));

	return status;
}

NTSTATUS
OnPrepareHardware(
_In_  WDFDEVICE     FxDevice,
_In_  WDFCMRESLIST  FxResourcesRaw,
_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

This routine caches the SPB resource connection ID.

Arguments:

FxDevice - a handle to the framework device object
FxResourcesRaw - list of translated hardware resources that
the PnP manager has assigned to the device
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PMAXM_CONTEXT pDevice = GetDeviceContext(FxDevice);
	BOOLEAN fSpbResourceFound = FALSE;
	BOOLEAN fJackDetectResourceFound = FALSE;
	BOOLEAN fMicDetectResourceFound = FALSE;
	NTSTATUS status = STATUS_INSUFFICIENT_RESOURCES;

	UNREFERENCED_PARAMETER(FxResourcesRaw);

	//
	// Parse the peripheral's resources.
	//

	ULONG resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);

	for (ULONG i = 0; i < resourceCount; i++)
	{
		PCM_PARTIAL_RESOURCE_DESCRIPTOR pDescriptor;
		UCHAR Class;
		UCHAR Type;

		pDescriptor = WdfCmResourceListGetDescriptor(
			FxResourcesTranslated, i);

		switch (pDescriptor->Type)
		{
		case CmResourceTypeConnection:
			//
			// Look for I2C or SPI resource and save connection ID.
			//
			Class = pDescriptor->u.Connection.Class;
			Type = pDescriptor->u.Connection.Type;
			if (Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL &&
				Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)
			{
				if (fSpbResourceFound == FALSE)
				{
					status = STATUS_SUCCESS;
					pDevice->I2CContext.I2cResHubId.LowPart = pDescriptor->u.Connection.IdLowPart;
					pDevice->I2CContext.I2cResHubId.HighPart = pDescriptor->u.Connection.IdHighPart;
					fSpbResourceFound = TRUE;
				}
				else
				{
				}
			}
			if (Class == CM_RESOURCE_CONNECTION_CLASS_GPIO && Type == CM_RESOURCE_CONNECTION_TYPE_GPIO_IO) {
				if (fJackDetectResourceFound == FALSE) {
					pDevice->GpioContext.GpioResHubId.LowPart = pDescriptor->u.Connection.IdLowPart;
					pDevice->GpioContext.GpioResHubId.HighPart = pDescriptor->u.Connection.IdHighPart;
					fJackDetectResourceFound = TRUE;
				}
				else if (fMicDetectResourceFound == FALSE) {
					pDevice->MicGpioContext.GpioResHubId.LowPart = pDescriptor->u.Connection.IdLowPart;
					pDevice->MicGpioContext.GpioResHubId.HighPart = pDescriptor->u.Connection.IdHighPart;
					fMicDetectResourceFound = TRUE;
				}
			}
			break;
		default:
			//
			// Ignoring all other resource types.
			//
			break;
		}
	}

	//
	// An SPB resource is required.
	//

	if (fSpbResourceFound == FALSE || fJackDetectResourceFound == FALSE || fMicDetectResourceFound == FALSE)
	{
		status = STATUS_NOT_FOUND;
		return status;
	}

	status = GpioTargetInitialize(FxDevice, &pDevice->GpioContext);
	if (!NT_SUCCESS(status))
	{
		return status;
	}

	status = GpioTargetInitialize(FxDevice, &pDevice->MicGpioContext);
	if (!NT_SUCCESS(status))
	{
		return status;
	}

	status = SpbTargetInitialize(FxDevice, &pDevice->I2CContext);
	if (!NT_SUCCESS(status))
	{
		return status;
	}

	return status;
}

NTSTATUS
OnReleaseHardware(
_In_  WDFDEVICE     FxDevice,
_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

Arguments:

FxDevice - a handle to the framework device object
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PMAXM_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	UNREFERENCED_PARAMETER(FxResourcesTranslated);

	/*IO_DISCONNECT_INTERRUPT_PARAMETERS params;

	params.Version = CONNECT_FULLY_SPECIFIED;
	params.ConnectionContext.InterruptObject = pDevice->Interrupt;

	IoDisconnectInterruptEx(&params);*/

	GpioTargetDeinitialize(FxDevice, &pDevice->GpioContext);
	GpioTargetDeinitialize(FxDevice, &pDevice->MicGpioContext);
	SpbTargetDeinitialize(FxDevice, &pDevice->I2CContext);

	return status;
}

NTSTATUS
OnD0Entry(
_In_  WDFDEVICE               FxDevice,
_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine allocates objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PMAXM_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	WdfTimerStart(pDevice->Timer, WDF_REL_TIMEOUT_IN_MS(10));

	BOOTCODEC(pDevice);

	return status;
}

NTSTATUS
OnD0Exit(
_In_  WDFDEVICE               FxDevice,
_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine destroys objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PMAXM_CONTEXT pDevice = GetDeviceContext(FxDevice);

	WdfTimerStop(pDevice->Timer, TRUE);

	pDevice->ConnectInterrupt = false;

	return STATUS_SUCCESS;
}

VOID
CodecJackSwitchWorkItem(
	IN WDFWORKITEM  WorkItem
)
{
	WDFDEVICE Device = (WDFDEVICE)WdfWorkItemGetParentObject(WorkItem);
	PMAXM_CONTEXT pDevice = GetDeviceContext(Device);

	BYTE gpioState;
	GpioReadDataSynchronously(&pDevice->GpioContext, &gpioState, sizeof(BYTE));
	if (pDevice->HeadphonesConnected != gpioState) {
		pDevice->HeadphonesConnected = gpioState;

		if (pDevice->HeadphonesConnected) {
			max98090_i2c_write(pDevice, M98090_REG_OUTPUT_ENABLE,
				M98090_HPREN_MASK | M98090_HPLEN_MASK |
				M98090_DAREN_MASK | M98090_DALEN_MASK);
		}
		else {
			max98090_i2c_write(pDevice, M98090_REG_OUTPUT_ENABLE,
				M98090_SPREN_MASK | M98090_SPLEN_MASK |
				M98090_DAREN_MASK | M98090_DALEN_MASK);
		}
	}

	BYTE micGpioState = 0;
	GpioReadDataSynchronously(&pDevice->MicGpioContext, &micGpioState, sizeof(BYTE));
	micGpioState = !micGpioState;
	if (pDevice->HeadsetMicConnected != micGpioState) {
		pDevice->HeadsetMicConnected = micGpioState;

		if (pDevice->HeadsetMicConnected) {
			max98090_i2c_write(pDevice, M98090_REG_MIC2_INPUT_LEVEL, (1 << M98090_MIC_PA2EN_SHIFT) | 0x00); //Set PA2EN to 1 for Headset, 0 for internal mic
			max98090_i2c_write(pDevice, M98090_REG_DIGITAL_MIC_ENABLE, (3 << M98090_MICCLK_SHIFT) | 0);
			max98090_i2c_write(pDevice, M98090_REG_INPUT_ENABLE, M98090_ADLEN_MASK | M98090_ADREN_MASK | M98090_MBEN_MASK);
		}
		else {
			max98090_i2c_write(pDevice, M98090_REG_MIC2_INPUT_LEVEL, (0 << M98090_MIC_PA2EN_SHIFT) | 0x00); //Set PA2EN to 1 for Headset, 0 for internal mic
			max98090_i2c_write(pDevice, M98090_REG_DIGITAL_MIC_ENABLE, (3 << M98090_MICCLK_SHIFT) | M98090_DIGMICR_MASK | M98090_DIGMICL_MASK);
			max98090_i2c_write(pDevice, M98090_REG_INPUT_ENABLE, 0x0); //Set to M98090_ADLEN_MASK | M98090_ADREN_MASK | M98090_MBEN_MASK for headset, 0 for internal mic
		}
	}

	WdfObjectDelete(WorkItem);
}

void MaxmJDetTimer(_In_ WDFTIMER hTimer) {
	WDFDEVICE Device = (WDFDEVICE)WdfTimerGetParentObject(hTimer);
	PMAXM_CONTEXT pDevice = GetDeviceContext(Device);

	if (!pDevice->ConnectInterrupt)
		return;

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_WORKITEM_CONFIG workitemConfig;
	WDFWORKITEM hWorkItem;

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(&attributes, MAXM_CONTEXT);
	attributes.ParentObject = Device;
	WDF_WORKITEM_CONFIG_INIT(&workitemConfig, CodecJackSwitchWorkItem);

	WdfWorkItemCreate(&workitemConfig,
		&attributes,
		&hWorkItem);

	WdfWorkItemEnqueue(hWorkItem);
}

NTSTATUS
MaxmEvtDeviceAdd(
IN WDFDRIVER       Driver,
IN PWDFDEVICE_INIT DeviceInit
)
{
	NTSTATUS                      status = STATUS_SUCCESS;
	WDF_IO_QUEUE_CONFIG           queueConfig;
	WDF_OBJECT_ATTRIBUTES         attributes;
	WDFDEVICE                     device;
	WDF_INTERRUPT_CONFIG interruptConfig;
	WDFQUEUE                      queue;
	UCHAR                         minorFunction;
	PMAXM_CONTEXT               devContext;

	UNREFERENCED_PARAMETER(Driver);

	PAGED_CODE();

	MaxmPrint(DEBUG_LEVEL_INFO, DBG_PNP,
		"MaxmEvtDeviceAdd called\n");

	{
		WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
		WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

		pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
		pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
		pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
		pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;

		WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
	}

	//
	// Because we are a virtual device the root enumerator would just put null values 
	// in response to IRP_MN_QUERY_ID. Lets override that.
	//

	minorFunction = IRP_MN_QUERY_ID;

	status = WdfDeviceInitAssignWdmIrpPreprocessCallback(
		DeviceInit,
		MaxmEvtWdmPreprocessMnQueryId,
		IRP_MJ_PNP,
		&minorFunction,
		1
		);
	if (!NT_SUCCESS(status))
	{
		MaxmPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfDeviceInitAssignWdmIrpPreprocessCallback failed Status 0x%x\n", status);

		return status;
	}

	//
	// Setup the device context
	//

	WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, MAXM_CONTEXT);

	//
	// Create a framework device object.This call will in turn create
	// a WDM device object, attach to the lower stack, and set the
	// appropriate flags and attributes.
	//

	status = WdfDeviceCreate(&DeviceInit, &attributes, &device);

	if (!NT_SUCCESS(status))
	{
		MaxmPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfDeviceCreate failed with status code 0x%x\n", status);

		return status;
	}

	WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);

	queueConfig.EvtIoInternalDeviceControl = MaxmEvtInternalDeviceControl;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&queue
		);

	if (!NT_SUCCESS(status))
	{
		MaxmPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	//
	// Create manual I/O queue to take care of hid report read requests
	//

	devContext = GetDeviceContext(device);

	WDF_IO_QUEUE_CONFIG_INIT(&queueConfig, WdfIoQueueDispatchManual);

	queueConfig.PowerManaged = WdfFalse;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&devContext->ReportQueue
		);

	if (!NT_SUCCESS(status))
	{
		MaxmPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	WDF_TIMER_CONFIG              timerConfig;
	WDFTIMER                      hTimer;

	WDF_TIMER_CONFIG_INIT_PERIODIC(&timerConfig, MaxmJDetTimer, 10);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	attributes.ParentObject = device;
	status = WdfTimerCreate(&timerConfig, &attributes, &hTimer);
	devContext->Timer = hTimer;
	if (!NT_SUCCESS(status))
	{
		MaxmPrint(DEBUG_LEVEL_ERROR, DBG_PNP, "(%!FUNC!) WdfTimerCreate failed status:%!STATUS!\n", status);
		return status;
	}

	devContext->FxDevice = device;

	return status;
}

NTSTATUS
MaxmEvtWdmPreprocessMnQueryId(
WDFDEVICE Device,
PIRP Irp
)
{
	NTSTATUS            status;
	PIO_STACK_LOCATION  IrpStack, previousSp;
	PDEVICE_OBJECT      DeviceObject;
	PWCHAR              buffer;

	PAGED_CODE();

	//
	// Get a pointer to the current location in the Irp
	//

	IrpStack = IoGetCurrentIrpStackLocation(Irp);

	//
	// Get the device object
	//
	DeviceObject = WdfDeviceWdmGetDeviceObject(Device);


	MaxmPrint(DEBUG_LEVEL_VERBOSE, DBG_PNP,
		"MaxmEvtWdmPreprocessMnQueryId Entry\n");

	//
	// This check is required to filter out QUERY_IDs forwarded
	// by the HIDCLASS for the parent FDO. These IDs are sent
	// by PNP manager for the parent FDO if you root-enumerate this driver.
	//
	previousSp = ((PIO_STACK_LOCATION)((UCHAR *)(IrpStack)+
		sizeof(IO_STACK_LOCATION)));

	if (previousSp->DeviceObject == DeviceObject)
	{
		//
		// Filtering out this basically prevents the Found New Hardware
		// popup for the root-enumerated Maxm on reboot.
		//
		status = Irp->IoStatus.Status;
	}
	else
	{
		switch (IrpStack->Parameters.QueryId.IdType)
		{
		case BusQueryDeviceID:
		case BusQueryHardwareIDs:
			//
			// HIDClass is asking for child deviceid & hardwareids.
			// Let us just make up some id for our child device.
			//
			buffer = (PWCHAR)ExAllocatePoolWithTag(
				NonPagedPool,
				MAXM_HARDWARE_IDS_LENGTH,
				MAXM_POOL_TAG
				);

			if (buffer)
			{
				//
				// Do the copy, store the buffer in the Irp
				//
				RtlCopyMemory(buffer,
					MAXM_HARDWARE_IDS,
					MAXM_HARDWARE_IDS_LENGTH
					);

				Irp->IoStatus.Information = (ULONG_PTR)buffer;
				status = STATUS_SUCCESS;
			}
			else
			{
				//
				//  No memory
				//
				status = STATUS_INSUFFICIENT_RESOURCES;
			}

			Irp->IoStatus.Status = status;
			//
			// We don't need to forward this to our bus. This query
			// is for our child so we should complete it right here.
			// fallthru.
			//
			IoCompleteRequest(Irp, IO_NO_INCREMENT);

			break;

		default:
			status = Irp->IoStatus.Status;
			IoCompleteRequest(Irp, IO_NO_INCREMENT);
			break;
		}
	}

	MaxmPrint(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"MaxmEvtWdmPreprocessMnQueryId Exit = 0x%x\n", status);

	return status;
}

VOID
MaxmEvtInternalDeviceControl(
IN WDFQUEUE     Queue,
IN WDFREQUEST   Request,
IN size_t       OutputBufferLength,
IN size_t       InputBufferLength,
IN ULONG        IoControlCode
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	WDFDEVICE           device;
	PMAXM_CONTEXT     devContext;

	UNREFERENCED_PARAMETER(OutputBufferLength);
	UNREFERENCED_PARAMETER(InputBufferLength);

	device = WdfIoQueueGetDevice(Queue);
	devContext = GetDeviceContext(device);

	switch (IoControlCode)
	{
	default:
		status = STATUS_NOT_SUPPORTED;
		break;
	}

	WdfRequestComplete(Request, status);

	return;
}
