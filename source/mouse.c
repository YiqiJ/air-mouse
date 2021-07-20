#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_hid.h"

#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "mouse.h"
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "fsl_fxos.h"
#include "fsl_pit.h"
#include "math.h"

#include "peripherals.h"
#include "pin_mux.h"

#include <stdio.h>
#include <stdlib.h>
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_RELEASE_BUS_COUNT 100U
#define MAX_ACCEL_AVG_COUNT 25U
#define HWTIMER_PERIOD 10000U
/* multiplicative conversion constants */
#define DegToRad 0.017453292
#define RadToDeg 57.295779

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

static usb_status_t USB_DeviceHidMouseAction(void);
static usb_status_t USB_DeviceHidInterruptIn(usb_device_handle deviceHandle,
                                             usb_device_endpoint_callback_message_struct_t *event,
                                             void *arg);
static void USB_DeviceApplicationInit(void);
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U))
extern void HW_TimerControl(uint8_t enable);
#endif

void BOARD_I2C_ReleaseBus(void);

/*
 * Read all data from sensors, accelerometer and magnetometer, function.
 * Must calculate g_dataScale before use this function.
 *
 * [Ax] The pointer store x axis acceleration value
 * [A]y The pointer store y axis acceleration value
 * [Az] The pointer store z axis acceleration value
 * [Mx] The pointer store x axis magnetic value
 * [My] The pointer store y axis magnetic value
 * [Mz] The pointer store z axis magnetic value
 */
static void Sensor_ReadData(int16_t *Ax, int16_t *Ay, int16_t *Az, int16_t *Mx, int16_t *My, int16_t *Mz);

/*
 * Magnetometer calibration
 *
 */
static int Magnetometer_Calibrate(void);

/*
 * Hardware timer initialize
 *
 */
static void HW_Timer_init(void);

/*
 * Delay function
 *
 * ticks Cycle clock delay
 */
static void Delay(uint32_t ticks);

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_SetupOutBuffer[8];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_MouseBuffer[USB_HID_MOUSE_REPORT_LENGTH];
usb_hid_mouse_struct_t g_UsbDeviceHidMouse;

extern uint8_t g_UsbDeviceCurrentConfigure;
extern uint8_t g_UsbDeviceInterface[USB_HID_MOUSE_INTERFACE_COUNT];

/* Whether the SW button is pressed */
volatile bool g_SW3Press     = false;
volatile bool g_SW2Press     = false;
volatile bool g_SW3LongPress = false;
/* Whether the mouse is active */
volatile bool g_MouseMode    = false;

volatile bool g_Pit0IsrFlag = false;
volatile bool g_PIT1IsrFlag = false;

volatile uint16_t SampleEventFlag;
fxos_handle_t g_fxosHandle;
uint8_t g_sensor_address[] = {0x1CU, 0x1EU, 0x1DU, 0x1FU};
uint8_t g_sensorRange      = 0;
uint8_t g_dataScale        = 0;

int16_t g_Ax_Raw = 0;
int16_t g_Ay_Raw = 0;
int16_t g_Az_Raw = 0;

double g_Ax = 0;
double g_Ay = 0;
double g_Az = 0;

int16_t g_Ax_buff[MAX_ACCEL_AVG_COUNT] = {0};
int16_t g_Ay_buff[MAX_ACCEL_AVG_COUNT] = {0};
int16_t g_Az_buff[MAX_ACCEL_AVG_COUNT] = {0};

int16_t g_Mx_Raw = 0;
int16_t g_My_Raw = 0;
int16_t g_Mz_Raw = 0;

int16_t g_Mx_Offset = 0;
int16_t g_My_Offset = 0;
int16_t g_Mz_Offset = 0;

double g_Mx = 0;
double g_My = 0;
double g_Mz = 0;

double g_Mx_LP = 0;
double g_My_LP = 0;
double g_Mz_LP = 0;

double g_Yaw       = 0;
double g_Yaw_LP    = 0;
double g_Pitch     = 0;
double g_Pitch_LP  = 0;
double g_Roll      = 0;
double g_Roll_LP   = 0;

double g_Yaw_Origin   = 0;
double g_Roll_Origin  = 0;
double g_Pitch_Origin = 0;

bool g_FirstRun = true;

int8_t g_YawSpeed[2]   = {0};
int8_t g_RollSpeed[2]  = {0};
int8_t g_PitchSpeed[2] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/

void USB0_IRQHandler(void)
{
    USB_DeviceKhciIsrFunction(g_UsbDeviceHidMouse.deviceHandle);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    __DSB();
}
void USB_DeviceClockInit(void)
{
    SystemCoreClockUpdate();
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
}
void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbDeviceKhciIrq[] = USB_IRQS;
    irqNumber                  = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];

    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
    USB_DeviceKhciTaskFunction(deviceHandle);
}
#endif

/* Update mouse pointer location. Draw a rectangular rotation */
#define MOUSE_SPEED_MIN  1
static usb_status_t USB_DeviceHidMouseAction(void)
{

	int8_t x = g_YawSpeed[1];
	int8_t y = g_RollSpeed[1];

	g_UsbDeviceHidMouse.buffer[0] = 0;
	g_UsbDeviceHidMouse.buffer[1] = 0;
	g_UsbDeviceHidMouse.buffer[2] = 0;

	if (g_MouseMode) {

		if (y >= MOUSE_SPEED_MIN) {
			g_UsbDeviceHidMouse.buffer[2] = (uint8_t)(y - MOUSE_SPEED_MIN + 1);
		}
		else if (y <= -MOUSE_SPEED_MIN) {
			g_UsbDeviceHidMouse.buffer[2] = (uint8_t)(y + MOUSE_SPEED_MIN - 1);
		}

		if (x >= MOUSE_SPEED_MIN) {
			g_UsbDeviceHidMouse.buffer[1] = (uint8_t)(x - MOUSE_SPEED_MIN + 1);
		}
		else if (x <= -MOUSE_SPEED_MIN) {
			g_UsbDeviceHidMouse.buffer[1] = (uint8_t)(x + MOUSE_SPEED_MIN - 1);
		}

		if (g_SW2Press) {
			g_UsbDeviceHidMouse.buffer[0] |= 0x01;	// mouse left button
		}
	}

    /* Send mouse report to the host */
    return USB_DeviceSendRequest(g_UsbDeviceHidMouse.deviceHandle, USB_HID_MOUSE_ENDPOINT_IN,
                                 g_UsbDeviceHidMouse.buffer, USB_HID_MOUSE_REPORT_LENGTH);
}

/* HID mouse interrupt IN pipe callback */
static usb_status_t USB_DeviceHidInterruptIn(usb_device_handle deviceHandle,
                                             usb_device_endpoint_callback_message_struct_t *event,
                                             void *arg)
{
    /* Resport sent */
    if (g_UsbDeviceHidMouse.attach)
    {
        if ((NULL != event) && (event->length == USB_UNINITIALIZED_VAL_32))
        {
            return kStatus_USB_Error;
        }
        return USB_DeviceHidMouseAction();
    }

    return kStatus_USB_Error;
}

usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Success;
    uint8_t *temp8     = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            /* USB bus reset signal detected */
            /* Initialize the control IN and OUT pipes */
            USB_DeviceControlPipeInit(g_UsbDeviceHidMouse.deviceHandle);
            g_UsbDeviceHidMouse.attach = 0U;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceGetStatus(g_UsbDeviceHidMouse.deviceHandle, kUSB_DeviceStatusSpeed,
                                                           &g_UsbDeviceHidMouse.speed))
            {
                USB_DeviceSetSpeed(g_UsbDeviceHidMouse.speed);
            }
#endif
        }
        break;
#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))
        case kUSB_DeviceEventAttach:
        {
            g_UsbDeviceHidMouse.connectStateChanged = 1U;
            g_UsbDeviceHidMouse.connectState        = 1U;
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
            g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionInit;
#else

#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
#else
            USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
#endif
        }
        break;
        case kUSB_DeviceEventDetach:
        {
            g_UsbDeviceHidMouse.connectStateChanged = 1U;
            g_UsbDeviceHidMouse.connectState        = 0U;
            g_UsbDeviceHidMouse.attach              = 0U;
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
            g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionInit;
#else

#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
#else
            USB_DeviceStop(g_UsbDeviceHidMouse.deviceHandle);
#endif

#endif
        }
        break;
#endif
        case kUSB_DeviceEventSetConfiguration:
            if (USB_HID_MOUSE_CONFIGURE_INDEX == (*temp8))
            {
                /* If the configuration is valid, initialized the HID mouse interrupt IN pipe */
                usb_device_endpoint_init_struct_t epInitStruct;
                usb_device_endpoint_callback_struct_t epCallback;

                epCallback.callbackFn    = USB_DeviceHidInterruptIn;
                epCallback.callbackParam = handle;

                epInitStruct.zlt          = 0U;
                epInitStruct.transferType = USB_ENDPOINT_INTERRUPT;
                epInitStruct.endpointAddress =
                    USB_HID_MOUSE_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                if (USB_SPEED_HIGH == g_UsbDeviceHidMouse.speed)
                {
                    epInitStruct.maxPacketSize = HS_HID_MOUSE_INTERRUPT_IN_PACKET_SIZE;
                    epInitStruct.interval      = HS_HID_MOUSE_INTERRUPT_IN_INTERVAL;
                }
                else
                {
                    epInitStruct.maxPacketSize = FS_HID_MOUSE_INTERRUPT_IN_PACKET_SIZE;
                    epInitStruct.interval      = FS_HID_MOUSE_INTERRUPT_IN_INTERVAL;
                }

                USB_DeviceInitEndpoint(g_UsbDeviceHidMouse.deviceHandle, &epInitStruct, &epCallback);

                g_UsbDeviceHidMouse.attach = 1U;
                error                      = USB_DeviceHidMouseAction(); /* run the cursor movement code */
            }
            break;
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
        case kUSB_DeviceEventDcdDetectionfinished:
            /*temp pointer point to detection result*/
            if (param)
            {
                if (kUSB_DcdSDP == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionSDP;
                }
                else if (kUSB_DcdCDP == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionCDP;
                }
                else if (kUSB_DcdDCP == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionDCP;
                }
                else if (kUSB_DcdTimeOut == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionTimeOut;
                }
                else if (kUSB_DcdError == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionError;
                }
                else
                {
                }
            }
            break;
#endif
        default:
            break;
    }

    return error;
}

/* Get setup buffer */
usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer)
{
    /* Keep the setup is 4-byte aligned */
    static uint32_t hid_mouse_setup[2];
    if (NULL == setupBuffer)
    {
        return kStatus_USB_InvalidParameter;
    }
    *setupBuffer = (usb_setup_struct_t *)&hid_mouse_setup;
    return kStatus_USB_Success;
}

/* Configure device remote wakeup */
usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable)
{
    return kStatus_USB_InvalidRequest;
}

/* Configure the endpoint status (idle or stall) */
usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
    if (status)
    {
        if ((USB_HID_MOUSE_ENDPOINT_IN == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else
        {
        }
    }
    else
    {
        if ((USB_HID_MOUSE_ENDPOINT_IN == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceUnstallEndpoint(handle, ep);
        }
        else
        {
        }
    }
    return kStatus_USB_InvalidRequest;
}

/* Get class-specific request buffer */
usb_status_t USB_DeviceGetClassReceiveBuffer(usb_device_handle handle,
                                             usb_setup_struct_t *setup,
                                             uint32_t *length,
                                             uint8_t **buffer)
{
    if ((NULL == buffer) || ((*length) > sizeof(s_SetupOutBuffer)))
    {
        return kStatus_USB_InvalidRequest;
    }
    *buffer = s_SetupOutBuffer;
    return kStatus_USB_Success;
}

/* Handle class-specific request */
usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    if (setup->wIndex != USB_HID_MOUSE_INTERFACE_INDEX)
    {
        return error;
    }

    switch (setup->bRequest)
    {
        case USB_DEVICE_HID_REQUEST_GET_REPORT:
            break;
        case USB_DEVICE_HID_REQUEST_GET_IDLE:
            break;
        case USB_DEVICE_HID_REQUEST_GET_PROTOCOL:
            break;
        case USB_DEVICE_HID_REQUEST_SET_REPORT:
            break;
        case USB_DEVICE_HID_REQUEST_SET_IDLE:
            break;
        case USB_DEVICE_HID_REQUEST_SET_PROTOCOL:
            break;
        default:
            break;
    }

    return error;
}

static void USB_DeviceApplicationInit(void)
{
    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    /* Set HID mouse default state */
    g_UsbDeviceHidMouse.speed        = USB_SPEED_FULL;
    g_UsbDeviceHidMouse.attach       = 0U;
    g_UsbDeviceHidMouse.deviceHandle = NULL;
    g_UsbDeviceHidMouse.buffer       = s_MouseBuffer;

    /* Initialize the usb stack and class drivers */
    if (kStatus_USB_Success != USB_DeviceInit(CONTROLLER_ID, USB_DeviceCallback, &g_UsbDeviceHidMouse.deviceHandle))
    {
        usb_echo("USB device mouse failed\r\n");
        return;
    }
    else
    {
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
        usb_echo("USB device DCD + HID mouse demo\r\n");
#else
        usb_echo("USB device HID mouse demo\r\n");
#endif
    }
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionInit;
#endif
    USB_DeviceIsrEnable();

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))
    /*USB_DeviceRun could not be called here to avoid DP/DM confliction between DCD function and USB function in case
      DCD is enabled. Instead, USB_DeviceRun should be called after the DCD is finished immediately*/
#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
    /* Start USB device HID mouse */
    USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
#else
    /* Start USB device HID mouse */
    USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
}

/*!
 * @brief Application task function.
 *
 * This function runs the task for application.
 *
 * @return None.
 */
#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))
void USB_DeviceAppTask(void *parameter)
{
    usb_hid_mouse_struct_t *usbDeviceHid = (usb_hid_mouse_struct_t *)parameter;

    if (usbDeviceHid->connectStateChanged)
    {
        usbDeviceHid->connectStateChanged = 0;
        if (g_UsbDeviceHidMouse.connectState)
        {
            /*user need call USB_DeviceRun here to usb function run if dcd function is disabled*/
            /*USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);*/
            usb_echo("USB device attached.\r\n");
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U))
            HW_TimerControl(1U);
#endif
        }
        else
        {
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U))
            /*USB_DeviceStop should be called here to avoid DP/DM confliction between DCD function and USB function in
             * case next time DCD dection. */
            USB_DeviceStop(g_UsbDeviceHidMouse.deviceHandle);
#endif
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U))
            HW_TimerControl(0U);
#endif
            usb_echo("USB device detached.\r\n");
        }
    }
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
    if ((kUSB_DeviceDCDDectionInit != usbDeviceHid->dcdDectionStatus) &&
        (kUSB_DeviceDCDDectionFinished != usbDeviceHid->dcdDectionStatus))
    {
        switch (usbDeviceHid->dcdDectionStatus)
        {
            case kUSB_DeviceDCDDectionSDP:
            {
                usb_echo("SDP(standard downstream port) is detected.\r\n");
                /* Start USB device HID mouse */
                USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
            }
            break;
            case kUSB_DeviceDCDDectionDCP:
            {
                usb_echo("DCP (dedicated charging port) is detected.\r\n");
#if (defined(FSL_FEATURE_USBPHY_HAS_DCD_ANALOG) && (FSL_FEATURE_USBPHY_HAS_DCD_ANALOG > 0U))
                /*
                 * This is a work-around to fix the DCP device detach event missing issue.
                 * The device (IP3511HS controller) VBUSDEBOUNCED bit is not updated on time before softeware read this
                 * bit, so when DCP is detached from usb port, softeware can't detect DCP disconnection.
                 */
                USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
            }
            break;
            case kUSB_DeviceDCDDectionCDP:
            {
                usb_echo("CDP(charging downstream port) is detected.\r\n");
                /* Start USB device HID mouse */
                USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
            }
            break;
            case kUSB_DeviceDCDDectionTimeOut:
            {
                usb_echo("Timeout error.\r\n");
            }
            break;
            case kUSB_DeviceDCDDectionError:
            {
                usb_echo("Detect error.\r\n");
            }
            break;
            default:
                break;
        }
        usbDeviceHid->dcdDectionStatus = kUSB_DeviceDCDDectionFinished;
    }
#endif
}
#endif




static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;

    BOARD_GPIO_ConfigurePins();

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(BOARD_GPIO_ACCEL_SDA_GPIO, BOARD_GPIO_ACCEL_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(BOARD_GPIO_ACCEL_SCL_GPIO, BOARD_GPIO_ACCEL_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(BOARD_GPIO_ACCEL_SDA_GPIO, BOARD_GPIO_ACCEL_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(BOARD_GPIO_ACCEL_SCL_GPIO, BOARD_GPIO_ACCEL_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(BOARD_GPIO_ACCEL_SCL_GPIO, BOARD_GPIO_ACCEL_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(BOARD_GPIO_ACCEL_SDA_GPIO, BOARD_GPIO_ACCEL_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(BOARD_GPIO_ACCEL_SCL_GPIO, BOARD_GPIO_ACCEL_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(BOARD_GPIO_ACCEL_SDA_GPIO, BOARD_GPIO_ACCEL_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
static void HW_Timer_init(void)
{
    /* Configure the SysTick timer */
    SysTick_Config(SystemCoreClock / HWTIMER_PERIOD);
}

void SysTick_Handler(void)
{
    SampleEventFlag = 1;
}

static void Delay(uint32_t ticks)
{
    while (ticks--)
    {
        __asm("nop");
    }
}

static void Sensor_ReadData(int16_t *Ax, int16_t *Ay, int16_t *Az, int16_t *Mx, int16_t *My, int16_t *Mz)
{
    fxos_data_t fxos_data;

    if (kStatus_Success != FXOS_ReadSensorData(&g_fxosHandle, &fxos_data))
    {
        PRINTF("Failed to read acceleration data!\r\n");
    }
    /* Get the accel data from the sensor data structure in 14 bit left format data*/
    *Ax = (int16_t)((uint16_t)((uint16_t)fxos_data.accelXMSB << 8) | (uint16_t)fxos_data.accelXLSB) / 4U;
    *Ay = (int16_t)((uint16_t)((uint16_t)fxos_data.accelYMSB << 8) | (uint16_t)fxos_data.accelYLSB) / 4U;
    *Az = (int16_t)((uint16_t)((uint16_t)fxos_data.accelZMSB << 8) | (uint16_t)fxos_data.accelZLSB) / 4U;
    *Ax *= g_dataScale;
    *Ay *= g_dataScale;
    *Az *= g_dataScale;
    *Mx = (int16_t)((uint16_t)((uint16_t)fxos_data.magXMSB << 8) | (uint16_t)fxos_data.magXLSB);
    *My = (int16_t)((uint16_t)((uint16_t)fxos_data.magYMSB << 8) | (uint16_t)fxos_data.magYLSB);
    *Mz = (int16_t)((uint16_t)((uint16_t)fxos_data.magZMSB << 8) | (uint16_t)fxos_data.magZLSB);
}

static int Magnetometer_Calibrate(void)
{
    int16_t Mx_max = 0;
    int16_t My_max = 0;
    int16_t Mz_max = 0;
    int16_t Mx_min = 0;
    int16_t My_min = 0;
    int16_t Mz_min = 0;

    uint32_t times = 0;
    PRINTF("\r\nCalibrating magnetometer...");
    while (times < 10000)
    {
        Sensor_ReadData(&g_Ax_Raw, &g_Ay_Raw, &g_Az_Raw, &g_Mx_Raw, &g_My_Raw, &g_Mz_Raw);
        if (times == 0)
        {
            Mx_max = Mx_min = g_Mx_Raw;
            My_max = My_min = g_My_Raw;
            Mz_max = Mz_min = g_Mz_Raw;
        }
        Mx_max = MAX(Mx_max, g_Mx_Raw);
        My_max = MAX(My_max, g_My_Raw);
        Mz_max = MAX(Mz_max, g_Mz_Raw);
        Mx_min = MIN(Mx_min, g_Mx_Raw);
        My_min = MIN(My_min, g_My_Raw);
        Mz_min = MIN(Mz_min, g_Mz_Raw);
        if (times == 9999)
        {
            if ((Mx_max > (Mx_min + 500)) && (My_max > (My_min + 500)) && (Mz_max > (Mz_min + 500)))
            {
                g_Mx_Offset = (Mx_max + Mx_min) / 2;
                g_My_Offset = (My_max + My_min) / 2;
                g_Mz_Offset = (Mz_max + Mz_min) / 2;
                PRINTF("\r\nCalibrate magnetometer successfully!");
                PRINTF("\r\nMagnetometer offset Mx: %d - My: %d - Mz: %d \r\n", g_Mx_Offset, g_My_Offset, g_Mz_Offset);
            }
            else
            {
                  /* use for manually calibration */
//                PRINTF("Calibrating magnetometer failed! Press any key to recalibrate...\r\n");
//                GETCHAR();
//                PRINTF("\r\nCalibrating magnetometer...");
//                times = 0;
            	return 1;	// failed
            }
        }
        times++;
        Delay(3000);
    }
    return 0;
}

/*
 * Interrupt Handler of switch 3.
 *
 * This function toggles the LED
 */
void BOARD_SW3_IRQ_HANDLER(void)
{
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
#else
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);
#endif
    if (!GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN)) {
       	// press
       	PIT_StartTimer(PIT, kPIT_Chnl_1);
       }
       else {
       	// release
       	PIT_StopTimer(PIT, kPIT_Chnl_1);

       	if (!g_PIT1IsrFlag) {
       		g_SW3Press = true;
       	}
	}
    g_PIT1IsrFlag = false;

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*
 * Interrupt Handler of switch 2.
 */
void BOARD_SW2_IRQ_HANDLER(void)
{
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
#else
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);
#endif

    if (!GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN)) {
		// press
		g_SW2Press = true;
	}
	else {
		// release
		g_SW2Press = false;
	}

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*
 * PIT0 interrupt handler. Toggle the blue LED while calibrating the magnetometer.
 */
void PIT0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    LED_BLUE_TOGGLE();
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

/*
 * PIT1 interrupt handler.
 * Check for long press.
 */
void PIT1_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_1, kPIT_TimerFlag);
    PIT_StopTimer(PIT, kPIT_Chnl_1);
    g_SW3LongPress = true;
    g_PIT1IsrFlag = true;
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

void Timer_Init(void)
{
	pit_config_t pitConfig;

	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(500000U, CLOCK_GetFreq(kCLOCK_BusClk)));

	/* Set timer period for channel 1 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_1, USEC_TO_COUNT(3000000U, CLOCK_GetFreq(kCLOCK_BusClk)));

	/* Enable timer interrupts */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	PIT_EnableInterrupts(PIT, kPIT_Chnl_1, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT0_IRQn);
	EnableIRQ(PIT1_IRQn);

}

void Timer_Start(void)
{
	PIT_StartTimer(PIT, kPIT_Chnl_0);
}

void Timer_Stop(void)
{
	PIT_StopTimer(PIT, kPIT_Chnl_0);
}

int main(void)
{

	fxos_config_t config = {0};
	status_t result;
	uint16_t i              = 0;
	uint16_t loopCounter    = 0;
	double sinAngle         = 0;
	double cosAngle         = 0;
	double Bx               = 0;
	double By               = 0;
	double Roll = 0, Pitch = 0, Yaw = 0;
	double RollLast = 0, YawLast = 0, PitchLast = 0;
	uint8_t array_addr_size = 0;
	bool set_origin = false;
	int RollSpeed = 0;
	int YawSpeed = 0;
	int PitchSpeed = 0;

	int8_t temp_speed = 0;

	/* Define the initial structure for the input switch pin */
	gpio_pin_config_t sw_config = {
		kGPIO_DigitalInput,
		0,
	};

	/* Define the inititial structure for the output LED pin */
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput,
		1,
	};


    BOARD_InitPins();
	BOARD_InitBootClocks();
	BOARD_I2C_ReleaseBus();
	BOARD_I2C_ConfigurePins();
	BOARD_InitDebugConsole();
	BOARD_InitPeripherals();

    USB_DeviceApplicationInit();

    HW_Timer_init();

	/* Configure the I2C function */
	config.I2C_SendFunc    = BOARD_Accel_I2C_Send;
	config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

	/* Initialize sensor devices */
	array_addr_size = sizeof(g_sensor_address) / sizeof(g_sensor_address[0]);
	for (i = 0; i < array_addr_size; i++)
	{
		config.slaveAddress = g_sensor_address[i];
		/* Initialize accelerometer sensor */
		result = FXOS_Init(&g_fxosHandle, &config);
		if (result == kStatus_Success)
		{
			break;
		}
	}

	if (kStatus_Success != result)
	{
		PRINTF("\r\nSensor device initialize failed!\r\n");
	}

	/* Get sensor range */
	if (kStatus_Success != FXOS_ReadReg(&g_fxosHandle, XYZ_DATA_CFG_REG, &g_sensorRange, 1))
	{
		PRINTF("\r\nGet sensor range failed!\r\n");
	}

	switch (g_sensorRange)
	{
		case 0x00:
			g_dataScale = 2U;
			break;
		case 0x01:
			g_dataScale = 4U;
			break;
		case 0x10:
			g_dataScale = 8U;
			break;
		default:
			break;
	}

    /* instructions for manual calibration */
//	PRINTF("\r\nTo calibrate Magnetometer, roll the board on all orientations to get max and min values\r\n");
//	PRINTF("\r\nPress any key to start calibrating...\r\n");
//	GETCHAR();
//	Magnetometer_Calibrate();

	// do calibration myself
	g_Mx_Offset = 456;
	g_My_Offset = -61;
	g_Mz_Offset = 731;


	PORT_SetPinInterruptConfig(BOARD_SW3_PORT, BOARD_SW3_GPIO_PIN, kPORT_InterruptEitherEdge);
	EnableIRQ(BOARD_SW3_IRQ);
	GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);

	PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_GPIO_PIN, kPORT_InterruptEitherEdge);
	EnableIRQ(BOARD_SW2_IRQ);
	GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);

	/* Init output LED GPIO. */
	GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
	GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);

	Timer_Init();


    while (1U)
    {
    	if (g_SW3Press)
		{
			g_MouseMode = !g_MouseMode;
			if (g_MouseMode) {
				LED_RED_ON();
			}
			else {
				LED_RED_OFF();
			}
			/* Reset state of button. */
			g_SW3Press = false;
			if (g_MouseMode) {
				set_origin = true;
			}
		}

    	if (g_SW3LongPress)
    	{
    		if (!g_MouseMode) {
    			// should not do calibration under mouse mode
				LED_RED_OFF();
				LED_GREEN_OFF();
				Timer_Start();
				if (Magnetometer_Calibrate() == 0) {
					// calibration success
					LED_GREEN_ON();
				}
				else {
					// calibration failed
					LED_RED_ON();
				}
				Timer_Stop();
				LED_BLUE_OFF();
				SDK_DelayAtLeastUs(1000000, CLOCK_GetFreq(kCLOCK_CoreSysClk));	// delay 1s
				LED_GREEN_OFF();
				LED_RED_OFF();
    		}
    		g_SW3LongPress = false;
    	}

    	if (SampleEventFlag == 1) /* Fix loop */
		{
			SampleEventFlag = 0;
			g_Ax_Raw        = 0;
			g_Ay_Raw        = 0;
			g_Az_Raw        = 0;
			g_Ax            = 0;
			g_Ay            = 0;
			g_Az            = 0;
			g_Mx_Raw        = 0;
			g_My_Raw        = 0;
			g_Mz_Raw        = 0;
			g_Mx            = 0;
			g_My            = 0;
			g_Mz            = 0;

			/* Read sensor data */
			Sensor_ReadData(&g_Ax_Raw, &g_Ay_Raw, &g_Az_Raw, &g_Mx_Raw, &g_My_Raw, &g_Mz_Raw);

			/* Average accelerometer value */
			for (i = 1; i < MAX_ACCEL_AVG_COUNT; i++)
			{
				g_Ax_buff[i] = g_Ax_buff[i - 1];
				g_Ay_buff[i] = g_Ay_buff[i - 1];
				g_Az_buff[i] = g_Az_buff[i - 1];
			}

			g_Ax_buff[0] = g_Ax_Raw;
			g_Ay_buff[0] = g_Ay_Raw;
			g_Az_buff[0] = g_Az_Raw;

			for (i = 0; i < MAX_ACCEL_AVG_COUNT; i++)
			{
				g_Ax += (double)g_Ax_buff[i];
				g_Ay += (double)g_Ay_buff[i];
				g_Az += (double)g_Az_buff[i];
			}

			g_Ax /= MAX_ACCEL_AVG_COUNT;
			g_Ay /= MAX_ACCEL_AVG_COUNT;
			g_Az /= MAX_ACCEL_AVG_COUNT;

			if (g_FirstRun)
			{
				g_Mx_LP = g_Mx_Raw;
				g_My_LP = g_My_Raw;
				g_Mz_LP = g_Mz_Raw;
			}

			g_Mx_LP += ((double)g_Mx_Raw - g_Mx_LP) * 0.01;
			g_My_LP += ((double)g_My_Raw - g_My_LP) * 0.01;
			g_Mz_LP += ((double)g_Mz_Raw - g_Mz_LP) * 0.01;

			/* Calculate magnetometer values */
			g_Mx = g_Mx_LP - g_Mx_Offset;
			g_My = g_My_LP - g_My_Offset;
			g_Mz = g_Mz_LP - g_Mz_Offset;

			/* Calculate roll angle g_Roll (-180deg, 180deg) and sin, cos */
			Roll   = atan2(g_Ay, g_Az) * RadToDeg;
			sinAngle = sin(Roll * DegToRad);
			cosAngle = cos(Roll * DegToRad);

			/* De-rotate by roll angle g_Roll */
			By   = g_My * cosAngle - g_Mz * sinAngle;
			g_Mz = g_Mz * cosAngle + g_My * sinAngle;
			g_Az = g_Ay * sinAngle + g_Az * cosAngle;

			/* Calculate pitch angle g_Pitch (-90deg, 90deg) and sin, cos*/
			Pitch  = atan2(-g_Ax, g_Az) * RadToDeg;
			sinAngle = sin(Pitch * DegToRad);
			cosAngle = cos(Pitch * DegToRad);

			/* De-rotate by pitch angle g_Pitch */
			Bx = g_Mx * cosAngle + g_Mz * sinAngle;

			/* Calculate yaw = ecompass angle psi (-180deg, 180deg) */
			Yaw = atan2(-By, Bx) * RadToDeg;

			if (set_origin) {
				g_Yaw_Origin = Yaw;
				g_Roll_Origin = Roll;
				g_Pitch_Origin = Pitch;
				YawLast = 0;
				RollLast = 0;
				PitchLast = 0;
				set_origin = false;
				g_Yaw_LP = 0;
			}

			Yaw = Yaw - g_Yaw_Origin;
			if (Yaw > 180) {
				Yaw -= 360;
			}
			else if (Yaw <= -180) {
				Yaw += 360;
			}
			g_Yaw = Yaw;

			Roll = Roll - g_Roll_Origin;
			if (Roll > 180) {
				Roll -= 360;
			}
			else if (Roll <= -180) {
				Roll += 360;
			}
			g_Roll = Roll;

			Pitch = Pitch - g_Pitch_Origin;
			if (Pitch > 180) {
				Pitch -= 360;
			}
			else if (Pitch <= -180) {
				Pitch += 360;
			}
			g_Pitch = Pitch;



			if (g_FirstRun)
			{
				g_Yaw_LP   = g_Yaw;
				g_Pitch_LP = g_Pitch;
				g_Roll_LP = g_Roll;
				g_FirstRun = false;
			}

			g_Yaw_LP += (g_Yaw - g_Yaw_LP) * 0.01;
			g_Pitch_LP += (g_Pitch - g_Pitch_LP) * 0.01;
			g_Roll_LP += (g_Roll - g_Roll_LP) * 0.01;

			if (++loopCounter > 10)
			{

				loopCounter = 0;

				// Yaw
				YawSpeed = (int)(g_Yaw_LP - YawLast);
				if (YawSpeed > 127) {
					YawSpeed = 127;
				}
				else if (YawSpeed < -127) {
					YawSpeed = -127;
				}

				temp_speed = -(int8_t)YawSpeed;
				if ((g_YawSpeed[0] < 0 && temp_speed > 0) || (g_YawSpeed[0] > 0 && temp_speed < 0)) {
					g_YawSpeed[0] += temp_speed;
				}
				else {
					g_YawSpeed[1] = g_YawSpeed[0];
					g_YawSpeed[0] = temp_speed;
				}

				// Roll
				RollSpeed = (int)(g_Roll_LP - RollLast);
				if (RollSpeed > 127) {
					RollSpeed = 127;
				}
				else if (RollSpeed < -127) {
					RollSpeed = -127;
				}

				temp_speed = -(int8_t)RollSpeed;
				if ((g_RollSpeed[0] < 0 && temp_speed > 0) || (g_RollSpeed[0] > 0 && temp_speed < 0)) {
					g_RollSpeed[0] += temp_speed;
				}
				else {
					g_RollSpeed[1] = g_RollSpeed[0];
					g_RollSpeed[0] = temp_speed;
				}

				// Pitch
				PitchSpeed = (int)(g_Pitch_LP - PitchLast);
				if (PitchSpeed > 127) {
					PitchSpeed = 127;
				}
				else if (PitchSpeed < -127) {
					PitchSpeed = -127;
				}

				temp_speed = (int8_t)PitchSpeed;
				if ((g_PitchSpeed[0] < 0 && temp_speed > 0) || (g_PitchSpeed[0] > 0 && temp_speed < 0)) {
					g_PitchSpeed[0] += temp_speed;
				}
				else {
					g_PitchSpeed[1] = g_PitchSpeed[0];
					g_PitchSpeed[0] = temp_speed;
				}

                /* print calculated angles for testing */
				if (g_MouseMode) {
					if (g_YawSpeed[1] || g_PitchSpeed[1] || g_RollSpeed[1]) {
						PRINTF("\r\nAngle: %d, %d, %d, %3.1lf, %3.1lf, %3.1lf", (int)(g_YawSpeed[1]), (int)(g_PitchSpeed[1]), (int)(g_RollSpeed[1]), g_Yaw_LP, g_Pitch_LP, g_Roll_LP);
					}
				}
				RollLast = g_Roll_LP;
				YawLast = g_Yaw_LP;
				PitchLast = g_Pitch_LP;


			}
		}

    }
}
