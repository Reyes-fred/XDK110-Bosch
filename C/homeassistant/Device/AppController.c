/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licensee�s application development. 
* Fitness and suitability of the example code for any use within Licensee�s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/
/**
 * @ingroup APPS_LIST
 *
 * @defgroup SEND_DATA_OVER_MQTT SendDataOverMQTT
 * @{
 *
 * @brief Demo application for using MQTT protocol.
 *
 * @details This example shows how to use the MQTT protocol to transfer the environmental data.
 *
 * You need to add your WLAN-Credentials in \ref AppController.h\n
 * When running the program keep the USB plugged in to the PC.
 * You can see in the console output of the XDK-Workbench the content of the environmental data.
 *
 * @file
 **/
/* module includes ********************************************************** */

/* own header files */
#include "XdkAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_APP_CONTROLLER

/* own header files */
#include "AppController.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "BCDS_BSP_Board.h"
#include "BCDS_CmdProcessor.h"
#include "BCDS_NetworkConfig.h"
#include "BCDS_Assert.h"
#include "XDK_WLAN.h"
#include "XDK_MQTT.h"
#include "XDK_Sensor.h"
#include "XDK_SNTP.h"
#include "XDK_ServalPAL.h"
#include "FreeRTOS.h"
#include "task.h"
#include "XDK_LED.h"

/* constant definitions ***************************************************** */

#define MQTT_CONNECT_TIMEOUT_IN_MS                  UINT32_C(60000)/**< Macro for MQTT connection timeout in milli-second */

#define MQTT_SUBSCRIBE_TIMEOUT_IN_MS                UINT32_C(5000)/**< Macro for MQTT subscription timeout in milli-second */

#define MQTT_PUBLISH_TIMEOUT_IN_MS                  UINT32_C(5000)/**< Macro for MQTT publication timeout in milli-second */

#define APP_TEMPERATURE_OFFSET_CORRECTION               (-3459)/**< Macro for static temperature offset correction. Self heating, temperature correction factor */
#define APP_MQTT_DATA_BUFFER_SIZE                   UINT32_C(256)/**< macro for data size of incoming subscribed and published messages */

#if APP_MQTT_SECURE_ENABLE
#define APP_RESPONSE_FROM_SNTP_SERVER_TIMEOUT       UINT32_C(10000)/**< Timeout for SNTP server time sync */
#endif /* APP_MQTT_SECURE_ENABLE */

/* local variables ********************************************************** */

/*
 * WIFI ENTERPRISE
static WLAN_Setup_T WLANSetupInfo =
        {
                .IsEnterprise = true,
                .IsHostPgmEnabled = true,
                .SSID = WLAN_SSID,
                .Username = WLAN_USRN,
                .Password = WLAN_PSK,
                .IsStatic = WLAN_STATIC_IP,
                .IpAddr = WLAN_IP_ADDR,
                .GwAddr = WLAN_GW_ADDR,
                .DnsAddr = WLAN_DNS_ADDR,
                .Mask = WLAN_MASK,
        };*< WLAN setup parameters
*/
// STANDAR WIFI
static WLAN_Setup_T WLANSetupInfo =
        {
                .IsEnterprise = false,
                .IsHostPgmEnabled = false,
                .SSID = WLAN_SSID,
                .Username = WLAN_USRN,
                .Password = WLAN_PSK,
                .IsStatic = WLAN_STATIC_IP,
                .IpAddr = WLAN_IP_ADDR,
                .GwAddr = WLAN_GW_ADDR,
                .DnsAddr = WLAN_DNS_ADDR,
                .Mask = WLAN_MASK,
        };/**< WLAN setup parameters */

static Sensor_Setup_T SensorSetup =
        {
                .CmdProcessorHandle = NULL,
                .Enable =
                        {
                                .Accel = false,
                                .Mag = false,
                                .Gyro = false,
                                .Humidity = true,
                                .Temp = true,
                                .Pressure = true,
                                .Light = true,
                                .Noise = true,
                        },
                .Config =
                        {
                                .Accel =
                                        {
                                                .Type = SENSOR_ACCEL_BMA280,
                                                .IsRawData = false,
                                                .IsInteruptEnabled = false,
                                                .Callback = NULL,
                                        },
                                .Gyro =
                                        {
                                                .Type = SENSOR_GYRO_BMG160,
                                                .IsRawData = false,
                                        },
                                .Mag =
                                        {
                                                .IsRawData = false
                                        },
                                .Light =
                                        {
                                                .IsInteruptEnabled = false,
                                                .Callback = NULL,
                                        },
                                .Temp =
                                        {
                                                .OffsetCorrection = APP_TEMPERATURE_OFFSET_CORRECTION,
                                        },
                        },
        };/**< Sensor setup parameters */

#if APP_MQTT_SECURE_ENABLE
static SNTP_Setup_T SNTPSetupInfo =
{
    .ServerUrl = SNTP_SERVER_URL,
    .ServerPort = SNTP_SERVER_PORT,
};/**< SNTP setup parameters */
#endif /* APP_MQTT_SECURE_ENABLE */

static MQTT_Setup_T MqttSetupInfo =
        {
                .MqttType = MQTT_TYPE_SERVALSTACK,
                .IsSecure = APP_MQTT_SECURE_ENABLE,
        };/**< MQTT setup parameters */

static MQTT_Connect_T MqttConnectInfo =
        {
                .ClientId = APP_MQTT_CLIENT_ID,
                .BrokerURL = APP_MQTT_BROKER_HOST_URL,
                .BrokerPort = APP_MQTT_BROKER_HOST_PORT,
                .CleanSession = true,
                .KeepAliveInterval = 100,
        };/**< MQTT connect parameters */

static void AppMQTTSubscribeCB(MQTT_SubscribeCBParam_T param);

static MQTT_Subscribe_T MqttSubscribeInfo =
        {
                .Topic = APP_MQTT_TOPIC,
                .QoS = 1UL,
                .IncomingPublishNotificationCB = AppMQTTSubscribeCB,
        };/**< MQTT subscribe parameters */

static MQTT_Publish_T MqttPublishHumidity =
        {
                .Topic = MQTT_TOPIC_HUMIDITY,
                .QoS = 1UL,
                .Payload = NULL,
                .PayloadLength = 0UL,
        };/**< MQTT publish parameters */

static MQTT_Publish_T MqttPublishPressure =
        {
                .Topic = MQTT_TOPIC_PRESSURE,
                .QoS = 1UL,
                .Payload = NULL,
                .PayloadLength = 0UL,
        };/**< MQTT publish parameters */

static MQTT_Publish_T MqttPublishTemperature =
        {
                .Topic = MQTT_TOPIC_TEMPERATURE,
                .QoS = 1UL,
                .Payload = NULL,
                .PayloadLength = 0UL,
        };/**< MQTT publish parameters */

static MQTT_Publish_T MqttPublishLight =
        {
                .Topic = MQTT_TOPIC_LIGHT,
                .QoS = 1UL,
                .Payload = NULL,
                .PayloadLength = 0UL,
        };/**< MQTT publish parameters */

static MQTT_Publish_T MqttPublishNoise =
        {
                .Topic = MQTT_TOPIC_NOISE,
                .QoS = 1UL,
                .Payload = NULL,
                .PayloadLength = 0UL,
        };/**< MQTT publish parameters */



static uint32_t AppIncomingMsgCount = 0UL;/**< Incoming message count */

static char AppIncomingMsgTopicBuffer[APP_MQTT_DATA_BUFFER_SIZE];/**< Incoming message topic buffer */

static char AppIncomingMsgPayloadBuffer[APP_MQTT_DATA_BUFFER_SIZE];/**< Incoming message payload buffer */

static CmdProcessor_T * AppCmdProcessor;/**< Handle to store the main Command processor handle to be used by run-time event driven threads */

static xTaskHandle AppControllerHandle = NULL;/**< OS thread handle for Application controller to be used by run-time blocking threads */

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

static void AppMQTTSubscribeCB(MQTT_SubscribeCBParam_T param)
{
    AppIncomingMsgCount++;
    memset(AppIncomingMsgTopicBuffer, 0, sizeof(AppIncomingMsgTopicBuffer));
    memset(AppIncomingMsgPayloadBuffer, 0, sizeof(AppIncomingMsgPayloadBuffer));

    if (param.PayloadLength < sizeof(AppIncomingMsgPayloadBuffer))
    {
        strncpy(AppIncomingMsgPayloadBuffer, (const char *) param.Payload, param.PayloadLength);
    }
    else
    {
        strncpy(AppIncomingMsgPayloadBuffer, (const char *) param.Payload, (sizeof(AppIncomingMsgPayloadBuffer) - 1U));
    }
    if (param.TopicLength < (int) sizeof(AppIncomingMsgTopicBuffer))
    {
        strncpy(AppIncomingMsgTopicBuffer, param.Topic, param.TopicLength);
    }
    else
    {
        strncpy(AppIncomingMsgTopicBuffer, param.Topic, (sizeof(AppIncomingMsgTopicBuffer) - 1U));
    }

    printf("AppMQTTSubscribeCB : #%d, Incoming Message:\r\n"
            "\tTopic: %s\r\n"
            "\tPayload: \r\n\"\"\"\r\n%s\r\n\"\"\"\r\n", (int) AppIncomingMsgCount,
            AppIncomingMsgTopicBuffer, AppIncomingMsgPayloadBuffer);

    if (strcmp(AppIncomingMsgPayloadBuffer, "1") == 0) {
    	LED_On(LED_INBUILT_RED);
    	LED_On(LED_INBUILT_ORANGE);
    	LED_On(LED_INBUILT_YELLOW);
    }else if(strcmp(AppIncomingMsgPayloadBuffer, "0") == 0){
    	LED_Off(LED_INBUILT_RED);
    	LED_Off(LED_INBUILT_ORANGE);
    	LED_Off(LED_INBUILT_YELLOW);
    }

}

/**
 * @brief This will validate the WLAN network connectivity
 *
 * Currently, upon abrupt disconnection the event from the WiFi chip is not
 * propagated / notified to the application. Until then, we manually validate
 * prior to ever HTTP Rest Client POST/GET cycle to make sure that the possibility
 * of software break is reduced. We check if the IP is valid to validate the same.
 *
 * If there is no connectivity we will restart the node after 10 seconds.
 */
static void AppControllerValidateWLANConnectivity(void)
{
    Retcode_T retcode = RETCODE_OK;
    NetworkConfig_IpStatus_T ipStatus = NETWORKCONFIG_IP_NOT_ACQUIRED;
    NetworkConfig_IpSettings_T ipAddressOnGetStatus;

    ipStatus = NetworkConfig_GetIpStatus();
    if (ipStatus == NETWORKCONFIG_IPV4_ACQUIRED)
    {
        retcode = NetworkConfig_GetIpSettings(&ipAddressOnGetStatus);
        if ((RETCODE_OK == retcode) && (UINT32_C(0) == (ipAddressOnGetStatus.ipV4)))
        {
            /* Our IP configuration is corrupted somehow in this case. No use in proceeding further. */
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NODE_IPV4_IS_CORRUPTED);
        }
    }
    else
    {
        /* Our network connection is lost. No use in proceeding further. */
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NODE_WLAN_CONNECTION_IS_LOST);
    }
    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        printf("AppControllerValidateWLANConnectivity : Resetting the device. Check if network is available. Node will do a soft reset in 10 seconds.\r\n\r\n");
        vTaskDelay(pdMS_TO_TICKS(10000));
        BSP_Board_SoftReset();
        assert(false); /* Code must not reach here */
    }
}

/**
 * @brief Responsible for controlling the send data over MQTT application control flow.
 *
 * - Synchronize SNTP time stamp for the system if MQTT communication is secure
 * - Connect to MQTT broker
 * - Subscribe to MQTT topic
 * - Read environmental sensor data
 * - Publish data periodically for a MQTT topic
 *
 * @param[in] pvParameters
 * Unused
 */
static void AppControllerFire(void* pvParameters)
{
    BCDS_UNUSED(pvParameters);

    Retcode_T retcode = RETCODE_OK;
    Sensor_Value_T sensorValue;
    char publishBufferH[APP_MQTT_DATA_BUFFER_SIZE];
    char publishBufferP[APP_MQTT_DATA_BUFFER_SIZE];
    char publishBufferT[APP_MQTT_DATA_BUFFER_SIZE];
    char publishBufferL[APP_MQTT_DATA_BUFFER_SIZE];
    char publishBufferN[APP_MQTT_DATA_BUFFER_SIZE];


    const char *publishHumidity    = "%ld";
    const char *publishPressure    = "%ld";
    const char *publishTemperature = "%f";
    const char *publishLigth	   = "%ld";
    const char *publishSound	   = "%f";

    memset(&sensorValue, 0x00, sizeof(sensorValue));
#if APP_MQTT_SECURE_ENABLE

    uint64_t sntpTimeStampFromServer = 0UL;

    /* We Synchronize the node with the SNTP server for time-stamp.
     * Since there is no point in doing a HTTPS communication without a valid time */
    do
    {
        retcode = SNTP_GetTimeFromServer(&sntpTimeStampFromServer, APP_RESPONSE_FROM_SNTP_SERVER_TIMEOUT);
        if ((RETCODE_OK != retcode) || (0UL == sntpTimeStampFromServer))
        {
            printf("AppControllerFire : SNTP server time was not synchronized. Retrying...\r\n");
        }
    }while (0UL == sntpTimeStampFromServer);

    BCDS_UNUSED(sntpTimeStampFromServer); /* Copy of sntpTimeStampFromServer will be used be HTTPS for TLS handshake */
#endif /* APP_MQTT_SECURE_ENABLE */

    if (RETCODE_OK == retcode)
    {
        retcode = MQTT_ConnectToBroker(&MqttConnectInfo, MQTT_CONNECT_TIMEOUT_IN_MS);
    }

    if (RETCODE_OK == retcode)
    {
        retcode = MQTT_SubsribeToTopic(&MqttSubscribeInfo, MQTT_SUBSCRIBE_TIMEOUT_IN_MS);
    }

    if (RETCODE_OK != retcode)
    {
        /* We raise error and still proceed to publish data periodically */
        Retcode_RaiseError(retcode);
    }

    /* A function that implements a task must not exit or attempt to return to
     its caller function as there is nothing to return to. */
    while (1)
    {
        /* Resetting / clearing the necessary buffers / variables for re-use */
        retcode = RETCODE_OK;

        /* Check whether the WLAN network connection is available */
        AppControllerValidateWLANConnectivity();

        retcode = Sensor_GetData(&sensorValue);
        if (RETCODE_OK == retcode)
        {
            int32_t lengthH = snprintf((char *) publishBufferH, APP_MQTT_DATA_BUFFER_SIZE, publishHumidity,
                    (long int) sensorValue.RH);

            int32_t lengthP = snprintf((char *) publishBufferP, APP_MQTT_DATA_BUFFER_SIZE, publishPressure,
                                (long int) sensorValue.Pressure);

            int32_t lengthT = snprintf((char *) publishBufferT, APP_MQTT_DATA_BUFFER_SIZE, publishTemperature,
                                (sensorValue.Temp /= 1000));

            int32_t lengthL = snprintf((char *) publishBufferL, APP_MQTT_DATA_BUFFER_SIZE, publishLigth,
                                (long int) sensorValue.Light);

            int32_t lengthN = snprintf((char *) publishBufferN, APP_MQTT_DATA_BUFFER_SIZE, publishSound,
                                sensorValue.Noise);

            //Send Humidity
            MqttPublishHumidity.Payload = publishBufferH;
            MqttPublishHumidity.PayloadLength = lengthH;
            retcode = MQTT_PublishToTopic(&MqttPublishHumidity, MQTT_PUBLISH_TIMEOUT_IN_MS);

            //Publish Pressure
            MqttPublishPressure.Payload = publishBufferP;
            MqttPublishPressure.PayloadLength = lengthP;
            retcode = MQTT_PublishToTopic(&MqttPublishPressure, MQTT_PUBLISH_TIMEOUT_IN_MS);

            //Publish Temperature
            MqttPublishTemperature.Payload = publishBufferT;
            MqttPublishTemperature.PayloadLength = lengthT;
            retcode = MQTT_PublishToTopic(&MqttPublishTemperature, MQTT_PUBLISH_TIMEOUT_IN_MS);

            //Publish Light
            MqttPublishLight.Payload = publishBufferL;
            MqttPublishLight.PayloadLength = lengthL;
            retcode = MQTT_PublishToTopic(&MqttPublishLight, MQTT_PUBLISH_TIMEOUT_IN_MS);

            //Publish Light
            MqttPublishNoise.Payload = publishBufferN;
            MqttPublishNoise.PayloadLength = lengthN;
            retcode = MQTT_PublishToTopic(&MqttPublishNoise, MQTT_PUBLISH_TIMEOUT_IN_MS);

        }
        if (RETCODE_OK != retcode)
        {
            Retcode_RaiseError(retcode);
        }
        vTaskDelay(APP_MQTT_DATA_PUBLISH_PERIODICITY);
    }
}

/**
 * @brief To enable the necessary modules for the application
 * - WLAN
 * - ServalPAL
 * - SNTP (if secure communication)
 * - MQTT
 * - Sensor
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerEnable(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    Retcode_T retcode = WLAN_Enable();
    if (RETCODE_OK == retcode)
    {
        retcode = ServalPAL_Enable();
    }
#if APP_MQTT_SECURE_ENABLE
    if (RETCODE_OK == retcode)
    {
        retcode = SNTP_Enable();
    }
#endif /* APP_MQTT_SECURE_ENABLE */
    if (RETCODE_OK == retcode)
    {
        retcode = MQTT_Enable();
    }
    if (RETCODE_OK == retcode)
    {
        retcode = Sensor_Enable();
    }
    if (RETCODE_OK == retcode)
    {
        if (pdPASS != xTaskCreate(AppControllerFire, (const char * const ) "AppController", TASK_STACK_SIZE_APP_CONTROLLER, NULL, TASK_PRIO_APP_CONTROLLER, &AppControllerHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
        }
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerEnable : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/**
 * @brief To setup the necessary modules for the application
 * - WLAN
 * - ServalPAL
 * - SNTP (if secure communication)
 * - MQTT
 * - Sensor
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerSetup(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    Retcode_T retcode = WLAN_Setup(&WLANSetupInfo);
    if (RETCODE_OK == retcode)
    {
        retcode = ServalPAL_Setup(AppCmdProcessor);
    }
#if APP_MQTT_SECURE_ENABLE
    if (RETCODE_OK == retcode)
    {
        retcode = SNTP_Setup(&SNTPSetupInfo);
    }
#endif /* APP_MQTT_SECURE_ENABLE */
    if (RETCODE_OK == retcode)
    {
        retcode = MQTT_Setup(&MqttSetupInfo);
    }
    if (RETCODE_OK == retcode)
    {
        SensorSetup.CmdProcessorHandle = AppCmdProcessor;
        retcode = Sensor_Setup(&SensorSetup);
    }
    if (RETCODE_OK == retcode)
    {
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerEnable, NULL, UINT32_C(0));
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerSetup : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/* global functions ********************************************************* */

/** Refer interface header for description */
void AppController_Init(void * cmdProcessorHandle, uint32_t param2)
{
    BCDS_UNUSED(param2);

    Retcode_T retcode = RETCODE_OK;

    if (cmdProcessorHandle == NULL)
    {
        printf("AppController_Init : Command processor handle is NULL \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        AppCmdProcessor = (CmdProcessor_T *) cmdProcessorHandle;
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerSetup, NULL, UINT32_C(0));
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/**@} */
/** ************************************************************************* */
