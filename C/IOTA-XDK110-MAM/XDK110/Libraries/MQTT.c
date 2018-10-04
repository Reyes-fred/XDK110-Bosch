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
 * @file
 *
 * This module handles the MQTT communication
 *
 */

/* module includes ********************************************************** */

/* own header files */
#include "XdkCommonInfo.h"

#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_COMMON_ID_MQTT

/* own header files */
#include "XDK_MQTT.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "aws_mqtt_agent.h"
#include "aws_bufferpool.h"
#include "aws_secure_sockets.h"
#include "HTTPRestClientSecurity.h"
#include "BCDS_NetworkConfig.h"
#include "Serval_Msg.h"
#include "Serval_Http.h"
#include "Serval_HttpClient.h"
#include "Serval_Types.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Serval_Mqtt.h"

/* constant definitions ***************************************************** */

/**<  Macro for the number of topics to subscribe */
#define MQTT_SUBSCRIBE_COUNT                1UL

/**<  Macro for the non secure serval stack expected MQTT URL format */
#define MQTT_URL_FORMAT_NON_SECURE          "mqtt://%s:%d"

/**<  Macro for the secure serval stack expected MQTT URL format */
#define MQTT_URL_FORMAT_SECURE              "mqtts://%s:%d"

/* local variables ********************************************************** */

/**< Handle for MQTT subscribe operation  */
static SemaphoreHandle_t MqttSubscribeHandle;
/**< Handle for MQTT publish operation  */
static SemaphoreHandle_t MqttPublishHandle;
/**< Handle for MQTT send operation  */
static SemaphoreHandle_t MqttSendHandle;
/**< Handle for MQTT send operation  */
static SemaphoreHandle_t MqttConnectHandle;
/**< MQTT setup information */
static MQTT_Setup_T MqttSetupInfo;
/**< MQTT incoming publish notification callback for the application */
static MQTT_SubscribeCB_T IncomingPublishNotificationCB;
/**< MQTT session instance */
static MqttSession_T MqttSession;
/**< MQTT connection status */
static bool MqttConnectionStatus = false;
/**< MQTT subscription status */
static bool MqttSubscriptionStatus = false;
/**< MQTT publish status */
static bool MqttPublishStatus = false;
static MQTTAgentHandle_t mqttAgentHandle = NULL;

/**
 * @brief Event handler for incoming publish MQTT data
 *
 * @param[in] publishData
 * Event Data for publish
 */
static void HandleEventIncomingPublish(MqttPublishData_T publishData)
{
    if (NULL != IncomingPublishNotificationCB)
    {
        MQTT_SubscribeCBParam_T param =
                {
                        .Topic = publishData.topic.start,
                        .TopicLength = publishData.topic.length,
                        .Payload = (const char *) publishData.payload,
                        .PayloadLength = publishData.length
                };
        IncomingPublishNotificationCB(param);
    }
}

/**
 * @brief Callback function used by the stack to communicate events to the application.
 * Each event will bring with it specialized data that will contain more information.
 *
 * @param[in] session
 * MQTT session
 *
 * @param[in] event
 * MQTT event
 *
 * @param[in] eventData
 * MQTT data based on the event
 *
 */
static retcode_t MqttEventHandler(MqttSession_T* session, MqttEvent_t event, const MqttEventData_t* eventData)
{
    BCDS_UNUSED(session);
    Retcode_T retcode = RETCODE_OK;
    printf("MqttEventHandler : Event - %d\r\n", (int) event);
    switch (event)
    {
    case MQTT_CONNECTION_ESTABLISHED:
        MqttConnectionStatus = true;
        if (pdTRUE != xSemaphoreGive(MqttConnectHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SEMAPHORE_ERROR);
        }
        break;
    case MQTT_CONNECTION_ERROR:
        case MQTT_CONNECT_SEND_FAILED:
        case MQTT_CONNECT_TIMEOUT:
        MqttConnectionStatus = false;
        if (pdTRUE != xSemaphoreGive(MqttConnectHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SEMAPHORE_ERROR);
        }

        break;
    case MQTT_CONNECTION_CLOSED:
        MqttConnectionStatus = false;
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_CONNECTION_CLOSED);
        break;
    case MQTT_SUBSCRIPTION_ACKNOWLEDGED:
        MqttSubscriptionStatus = true;
        if (pdTRUE != xSemaphoreGive(MqttSubscribeHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SEMAPHORE_ERROR);
        }
        break;
    case MQTT_SUBSCRIBE_SEND_FAILED:
        case MQTT_SUBSCRIBE_TIMEOUT:
        MqttSubscriptionStatus = false;
        if (pdTRUE != xSemaphoreGive(MqttSubscribeHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SEMAPHORE_ERROR);
        }
        break;
    case MQTT_SUBSCRIPTION_REMOVED:
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_REMOVED);
        break;
    case MQTT_INCOMING_PUBLISH:
        HandleEventIncomingPublish(eventData->publish);
        break;
    case MQTT_PUBLISHED_DATA:
        MqttPublishStatus = true;
        if (pdTRUE != xSemaphoreGive(MqttPublishHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SEMAPHORE_ERROR);
        }
        break;
    case MQTT_PUBLISH_SEND_FAILED:
        case MQTT_PUBLISH_SEND_ACK_FAILED:
        case MQTT_PUBLISH_TIMEOUT:
        MqttPublishStatus = false;
        if (pdTRUE != xSemaphoreGive(MqttPublishHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SEMAPHORE_ERROR);
        }
        break;
    default:
        printf("MqttEventHandler : Unhandled MQTT Event\r\n");
        break;
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
    }

    return RC_OK;
}

#ifdef mqttconfigENABLE_SUBSCRIPTION_MANAGEMENT

static MQTTBool_t MqttAgentSubscribeCallback(void * pvPublishCallbackContext, const MQTTPublishData_t * const pxPublishData)
{
    if ((NULL != IncomingPublishNotificationCB) &&
            (NULL != pvPublishCallbackContext) &&
            (NULL != pxPublishData))
    {
        MQTT_SubscribeCBParam_T param =
        {
            .Topic = (const char *) pxPublishData->pucTopic,
            .TopicLength = pxPublishData->usTopicLength,
            .Payload = (const char *) pxPublishData->pvData,
            .PayloadLength = pxPublishData->ulDataLength
        };
        IncomingPublishNotificationCB(param);
    }

    return eMQTTFalse; /* Returning eMQTTFalse will free the buffer */
}

#endif /* mqttconfigENABLE_SUBSCRIPTION_MANAGEMENT */

/** Refer interface header for description */
Retcode_T MQTT_Setup(MQTT_Setup_T * setup)
{
    Retcode_T retcode = RETCODE_OK;
    if (NULL == setup)
    {
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        switch (setup->MqttType)
        {
        case MQTT_TYPE_SERVALSTACK:
            if (setup->IsSecure)
            {
#if SERVAL_ENABLE_TLS_CLIENT
                retcode = HTTPRestClientSecurity_Setup();
#else /* SERVAL_ENABLE_TLS_CLIENT */
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_HTTP_ENABLE_SERVAL_TLS_CLIENT);
#endif /* SERVAL_ENABLE_TLS_CLIENT */
            }
            if (RETCODE_OK == retcode)
            {
                MqttSubscribeHandle = xSemaphoreCreateBinary();
                if (NULL == MqttSubscribeHandle)
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                }
            }
            if (RETCODE_OK == retcode)
            {
                MqttPublishHandle = xSemaphoreCreateBinary();
                if (NULL == MqttPublishHandle)
                {
                    vSemaphoreDelete(MqttSubscribeHandle);
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                }
            }
            if (RETCODE_OK == retcode)
            {
                MqttSendHandle = xSemaphoreCreateBinary();
                if (NULL == MqttSendHandle)
                {
                    vSemaphoreDelete(MqttSubscribeHandle);
                    vSemaphoreDelete(MqttPublishHandle);
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                }
            }
            if (RETCODE_OK == retcode)
            {
                MqttConnectHandle = xSemaphoreCreateBinary();
                if (NULL == MqttConnectHandle)
                {
                    vSemaphoreDelete(MqttSubscribeHandle);
                    vSemaphoreDelete(MqttPublishHandle);
                    vSemaphoreDelete(MqttSendHandle);
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                }
            }
            break;
        case MQTT_TYPE_AWS:

            if (pdPASS != BUFFERPOOL_Init())
            {
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
            }
            if (RETCODE_OK == retcode)
            {
                if (pdPASS != MQTT_AGENT_Init())
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                }
            }
            if (RETCODE_OK == retcode)
            {
                if (pdPASS != SOCKETS_Init())
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                }
            }
            if (RETCODE_OK == retcode)
            {
                if (eMQTTAgentSuccess != MQTT_AGENT_Create(&mqttAgentHandle))
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                }
            }

            break;
        default:
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
            break;
        }

        if (RETCODE_OK == retcode)
        {
            MqttSetupInfo = *setup;
        }
    }
    return retcode;
}

/** Refer interface header for description */
Retcode_T MQTT_Enable(void)
{
    Retcode_T retcode = RETCODE_OK;

    switch (MqttSetupInfo.MqttType)
    {
    case MQTT_TYPE_SERVALSTACK:
        if (MqttSetupInfo.IsSecure)
        {
#if SERVAL_ENABLE_TLS_CLIENT
            retcode = HTTPRestClientSecurity_Enable();
#else /* SERVAL_ENABLE_TLS_CLIENT */
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_HTTP_ENABLE_SERVAL_TLS_CLIENT);
#endif /* SERVAL_ENABLE_TLS_CLIENT */
        }
        break;
    case MQTT_TYPE_AWS:
        /* Do Nothing, yet */
        break;
    default:
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
        break;
    }

    return retcode;
}

/** Refer interface header for description */
Retcode_T MQTT_ConnectToBroker(MQTT_Connect_T * connect, uint32_t timeout)
{
    Retcode_T retcode = RETCODE_OK;

    if (NULL == connect)
    {
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        switch (MqttSetupInfo.MqttType)
        {
        case MQTT_TYPE_SERVALSTACK:
            {
            Ip_Address_T brokerIpAddress = 0UL;
            StringDescr_T clientID;
            StringDescr_T Username;
            StringDescr_T Password;
            char mqttBrokerURL[30] = { 0 };
            char serverIpStringBuffer[16] = { 0 };

            if (RC_OK != Mqtt_initialize())
            {
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_INIT_FAILED);
            }

            if (RETCODE_OK == retcode)
            {
                if (RC_OK != Mqtt_initializeInternalSession(&MqttSession))
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_INIT_INTERNAL_SESSION_FAILED);
                }
            }

            if (RETCODE_OK == retcode)
            {
                retcode = NetworkConfig_GetIpAddress((uint8_t *) connect->BrokerURL, &brokerIpAddress);
            }
            if (RETCODE_OK == retcode)
            {
                if (0 > Ip_convertAddrToString(&brokerIpAddress, serverIpStringBuffer))
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_IPCONIG_FAIL);
                }
            }
            if (RETCODE_OK == retcode)
            {
                MqttSession.MQTTVersion = 3;
                MqttSession.keepAliveInterval = connect->KeepAliveInterval;
                MqttSession.cleanSession = connect->CleanSession;
                MqttSession.will.haveWill = false;
                MqttSession.onMqttEvent = MqttEventHandler;

                StringDescr_wrap(&Username, connect->User);
                MqttSession.username = Username;
                StringDescr_wrap(&Password, connect->Password);
               	MqttSession.password = Password;

                StringDescr_wrap(&clientID, connect->ClientId);
                MqttSession.clientID = clientID;

                if (MqttSetupInfo.IsSecure)
                {
                    sprintf(mqttBrokerURL, MQTT_URL_FORMAT_SECURE, serverIpStringBuffer, connect->BrokerPort);
                    MqttSession.target.scheme = SERVAL_SCHEME_MQTTS;
                }
                else
                {
                    sprintf(mqttBrokerURL, MQTT_URL_FORMAT_NON_SECURE, serverIpStringBuffer, connect->BrokerPort);
                    MqttSession.target.scheme = SERVAL_SCHEME_MQTT;
                }

                if (RC_OK == SupportedUrl_fromString((const char *) mqttBrokerURL, (uint16_t) strlen((const char *) mqttBrokerURL), &MqttSession.target))
                {
                    MqttConnectionStatus = false;
                    /* This is a dummy take. In case of any callback received
                     * after the previous timeout will be cleared here. */
                    (void) xSemaphoreTake(MqttConnectHandle, 0UL);
                    if (RC_OK != Mqtt_connect(&MqttSession))
                    {
                        printf("MQTT_Enable : Failed to connect MQTT \r\n");
                        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_CONNECT_FAILED);
                    }
                }
                else
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_PARSING_ERROR);
                }
            }
            if (RETCODE_OK == retcode)
            {
                if (pdTRUE != xSemaphoreTake(MqttConnectHandle, pdMS_TO_TICKS(timeout)))
                {
                    printf("MQTT_Enable : Failed since Post CB was not received \r\n");
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_CONNECT_CB_NOT_RECEIVED);
                }
                else
                {
                    if (true != MqttConnectionStatus)
                    {
                        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_CONNECT_STATUS_ERROR);
                    }
                }
            }
        }
            break;
        case MQTT_TYPE_AWS:
            {
            MQTTAgentConnectParams_t mqttAgentConnectParams =
                    {
                            .pcURL = connect->BrokerURL, /* The URL of the MQTT broker to connect to. */
                            .xFlags = mqttagentREQUIRE_TLS, /* Connection flags. */
                            .xURLIsIPAddress = pdFALSE, /* Deprecated. */
                            .usPort = connect->BrokerPort, /* Port number on which the MQTT broker is listening. */
                            .pucClientId = (const uint8_t *) connect->ClientId, /* Client Identifier of the MQTT client. It should be unique per broker. */
                            .usClientIdLength = 0, /* The length of the client Id, filled in later as not const. */
                            .xSecuredConnection = pdFALSE, /* Deprecated. */
                            .pvUserData = NULL, /* User data supplied to the callback. Can be NULL. */
                            .pxCallback = NULL, /* Callback used to report various events. Can be NULL. */
                            .pcCertificate = NULL, /* Certificate used for secure connection. Can be NULL. */
                            .ulCertificateSize = 0 /* Size of certificate used for secure connection. */
                    };
            mqttAgentConnectParams.usClientIdLength = (uint16_t) strlen(connect->ClientId);

            if (eMQTTAgentSuccess != MQTT_AGENT_Connect(mqttAgentHandle, &mqttAgentConnectParams, timeout))
            {
                (void) MQTT_AGENT_Delete(mqttAgentHandle);
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_CONNECT_FAILED);
            }
        }
            break;
        default:
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
            break;
        }
    }
    return retcode;
}

/** Refer interface header for description */
Retcode_T MQTT_SubsribeToTopic(MQTT_Subscribe_T * subscribe, uint32_t timeout)
{
    Retcode_T retcode = RETCODE_OK;

    if (NULL == subscribe)
    {
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        switch (MqttSetupInfo.MqttType)
        {
        case MQTT_TYPE_SERVALSTACK:
            {
            static StringDescr_T subscribeTopicDescription[MQTT_SUBSCRIBE_COUNT];
            Mqtt_qos_t qos[MQTT_SUBSCRIBE_COUNT];

            StringDescr_wrap(&(subscribeTopicDescription[0]), subscribe->Topic);
            qos[0] = (Mqtt_qos_t) subscribe->QoS;

            printf("MQTT_Subsribe : Subscribing to topic: %s, Qos: %d\r\n", subscribe->Topic, qos[0]);
            IncomingPublishNotificationCB = subscribe->IncomingPublishNotificationCB;
            MqttSubscriptionStatus = false;
            /* This is a dummy take. In case of any callback received
             * after the previous timeout will be cleared here. */
            (void) xSemaphoreTake(MqttSubscribeHandle, 0UL);
            if (RC_OK != Mqtt_subscribe(&MqttSession, MQTT_SUBSCRIBE_COUNT, subscribeTopicDescription, qos))
            {
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_FAILED);
            }

            if (RETCODE_OK == retcode)
            {
                if (pdTRUE != xSemaphoreTake(MqttSubscribeHandle, pdMS_TO_TICKS(timeout)))
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_CB_NOT_RECEIVED);
                }
                else
                {
                    if (true != MqttSubscriptionStatus)
                    {
                        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_STATUS_ERROR);
                    }
                }
            }
        }
            break;
        case MQTT_TYPE_AWS:
            {
            if ((MQTTQoS_t) subscribe->QoS <= eMQTTQoS2)
            {
                MQTTAgentSubscribeParams_t mqttSubscription =
                        {
                                .pucTopic = (const uint8_t*) subscribe->Topic,
                                .usTopicLength = 0,
                                .xQoS = (MQTTQoS_t) subscribe->QoS,
                                .pvPublishCallbackContext = (void*) MqttSetupInfo.CmdProcessorHandle,
                                .pxPublishCallback = MqttAgentSubscribeCallback
                        };
                mqttSubscription.usTopicLength = strlen(subscribe->Topic);
                IncomingPublishNotificationCB = subscribe->IncomingPublishNotificationCB;

                /* MQTT_AGENT_Subscribe is a blocking call */
                if (eMQTTAgentSuccess != MQTT_AGENT_Subscribe(mqttAgentHandle, &mqttSubscription, timeout))
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_FAILED);
                }
            }
            else
            {
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
            }
        }
            break;
        default:
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
            break;
        }

    }
    return retcode;
}

/** Refer interface header for description */
Retcode_T MQTT_PublishToTopic(MQTT_Publish_T * publish, uint32_t timeout)
{
    Retcode_T retcode = RETCODE_OK;

    if (NULL == publish)
    {
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {

        switch (MqttSetupInfo.MqttType)
        {
        case MQTT_TYPE_SERVALSTACK:
            {
            static StringDescr_T publishTopicDescription;
            StringDescr_wrap(&publishTopicDescription, publish->Topic);

            MqttPublishStatus = false;
            /* This is a dummy take. In case of any callback received
             * after the previous timeout will be cleared here. */
            (void) xSemaphoreTake(MqttPublishHandle, 0UL);
            if (RC_OK != Mqtt_publish(&MqttSession, publishTopicDescription, publish->Payload, publish->PayloadLength, (uint8_t) publish->QoS, false))
            {
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_PUBLISH_FAILED);
            }
            if (RETCODE_OK == retcode)
            {
                if (pdTRUE != xSemaphoreTake(MqttPublishHandle, pdMS_TO_TICKS(timeout)))
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_CB_NOT_RECEIVED);
                }
                else
                {
                    if (true != MqttPublishStatus)
                    {
                        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_STATUS_ERROR);
                    }
                }
            }
        }
            break;

        case MQTT_TYPE_AWS:
            {
            if ((MQTTQoS_t) publish->QoS <= eMQTTQoS2)
            {
                MQTTAgentPublishParams_t mqttPublishMessage =
                        {
                                .pucTopic = (const uint8_t *) publish->Topic,
                                .usTopicLength = 0,
                                .xQoS = (MQTTQoS_t) publish->QoS,
                                .pvData = publish->Payload,
                                .ulDataLength = publish->PayloadLength,
                        };
                mqttPublishMessage.usTopicLength = strlen(publish->Topic);
                /* MQTT_AGENT_Publish is a blocking call */
                if (eMQTTAgentSuccess != MQTT_AGENT_Publish(mqttAgentHandle, &mqttPublishMessage, timeout))
                {
                    retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_FAILED);
                }
            }
            else
            {
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
            }
        }
            break;

        default:
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
            break;
        }
    }

    return retcode;
}
