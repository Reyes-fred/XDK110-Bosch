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
 * @ingroup CONNECTIVITY
 *
 * @defgroup MQTT MQTT
 * @{
 *
 * @brief This module handles the MQTT communication.
 *
 * @file
 */

/* header definition ******************************************************** */
#ifndef XDK_MQTT_H_
#define XDK_MQTT_H_

/* local interface declaration ********************************************** */
#include "BCDS_Retcode.h"
#include "BCDS_CmdProcessor.h"

/**
 * @brief   Enum to represent the supported MQTT types.
 */
enum MQTT_Type_E
{
    MQTT_TYPE_SERVALSTACK,
    MQTT_TYPE_AWS
};

/**
 * @brief   Typedef to represent the supported MQTT type.
 */
typedef enum MQTT_Type_E MQTT_Type_T;

/**
 * @brief   Structure to represent the MQTT setup features.
 */
struct MQTT_Setup_S
{
    CmdProcessor_T * CmdProcessorHandle; /**< Command processor handle to handle MQTT agent events and servicing. */
    MQTT_Type_T MqttType; /**< The MQTT type */
    bool IsSecure; /**< Boolean representing if we will do a HTTP secure communication. Unused if MqttType is MQTT_TYPE_AWS. */
};

/**
 * @brief   Typedef to represent the MQTT setup feature.
 */
typedef struct MQTT_Setup_S MQTT_Setup_T;

/**
 * @brief   Structure to represent the MQTT connect features.
 */
struct MQTT_Connect_S
{
	const char * User;
	const char * Password;
    const char * ClientId; /**< The client identifier which is a identifier of each MQTT client connecting to a MQTT broker. It needs to be unique for the broker to know the state of the client. */
    const char * BrokerURL; /**< The URL pointing to the MQTT broker */
    uint16_t BrokerPort; /**< The port number of the MQTT broker */
    bool CleanSession; /**< The clean session flag indicates to the broker whether the client wants to establish a clean session or a persistent session where all subscriptions and messages (QoS 1 & 2) are stored for the client. */
    uint32_t KeepAliveInterval; /**< The keep alive interval (in seconds) is the time the client commits to for when sending regular pings to the broker. The broker responds to the pings enabling both sides to determine if the other one is still alive and reachable */
};

/**
 * @brief   Typedef to represent the MQTT connect feature.
 */
typedef struct MQTT_Connect_S MQTT_Connect_T;

/**
 * @brief   Structure to represent the MQTT publish features.
 */
struct MQTT_Publish_S
{
    const char * Topic; /**< The MQTT topic to which the messages are to be published */
    uint32_t QoS; /**< The MQTT Quality of Service level. If 0, the message is send in a fire and forget way and it will arrive at most once. If 1 Message reception is acknowledged by the other side, retransmission could occur. */
    const char * Payload; /**< Pointer to the payload to be published */
    uint32_t PayloadLength; /**< Length of the payload to be published */
};

/**
 * @brief   Typedef to represent the MQTT publish feature.
 */
typedef struct MQTT_Publish_S MQTT_Publish_T;

/**
 * @brief   Structure to represent the incoming MQTT message informations.
 */
struct MQTT_SubscribeCBParam_S
{
    const char * Topic; /**< The incoming MQTT topic pointer */
    uint32_t TopicLength; /**< The incoming MQTT topic length */
    const char * Payload; /**< The incoming MQTT payload pointer */
    uint32_t PayloadLength; /**< The incoming MQTT payload length */
};

/**
 * @brief   Typedef to represent the incoming MQTT message information.
 */
typedef struct MQTT_SubscribeCBParam_S MQTT_SubscribeCBParam_T;

/**
 * @brief   Typedef to the function to be called upon receiving incoming MQTT messages.
 */
typedef void (*MQTT_SubscribeCB_T)(MQTT_SubscribeCBParam_T param);

/**
 * @brief   Structure to represent the MQTT subscribe features.
 */
struct MQTT_Subscribe_S
{
    const char * Topic; /**< The MQTT topic for which the messages are to be subscribed */
    uint32_t QoS; /**< The MQTT Quality of Service level. If 0, the message is send in a fire and forget way and it will arrive at most once. If 1 Message reception is acknowledged by the other side, retransmission could occur. */
    MQTT_SubscribeCB_T IncomingPublishNotificationCB; /**< The function to be called upon receiving incoming MQTT messages. Can be NULL. */
};

/**
 * @brief   Structure to represent the MQTT subscribe feature.
 */
typedef struct MQTT_Subscribe_S MQTT_Subscribe_T;

/**
 * @brief This will setup the MQTT
 *
 * @param[in] setup
 * Pointer to the MQTT setup feature
 *
 * @return  RETCODE_OK on success, or an error code otherwise.
 *
 * @note
 * - This must be the first API to be called if MQTT feature is to be used.
 * - Do not call this API more than once.
 */
Retcode_T MQTT_Setup(MQTT_Setup_T * setup);

/**
 * @brief This will enable the MQTT by connecting to the broker
 *
 * @return  RETCODE_OK on success, or an error code otherwise.
 *
 * @note
 * - If setup->IsSecure was enabled at the time of #MQTT_Setup,
 *   then we will enable the HTTPs server certificates as well.
 * - #MQTT_Setup must have been successful prior.
 * - #WLAN_Enable must have been successful prior.
 * - Do not call this API more than once.
 */
Retcode_T MQTT_Enable(void);

/**
 * @brief This will connect to a MQTT broker
 *
 * @param[in] connect
 * Pointer to the MQTT connect feature
 *
 * @param[in] timeout
 * Timeout in milli-second to await successful connection
 *
 * @return  RETCODE_OK on success, or an error code otherwise.
 */
Retcode_T MQTT_ConnectToBroker(MQTT_Connect_T * connect, uint32_t timeout);

/**
 * @brief This will subscribe to a MQTT topic
 *
 * @param[in] subscribe
 * Pointer to the MQTT subscribe feature
 *
 * @param[in] timeout
 * Timeout in milli-second to await successful subscription
 *
 * @return  RETCODE_OK on success, or an error code otherwise.
 */
Retcode_T MQTT_SubsribeToTopic(MQTT_Subscribe_T * subscribe, uint32_t timeout);

/**
 * @brief This will publish to a MQTT topic
 *
 * @param[in] subscribe
 * Pointer to the MQTT publish feature
 *
 * @param[in] timeout
 * Timeout in milli-second to await successful publication
 *
 * @return  RETCODE_OK on success, or an error code otherwise.
 */
Retcode_T MQTT_PublishToTopic(MQTT_Publish_T * publish, uint32_t timeout);

#endif /* XDK_MQTT_H_ */

/**@} */
