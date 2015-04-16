// MESSAGE AQ_ROBOTMSG PACKING

#define MAVLINK_MSG_ID_AQ_ROBOTMSG 152

typedef struct __mavlink_aq_robotmsg_t
{
 float x; ///< value1
 float y; ///< value2
 float z; ///< value3
 float heading; ///< value4
 float ID; ///< value5
 uint16_t Index; ///< Index of message
 uint16_t cmd; ///< value6
} mavlink_aq_robotmsg_t;

#define MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN 24
#define MAVLINK_MSG_ID_152_LEN 24

#define MAVLINK_MSG_ID_AQ_ROBOTMSG_CRC 0
#define MAVLINK_MSG_ID_152_CRC 0



#define MAVLINK_MESSAGE_INFO_AQ_ROBOTMSG { \
	"AQ_ROBOTMSG", \
	7, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_aq_robotmsg_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_aq_robotmsg_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_aq_robotmsg_t, z) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_aq_robotmsg_t, heading) }, \
         { "ID", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_aq_robotmsg_t, ID) }, \
         { "Index", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_aq_robotmsg_t, Index) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_aq_robotmsg_t, cmd) }, \
         } \
}


/**
 * @brief Pack a aq_robotmsg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Index Index of message
 * @param x value1
 * @param y value2
 * @param z value3
 * @param heading value4
 * @param ID value5
 * @param cmd value6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aq_robotmsg_pack2(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t Index, float x, float y, float z, float heading, float ID, uint16_t cmd)
{
       // printf("here 0!/n");
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, heading);
	_mav_put_float(buf, 16, ID);
	_mav_put_uint16_t(buf, 20, Index);
	_mav_put_uint16_t(buf, 22, cmd);
       // printf("here!/n");
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#else
	mavlink_aq_robotmsg_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.heading = heading;
	packet.ID = ID;
	packet.Index = Index;
	packet.cmd = cmd;
      //  printf("here1!/n");
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AQ_ROBOTMSG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN, MAVLINK_MSG_ID_AQ_ROBOTMSG_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#endif
}

/**
 * @brief Pack a aq_robotmsg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Index Index of message
 * @param x value1
 * @param y value2
 * @param z value3
 * @param heading value4
 * @param ID value5
 * @param cmd value6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aq_robotmsg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t Index,float x,float y,float z,float heading,float ID,uint16_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, heading);
	_mav_put_float(buf, 16, ID);
	_mav_put_uint16_t(buf, 20, Index);
	_mav_put_uint16_t(buf, 22, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#else
	mavlink_aq_robotmsg_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.heading = heading;
	packet.ID = ID;
	packet.Index = Index;
	packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AQ_ROBOTMSG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN, MAVLINK_MSG_ID_AQ_ROBOTMSG_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#endif
}

/**
 * @brief Encode a aq_robotmsg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aq_robotmsg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aq_robotmsg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aq_robotmsg_t* aq_robotmsg)
{
	return mavlink_msg_aq_robotmsg_pack2(system_id, component_id, msg, aq_robotmsg->Index, aq_robotmsg->x, aq_robotmsg->y, aq_robotmsg->z, aq_robotmsg->heading, aq_robotmsg->ID, aq_robotmsg->cmd);
}

/**
 * @brief Encode a aq_robotmsg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aq_robotmsg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aq_robotmsg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aq_robotmsg_t* aq_robotmsg)
{
	return mavlink_msg_aq_robotmsg_pack_chan(system_id, component_id, chan, msg, aq_robotmsg->Index, aq_robotmsg->x, aq_robotmsg->y, aq_robotmsg->z, aq_robotmsg->heading, aq_robotmsg->ID, aq_robotmsg->cmd);
}

/**
 * @brief Send a aq_robotmsg message
 * @param chan MAVLink channel to send the message
 *
 * @param Index Index of message
 * @param x value1
 * @param y value2
 * @param z value3
 * @param heading value4
 * @param ID value5
 * @param cmd value6
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aq_robotmsg_send(mavlink_channel_t chan, uint16_t Index, float x, float y, float z, float heading, float ID, uint16_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, heading);
	_mav_put_float(buf, 16, ID);
	_mav_put_uint16_t(buf, 20, Index);
	_mav_put_uint16_t(buf, 22, cmd);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ROBOTMSG, buf, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN, MAVLINK_MSG_ID_AQ_ROBOTMSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ROBOTMSG, buf, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#endif
#else
	mavlink_aq_robotmsg_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.heading = heading;
	packet.ID = ID;
	packet.Index = Index;
	packet.cmd = cmd;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ROBOTMSG, (const char *)&packet, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN, MAVLINK_MSG_ID_AQ_ROBOTMSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ROBOTMSG, (const char *)&packet, MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#endif
#endif
}

#endif

// MESSAGE AQ_ROBOTMSG UNPACKING


/**
 * @brief Get field Index from aq_robotmsg message
 *
 * @return Index of message
 */
static inline uint16_t mavlink_msg_aq_robotmsg_get_Index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field x from aq_robotmsg message
 *
 * @return value1
 */
static inline float mavlink_msg_aq_robotmsg_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from aq_robotmsg message
 *
 * @return value2
 */
static inline float mavlink_msg_aq_robotmsg_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from aq_robotmsg message
 *
 * @return value3
 */
static inline float mavlink_msg_aq_robotmsg_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field heading from aq_robotmsg message
 *
 * @return value4
 */
static inline float mavlink_msg_aq_robotmsg_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ID from aq_robotmsg message
 *
 * @return value5
 */
static inline float mavlink_msg_aq_robotmsg_get_ID(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field cmd from aq_robotmsg message
 *
 * @return value6
 */
static inline uint16_t mavlink_msg_aq_robotmsg_get_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Decode a aq_robotmsg message into a struct
 *
 * @param msg The message to decode
 * @param aq_robotmsg C-struct to decode the message contents into
 */
static inline void mavlink_msg_aq_robotmsg_decode(const mavlink_message_t* msg, mavlink_aq_robotmsg_t* aq_robotmsg)
{
#if MAVLINK_NEED_BYTE_SWAP
	aq_robotmsg->x = mavlink_msg_aq_robotmsg_get_x(msg);
	aq_robotmsg->y = mavlink_msg_aq_robotmsg_get_y(msg);
	aq_robotmsg->z = mavlink_msg_aq_robotmsg_get_z(msg);
	aq_robotmsg->heading = mavlink_msg_aq_robotmsg_get_heading(msg);
	aq_robotmsg->ID = mavlink_msg_aq_robotmsg_get_ID(msg);
	aq_robotmsg->Index = mavlink_msg_aq_robotmsg_get_Index(msg);
	aq_robotmsg->cmd = mavlink_msg_aq_robotmsg_get_cmd(msg);
#else
	memcpy(aq_robotmsg, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_AQ_ROBOTMSG_LEN);
#endif
}
