#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <sys/time.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define SLIME_DEBUG_PRINT(fmt, ...) fprintf(stderr, "\033[32;1m[slime.cc] " fmt "\033[0m\n", ##__VA_ARGS__)

#define PACKET_HEARTBEAT 0
// #define PACKET_ROTATION 1 // Deprecated
// #define PACKET_GYRO 2 // Deprecated
#define PACKET_HANDSHAKE 3
#define PACKET_ACCEL 4
// #define PACKET_MAG 5 // Deprecated
// #define PACKET_RAW_CALIBRATION_DATA 6 // Deprecated
// #define PACKET_CALIBRATION_FINISHED 7 // Deprecated
#define PACKET_CONFIG 8
// #define PACKET_RAW_MAGNETOMETER 9 // Deprecated
#define PACKET_PING_PONG 10
#define PACKET_SERIAL 11
#define PACKET_BATTERY_LEVEL 12
#define PACKET_TAP 13
#define PACKET_ERROR 14
#define PACKET_SENSOR_INFO 15
// #define PACKET_ROTATION_2 16 // Deprecated
#define PACKET_ROTATION_DATA 17
#define PACKET_MAGNETOMETER_ACCURACY 18
#define PACKET_SIGNAL_STRENGTH 19
#define PACKET_TEMPERATURE 20
// #define PACKET_USER_ACTION 21 // Joycon buttons only currently
#define PACKET_FEATURE_FLAGS 22

#define PACKET_BUNDLE 100

#define PACKET_INSPECTION 105  // 0x69

#define PACKET_RECEIVE_HEARTBEAT 1
#define PACKET_RECEIVE_VIBRATE 2
#define PACKET_RECEIVE_HANDSHAKE 3
#define PACKET_RECEIVE_COMMAND 4

#define PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA 1
#define PACKET_INSPECTION_PACKETTYPE_FUSED_IMU_DATA 2
#define PACKET_INSPECTION_PACKETTYPE_CORRECTION_DATA 3
#define PACKET_INSPECTION_DATATYPE_INT 1
#define PACKET_INSPECTION_DATATYPE_FLOAT 2

template <typename T>
unsigned char* convert_to_chars(T src, unsigned char* target) {
    union uwunion {
        unsigned char c[sizeof(T)];
        T v;
    } un;
    un.v = src;
    for (size_t i = 0; i < sizeof(T); i++) {
        target[i] = un.c[sizeof(T) - i - 1];
    }
    return target;
}

template <typename T>
T convert_chars(unsigned char* const src) {
    union uwunion {
        unsigned char c[sizeof(T)];
        T v;
    } un;
    for (size_t i = 0; i < sizeof(T); i++) {
        un.c[i] = src[sizeof(T) - i - 1];
    }
    return un.v;
}

static struct {
    int socket;

    struct sockaddr_in s;

    bool connected;
    uint64_t packetNumber;

    unsigned char buf[128];
    char packetBuffer[1024];
    int packetBufferHead;
} slimeNetworkState = { 0 };

void _slimeBeginPacket() {
    slimeNetworkState.packetBufferHead = 0;
}

void _slimeWrite(const unsigned char *buffer, size_t size) {
    memcpy(slimeNetworkState.packetBuffer + slimeNetworkState.packetBufferHead, buffer, size);
    slimeNetworkState.packetBufferHead += size;
}

void _slimeEndPacket() {
    int broadcastEnable = slimeNetworkState.connected ? 0 : 1;
    setsockopt(slimeNetworkState.socket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    if(slimeNetworkState.connected) {
        // send directly to connected host
        if(sendto(
                slimeNetworkState.socket,
                slimeNetworkState.packetBuffer,
                slimeNetworkState.packetBufferHead,
                0,
                (struct sockaddr *)&slimeNetworkState.s,
                sizeof(struct sockaddr_in)
            ) < 0)
        {
            SLIME_DEBUG_PRINT("sendto (direct) failed");
        }
    } else {
        // broadcast it
        struct sockaddr_in s;

        memset(&s, '\0', sizeof(struct sockaddr_in));
        s.sin_family = AF_INET;
        s.sin_port = htons(6969);
        s.sin_addr.s_addr = INADDR_BROADCAST;

        if(sendto(
                slimeNetworkState.socket,
                slimeNetworkState.packetBuffer,
                slimeNetworkState.packetBufferHead,
                0,
                (struct sockaddr *)&s,
                sizeof(struct sockaddr_in)
            ) < 0)
        {
            SLIME_DEBUG_PRINT("sendto (broadcast) failed");
        }
    }
}

void _slimeSendByte(uint8_t c) {
    _slimeWrite((const unsigned char *)&c, 1);
}

void _slimeSendShort(uint16_t s) {
    convert_to_chars(s, slimeNetworkState.buf);
    _slimeWrite(slimeNetworkState.buf, sizeof(s));
}

void _slimeSendFloat(float f) {
    convert_to_chars(f, slimeNetworkState.buf);
    _slimeWrite(slimeNetworkState.buf, sizeof(f));
}

void _slimeSendInt(uint32_t i) {
    convert_to_chars(i, slimeNetworkState.buf);
    _slimeWrite(slimeNetworkState.buf, sizeof(i));
}

void _slimeSendLong(uint64_t l) {
    convert_to_chars(l, slimeNetworkState.buf);
    _slimeWrite(slimeNetworkState.buf, sizeof(l));
}

void _slimeSendBytes(const uint8_t *c, size_t length) {
    _slimeWrite(c, length);
}

void _slimeSendPacketNumber(void) {
    _slimeSendLong(slimeNetworkState.packetNumber++);
}

void _slimeSendShortString(const char *str) {
    uint8_t size = (uint8_t)strlen(str);
    _slimeSendByte(size);
    _slimeSendBytes((const uint8_t *)str, size);
}

void _slimeSendPacketType(uint8_t type) {
    _slimeSendByte(0);
    _slimeSendByte(0);
    _slimeSendByte(0);
    _slimeSendByte(type);
}

void _slimeSendLongString(const char* str) {
    int size = strlen(str);
    _slimeSendInt(size);
    _slimeSendBytes((const uint8_t *)str, size);
}

void slimeConfigureSensor(uint8_t sensor);


// Search for server on the network. Returns true if found and connected
// Blocks the calling thread for up to 5 seconds if no server is running
// You can set deviceId value to any unique value for this device.
// The port `9185 + deviceId` should be free
bool slimeDiscoverServer(uint8_t deviceId) {
    if(slimeNetworkState.connected) return true;
    if(slimeNetworkState.socket == 0) {
        slimeNetworkState.socket = socket(AF_INET, SOCK_DGRAM, 0);

        // SlimeVR server may fail to reconnect if the address (port) has changed
        // so we need to bind it to keep it the same between connections.
        struct sockaddr_in servaddr;
        bzero(&servaddr, sizeof(servaddr));
        servaddr.sin_family = AF_INET; 
        servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        servaddr.sin_port = htons(9185 + deviceId);

        if(bind(slimeNetworkState.socket, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
            SLIME_DEBUG_PRINT("Failed to bind on port %d! You may have trouble reconnecting", 9185 + deviceId);
        }

        struct timeval timeout = { 5, 0 }; // set timeout for 5 seconds
        setsockopt(slimeNetworkState.socket, SOL_SOCKET, SO_RCVTIMEO,(char*)&timeout,sizeof(struct timeval));
    }

    SLIME_DEBUG_PRINT("Discovering server...");

    // The mac itself doesn't matter, as long as it's the same when the deviceId is the same
    // and doesn't conflict with any other addresses hopefully. It also can't be 00:00:00:00:00:00
    uint8_t mac[6];
    mac[0] = 255;
    mac[1] = deviceId;
    mac[2] = deviceId % 2;
    mac[3] = 120;
    mac[4] = 125;

    _slimeBeginPacket();
    _slimeSendPacketType(PACKET_HANDSHAKE);
    _slimeSendLong(0); // packet number is always 0

    _slimeSendInt(0); // board
    _slimeSendInt(0); // imu
    _slimeSendInt(0); // hardware mcu
    _slimeSendInt(0);
    _slimeSendInt(0);
    _slimeSendInt(0);
    _slimeSendInt(1); // firmware build number
    _slimeSendShortString("UWUFirmware"); // firmware
    _slimeSendBytes(mac, 6);

    _slimeEndPacket();

    // Now we need to receive packets and see if we've received back handshake
    while(true) {
        socklen_t addrlen = sizeof(struct sockaddr_in);
        auto size = recvfrom(slimeNetworkState.socket, slimeNetworkState.packetBuffer, 1024,
            0,
            (struct sockaddr *)&slimeNetworkState.s,
            &addrlen);

        if(size <= 0) {
            SLIME_DEBUG_PRINT("recvfrom failed or timed out (is the server running?)");
            return false;
        }

        if(addrlen != sizeof(struct sockaddr_in)) {
            SLIME_DEBUG_PRINT("addrlen returned by recvfrom (%d) not equal to sizeof sockaddr_in (%d)", addrlen, sizeof(struct sockaddr_in));
            return false;
        }

        if(slimeNetworkState.packetBuffer[0] == PACKET_HANDSHAKE) {
            if (strncmp((char*)slimeNetworkState.packetBuffer + 1, "Hey OVR =D 5", 12) != 0) {
                SLIME_DEBUG_PRINT("response packet did not contain string 'Hey OVR =D 5'");
                return false;
            }
            
            SLIME_DEBUG_PRINT("handshake successful, connected");
            slimeNetworkState.connected = true;
            return true;
        } else {
            continue;
        }
    }
}

// Call this every once in a while if you're not sending rotation to keep the connection alive
void slimeSendHeartbeat(void) {
    if(!slimeNetworkState.connected) return;

    _slimeBeginPacket();
    _slimeSendPacketType(PACKET_HEARTBEAT);
    _slimeSendPacketNumber();
    _slimeEndPacket();
}

// Must be called at least once for each sensor you're using, except for sensor 0 which
// is configured automatically
void slimeConfigureSensor(uint8_t sensor) {
    _slimeBeginPacket();

    _slimeSendPacketType(PACKET_SENSOR_INFO);
    _slimeSendPacketNumber();
    _slimeSendByte(sensor); // sensor id
    _slimeSendByte(1); // sensor state
    _slimeSendByte(0); // sensor type

    _slimeEndPacket();
}

// Send rotation to server with the given sensor. For simple uses just set sensor to 0
// and it will work. If you want more sensors you will need to call slimeConfigureSensor
// with each sensor you want to use
void slimeSendRotationQuatSensor(uint8_t sensor, float x, float y, float z, float w) {
    if(!slimeNetworkState.connected) return;

    _slimeBeginPacket();

    _slimeSendPacketType(PACKET_ROTATION_DATA);
    _slimeSendPacketNumber();
    _slimeSendByte(sensor); // sensor id
    _slimeSendByte(1); // DATA_TYPE_NORMAL
    _slimeSendFloat(x);
    _slimeSendFloat(y);
    _slimeSendFloat(z);
    _slimeSendFloat(w);

    _slimeSendByte(0); // calibration accuracy(?)
    
    _slimeEndPacket();
}

// Shortcut for sending rotation using sensor 0. Use this for simple use cases
void slimeSendRotationQuat(float x, float y, float z, float w) {
    slimeSendRotationQuatSensor(0, x, y, z, w);
}