#ifndef _Gps_h
#define _Gps_h

class gps
{
public:
    gps();
    void GPS_Serial2Init(void);
    bool GPS_newFrame(uint8_t data);
    void insertPeriod(String &number, unsigned int position);
    void codisplay(int key, uint32_t num);
    bool HasLock();
    float Speed();

private:
    void Serial2GpsPrint(const char PROGMEM *str);
    uint16_t grab_fields(char *src, uint8_t mult);
    uint8_t _step = 0; // State machine state
    uint8_t _msg_id;
    uint16_t _payload_length;
    uint16_t _payload_counter;
    uint8_t _ck_a; // Packet checksum accumulators
    uint8_t _ck_b;
    uint32_t GPS_speed;
    uint32_t avg_GPS_speed[SPEED_SAMPLES];
};

// void GPS_Serial2Init(void);
// bool GPS_newFrame(uint8_t data);

const unsigned char UBLOX_INIT[] PROGMEM = {
    // PROGMEM array must be outside any function !!!
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19, // disable all default NMEA messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,                               // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,                               // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,                               // set SOL MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,                               // set VELNED MSG rate
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41, // set WAAS to EGNOS
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A              // set rate to 5Hz
};

struct ubx_header
{
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
};
struct ubx_nav_posllh
{
    uint32_t time; // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
};
struct ubx_nav_solution
{
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
};
struct ubx_nav_velned
{
    uint32_t time; // GPS msToW[5] = {9600, 19200, 38400, 57600, 115200}
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
};

enum ubs_protocol_bytes
{
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type
{
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
};

enum ubx_nav_status_bits
{
    NAV_STATUS_FIX_VALID = 1
};

struct bytes
{
    int8_t bytes[100];
};

struct myTime
{
};
// Receive buffer
static union
{
    ubx_nav_posllh posllh;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    bytes bt;
    // uint8_t bytes[];
} _buffer;

typedef struct
{
    uint8_t OK_TO_ARM : 1;
    uint8_t ARMED : 1;
    uint8_t ACC_CALIBRATED : 1;
    uint8_t ANGLE_MODE : 1;
    uint8_t HORIZON_MODE : 1;
    uint8_t MAG_MODE : 1;
    uint8_t BARO_MODE : 1;
    uint8_t GPS_mode : 2; // 0-3 NONE,HOLD, HOME, NAV (see GPS_MODE_* defines
    uint8_t GPS_FIX : 1;
    uint8_t GPS_FIX_HOME : 1;
    uint8_t GPS_BARO_MODE : 1; // This flag is used when GPS controls baro mode instead of user (it will replace rcOptions[BARO]
    uint8_t GPS_head_set : 1;  // it is 1 if the navigation engine got commands to control heading (SET_POI or SET_HEAD) CLEAR_HEAD will zero it
    uint8_t LAND_COMPLETED : 1;
    uint8_t LAND_IN_PROGRESS : 1;
} flags_struct_t;

enum
{
    D_STATUS = 0,
    D_SAT,
    D_SAT_COUNT,
    D_TIME,
    D_TIME_H,
    D_TIME_M,
    D_TIME_S,
    D_LAT,
    D_LON,
    D_SPEED,
    D_SPEED_VAL,
    D_MAX
};
enum
{
    D_X = 0,
    D_Y = 1
};

#endif