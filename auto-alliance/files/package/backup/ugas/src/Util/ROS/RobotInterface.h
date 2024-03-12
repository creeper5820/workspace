//
// Created by soulde on 2023/3/4.
//

#ifndef SERIAL_SERIALFRAMEDEALER_H
#define SERIAL_SERIALFRAMEDEALER_H

#define float32_t float
static constexpr unsigned char HEAD = 0xFE;

enum class SendPackageID : unsigned char {
    GIMBAL = 0x01,
    MOVE = 0x02,
    GNSS = 0x03,
    HEARTBEAT = 0x41
};

enum class RecvPackageID : unsigned char {
    GIMBAL = 0x81,
    ODOM = 0x82,
    GOAL = 0x83,
    ENEMY = 0x84,
    UWB = 0x85,
    BUFF = 0xC1
};
enum Enemy {
    UNKNOWN = -1,
    BLUE = 0,
    RED = 1,

};
#pragma pack(1)
inline struct Header {
    const uint8_t head = HEAD;
    uint8_t id = 0x00;
    uint8_t crc8 = 0x00;
} header;

inline struct MoveControlFrame {
    Header header;
    float32_t x = 0.;
    float32_t y = 0.;
    float32_t yaw = 0.;
    uint8_t crc8 = 0x0;

    MoveControlFrame() {
        header.id = static_cast<unsigned char>(SendPackageID::MOVE);
    }

} moveControlFrame;
inline std::atomic<bool> isMoveControlFrameUpdated = false;

inline struct GimbalControlFrame {
    Header header;
    float32_t pitch = 0.;
    float32_t yaw = 0.;
    uint8_t fire = 0;
    uint8_t crc8 = 0x0;

    GimbalControlFrame() {
        header.id = static_cast<unsigned char>(SendPackageID::GIMBAL);
    }
} gimbalControlFrame;

inline struct GoalFeedbackFrame {
    Header header;
    float32_t x{};
    float32_t y{};
    uint8_t signal{};
    uint8_t crc8 = 0x0;
} goalFeedbackFrame;

inline struct EnemyFeedbackFrame {
    Header header;
    uint8_t isRed{};
    uint8_t barrel{};
    uint8_t crc8 = 0x0;
} enemyFeedbackFrame;

inline struct UWBFeedbackFrame {
    Header header;
    float32_t x{};
    float32_t y{};
    uint8_t crc8 = 0x0;
} uwbFeedbackFrame;
#pragma pack()


#endif //SERIAL_SERIALFRAMEDEALER_H
