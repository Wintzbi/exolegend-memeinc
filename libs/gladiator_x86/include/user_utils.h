
#ifndef USER_UTILS_H
#define USER_UTILS_H

#include <Arduino.h>

struct Position
{
    float x{}, y{}, a{};
};

enum class WheelAxis
{
    RIGHT,
    LEFT
};

enum class WeaponPin
{
    M1,
    M2,
    M3
};
enum class WeaponMode
{
    SERVO,
    PWM
};
enum class RemoteMode
{
    ON,
    OFF
};

struct RobotData
{
    Position position{}, cposition{};
    double speedLimit{}, vl{}, vr{};
    short score{};
    uint8_t lifes{}, id{}, teamId{};
    String macAddress{};
    bool remote = false;
    int inventory = 0;
};

struct Wall
{
    bool active;
    Position p1, p2;
};

struct Coin
{
    uint8_t value{};
    Position p{};
};

struct Module
{
    Wall left, right, top, bottom;
    Coin coin;
    uint8_t value, i, j;
};

struct MazeSquare
{
    uint8_t i{0};
    uint8_t j{0};
    MazeSquare *northSquare{nullptr};
    MazeSquare *southSquare{nullptr};
    MazeSquare *westSquare{nullptr};
    MazeSquare *eastSquare{nullptr};
    Coin coin{};
    uint8_t possession{0};
    uint8_t danger{0};
    bool isBomb{false};
};

struct RobotList
{
    uint8_t ids[4];
};
#endif
