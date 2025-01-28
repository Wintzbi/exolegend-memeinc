#ifndef GLADIATOR_H
#define GLADIATOR_H

#include "user_utils.h"
#include <Arduino.h>

class Maze
{
  public:
    MazeSquare *getNearestSquare();
    MazeSquare *getSquare(uint8_t i, uint8_t j);
    float getSize() const;
    float getSquareSize() const;
    float getCurrentMazeSize();
};

class Robot
{
  public:
    RobotData getData();
    void setCalibrationOffset(float dx, float dy, float da);
    float getRobotRadius() const;
    float getWheelRadius() const;
};

class Control
{
  public:
    void setWheelSpeed(WheelAxis wheelAxis, double speed);
    void setWheelSpeed(WheelAxis wheelAxis, double speed, bool reset);
    double getWheelSpeed(WheelAxis wheelAxis);
    void setWheelPidCoefs(WheelAxis wheelAxis, float Kp, float Ki, float Kd);
};

class Weapon
{
  public:
    void initWeapon(WeaponPin weaponPin, WeaponMode weaponMode);
    void setTarget(WeaponPin weaponPin, uint8_t value);
    void dropBombs(const int nbBomb);
    bool canDropBombs(const int nbBomb);
    int getBombCount();
};

class Game
{
  public:
    void onReset(void (*resetFunction)());
    bool isStarted();
    void enableFreeMode(RemoteMode enableRemote);
    void enableFreeMode(RemoteMode enableRemote, Position initPosition);
    RobotData getOtherRobotData(uint8_t id);
    RobotList getPlayingRobotsId();
    void setMinotorPin(int newPin);
};

class Gladiator
{
  public:
    Gladiator();
    void log(const char *format, ...) __attribute__((format(printf, 2, 3)));
    void saveUserWaypoint(float x, float y);
    void printVersionInfo();
    Maze *maze;
    Robot *robot;
    Control *control;
    Weapon *weapon;
    Game *game;
};

#endif
