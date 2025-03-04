#include "array"
#include "gladiator.h"

/*
This file is used to check the asserv of the robot. It applies a "cycle" of consigns to left and right wheels.
A cycle consists to successive consigns applied to wheels. The order of consigns is as follows:
    0.0 > inc > 2*inc > 3*inc ... until the consign is equal to MAX_CONSIGN
When consign is equal to MAX_CONSIGN, the consign decreases:
    MAX_CONSIGN > MAX_CONSIGN - inc > MAX_CONSIGN - 2*inc > ... > 0.0 > -inc > -2*inc > ... until the consign is equal
to -MAX_CONSIGN.
When consign is equal to MAX_CONSIGN, the consign increases again:
    -MAX_CONSIGN > MAX_CONSIGN + inc > MAX_CONSIGN + 2*inc > ... > 0.0

Then the cycle is ended. If other PIDs are provided in "pidCycles", it executes a new cycle with the new PIDs.

The asserv evolution can be seen using the script "check_asserv.py". This scripts listen the USB port where the robot is
connected.
*/

Gladiator *gladiator;

static constexpr double MAX_CONSIGN = 1.0;
static constexpr unsigned long DURATION_BEFORE_CHANGE_MS = 2 * 1000;
static constexpr int WAIT_TIME_BEFORE_START_MS = 10 * 1000;

struct PidCycle
{
    double Kp;
    double Ki;
    double Kd;

    void setPIDsToGladiator()
    {
        gladiator->control->setWheelPidCoefs(WheelAxis::LEFT, this->Kp, this->Ki, this->Kd);
        gladiator->control->setWheelPidCoefs(WheelAxis::RIGHT, this->Kp, this->Ki, this->Kd);
        gladiator->log("PID coeff changed, set as follows");
        gladiator->log("%f %f %f", this->Kp, this->Ki, this->Kd);
    }
};

double consl = 0.0;
double consr = 0.0;
unsigned long lastTime = 0;
double increase = 0.1;
bool asservBegun = false;
bool firstOcc = true;
PidCycle pid0 = {2.0, 14.0, 0.0}; // Réglage conseillé: 2.0, 14.0, 0.0

static constexpr int MAX_PID_CYCLES = 1; // Put here the number of pids you want to test
std::array<PidCycle, MAX_PID_CYCLES> pidCycles = {pid0}; // Add here the PidCycles to test
int cycleInt = 0;

/*
Blocking the asserv at launch of the robot if one of those conditions is verified:
    - If at beginning, an amount of time has not been spent (ie the robot waits at beginning),
    - When the robot has tested all the PIDs (ie each cycle of "pidCycles" has been done).
*/
const bool blockAsserv()
{
    return (millis() - lastTime < WAIT_TIME_BEFORE_START_MS and !asservBegun) or cycleInt > MAX_PID_CYCLES;
}

void changePidCoeffs(const int cycleInt)
{
    if (cycleInt <= MAX_PID_CYCLES)
    {
        pidCycles[cycleInt - 1].setPIDsToGladiator();
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, consl, true);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consr, true);
    }
}

void updateConsign()
{
    if (firstOcc)
    {
        firstOcc = false;
        return;
    }
    consr += increase;
    consl += increase;
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consr);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consl);
}

void reset();
void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    gladiator->game->enableFreeMode(RemoteMode::OFF);
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void reset()
{
    consl = 0.0;
    consr = 0.0;
    asservBegun = false;
    firstOcc = true;
    cycleInt = 0;
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        if (blockAsserv())
        {
            return;
        }
        if (!asservBegun)
        {
            asservBegun = true;
            gladiator->log("asserv begun...");
        }
        if (millis() - lastTime > DURATION_BEFORE_CHANGE_MS)
        {
            lastTime = millis();
            updateConsign();

            if (abs(consl) > MAX_CONSIGN - 0.03)
            {
                increase = -increase;
            }
            /* The consign is at zero, now we will begin a new cycle with new PIDs. */
            if (abs(consl) < 0.01 && increase > 0)
            {
                cycleInt += 1;
                changePidCoeffs(cycleInt);
            }
        }

        RobotData data = gladiator->robot->getData();
        gladiator->log("%lu %f %f %f %f", millis(), consl, consr, data.vl, data.vr);
        delay(20);
    }
}
