#include "gladiator.h"
#include "Vector2.hpp"
#include <chrono>

Gladiator *gladiator;
bool robot1 = false;

void reset();
void loop();
void retreat();
void arme_fou(int duree);
void shouldGo();

float kw = 0.4f;
float kv = 2.f;
float wlimit = 0.5f;
float vlimit = 1.5;
float erreurPos = 0.05;
float angleThreshold = 0.1;


std::chrono::time_point<std::chrono::system_clock> start, end;
float time_elapsed;
int Num_tour = 0;

void setup()
{
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

bool isRobotStuck()
{
    static Position lastPosition = gladiator->robot->getData().position;
    static unsigned long lastMoveTime = millis();

    Position currentPosition = gladiator->robot->getData().position;
    double dx = currentPosition.x - lastPosition.x;
    double dy = currentPosition.y - lastPosition.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance > 0.01)
    {
        lastPosition = currentPosition;
        lastMoveTime = millis();
    }

    return (millis() - lastMoveTime) > 2500;
}

void retreat()
{
    if (isRobotStuck())
    {
        gladiator->log("Robot bloqué, activation de la retraite !");
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, -0.8);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -0.8);
        delay(500);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
    }
}

void reset()
{
    gladiator->log("Robot reset successfully");
    Position startPosition = gladiator->robot->getData().position;
    robot1 = (startPosition.x < 0.4);
}

void arme_fou(int duree)
{
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, 1);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 1);
    while (duree > 0)
    {
        gladiator->weapon->setTarget(WeaponPin::M1, 135);
        delay(100);
        gladiator->weapon->setTarget(WeaponPin::M1, 50);
        delay(100);
        duree -= 200;
    }
    gladiator->weapon->setTarget(WeaponPin::M1, 95);
}

void shouldGo()
{
    const MazeSquare *currentSquare = gladiator->maze->getNearestSquare();
    int dangerLevel = currentSquare->danger;

    if (dangerLevel >= 5)
    {
        gladiator->log("Danger détecté", dangerLevel);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
        delay(2000);
    }
}

void go_to(Position cons, Position pos)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos) // Suivi direct de l'ennemi avec précision
    {
        double rho = atan2(dy, dx);  // Direction vers l'ennemi
        double angleDifference = rho - pos.a;

        if (fabs(angleDifference) > angleThreshold)
        {
            // Rotation du robot
            double consw = kw * angleDifference;
            consw = (fabs(consw) > wlimit) ? (consw > 0 ? wlimit : -wlimit) : consw;
            consvl = -consw;
            consvr = consw;
        }
        else
        {
            // Déplacement vers l'ennemi
            double consv = kv * d;
            consv = (fabs(consv) > vlimit) ? (consv > 0 ? vlimit : -vlimit) : consv;

            consvl = consv - gladiator->robot->getRobotRadius() * 0;
            consvr = consv + gladiator->robot->getRobotRadius() * 0;
        }
    }
    else
    {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        shouldGo();

        if (gladiator->weapon->canDropBombs(1))
        {
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }

        RobotData myData = gladiator->robot->getData();
        RobotList robotList = gladiator->game->getPlayingRobotsId();
        RobotData enemyData{};
        bool enemyFound = false;

        for (uint8_t id : robotList.ids)
        {
            if (id != 0 && id != myData.id)
            {
                enemyData = gladiator->game->getOtherRobotData(id);
                if (enemyData.lifes > 0)
                {
                    enemyFound = true;
                    break;
                }
            }
        }

        if (enemyFound)
        {
            double dx = enemyData.position.x - myData.position.x;
            double dy = enemyData.position.y - myData.position.y;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance < 0.2)
            {
                gladiator->log("Ennemi proche ! Activation de l'arme folle.");
                arme_fou(200); // Utilise l'arme folle si proche
                go_to(enemyData.position, myData.position);  // Suivi précis tout en attaquant
            }
            else
            {
                go_to(enemyData.position, myData.position);  // Suivi droit de l'ennemi
                gladiator->log("Tracking enemy at (%f, %f)", enemyData.position.x, enemyData.position.y);
            }
        }
        else
        {
            gladiator->log("Aucun ennemi détecté");
        }

        retreat();
        delay(100);
    }
}
