#include "gladiator.h"
#include "Vector2.hpp"
#include <vector>
#include <array>
#include <math.h>
#include <cmath>
#undef abs

void reset();
void loop();
void retreat();
void arme_fou(int duree);
void shouldGo();
void dropBomb();
void boom();
void reductionAngle();

Gladiator *gladiator;

float kw = 0.4f;
float kv = 2.f;
float wlimit = 0.5f;
float vlimit = 1.25;
float erreurPos = 0.05;
float angleThreshold=0.1;

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}

void setup()
{
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);

    // Initialisation du servo
    gladiator->weapon->initWeapon(WeaponPin::M1, WeaponMode::SERVO);
    gladiator->weapon->setTarget(WeaponPin::M1, 80);
}

void dropBomb(){
    int bombcount = gladiator->weapon->getBombCount();
    if (gladiator->weapon->canDropBombs(1))
        {
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }
}

bool avoidDanger(const MazeSquare* neighbor){
    if(neighbor->danger < 1 ){
        return 1;
    }
    return 0;
}

void boom(const MazeSquare* nearestSquare, unsigned char teamId){
    if (nearestSquare->possession != teamId && avoidDanger(nearestSquare)){
        dropBomb();
    }
}

bool isRobotStuck()
{
    static Position lastPosition = gladiator->robot->getData().position;
    static unsigned long lastMoveTime = millis();

    Position currentPosition = gladiator->robot->getData().position;
    double dx = currentPosition.x - lastPosition.x;
    double dy = currentPosition.y - lastPosition.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance > 0.1)
    {
        lastPosition = currentPosition;
        lastMoveTime = millis();
    }

    return (millis() - lastMoveTime) > 2000;
}

void retreat()
{
    if (isRobotStuck())
    {
        gladiator->log("Robot bloqué, activation de la retraite !");
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, -0.4);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -0.4);
        delay(200);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, -0.4);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
        delay(200);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
    }
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
        delay(1500);
    }
}

inline float moduloPi(float a) // return angle in [-pi; pi]
{
    return (a < 0.0) ? (std::fmod(a - M_PI, 2 * M_PI) + M_PI) : (std::fmod(a + M_PI, 2 * M_PI) - M_PI);
}
inline bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs)
{
    constexpr float ANGLE_REACHED_THRESHOLD = 0.05;
    constexpr float POS_REACHED_THRESHOLD = 0.20;

    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};

    Vector2 posError = target - pos;

    float targetAngle = posError.angle();
    float angleError = moduloPi(targetAngle - posRaw.a);

    bool targetReached = false;
    float leftCommand = 0.f;
    float rightCommand = 0.f;

    if (posError.norm2() < POS_REACHED_THRESHOLD) //
    {
        gladiator->log("Ennemi proche ! Activation de l'arme folle. 1");
        arme_fou(300);
        retreat();
    }

    else if (std::abs(angleError) > ANGLE_REACHED_THRESHOLD)
    {
        float factor = 0.35;
        if (angleError < 0)
            factor = -factor;
        rightCommand = factor;
        leftCommand = -factor;
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);
    }
    else
    {
        float factor = 0.75;
        rightCommand = factor + angleError * 0.1; //+angleError*0.1  => terme optionel, "pseudo correction angulaire";
        leftCommand = factor - angleError * 0.1;  //-angleError*0.1   => terme optionel, "pseudo correction angulaire";
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);
    }

    if (showLogs || targetReached)
    {
        gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError, target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}

/******************************************/

struct PositionEx
{
    Vector2 pos;
    bool isEnnemy;
    int life_ennemies;
};

std::vector<PositionEx> ennemies{};

std::array<PositionEx, 4>
fetchEnnemyRobotsData()
{
    std::array<PositionEx, 4> res{};
    RobotList robotsIds = gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; ++i)
    {
        RobotData other = gladiator->game->getOtherRobotData(robotsIds.ids[i]);
        res.at(i).isEnnemy = other.teamId != gladiator->robot->getData().teamId;
        res.at(i).pos = {other.position.x, other.position.y};
        res.at(i).life_ennemies = other.lifes;
    }
    return res;
}

PositionEx findNearest(std::vector<PositionEx> ennemies) // renvoie la chose la plus proche
{
    float min = 100000.f;
    PositionEx nearest{};
    const Position robotPos = gladiator->robot->getData().position;
    for (auto ennemy : ennemies)
    {
        const float dist = std::sqrt((robotPos.x - ennemy.pos.x()) * (robotPos.x - ennemy.pos.x()) + (robotPos.y - ennemy.pos.y()) * (robotPos.y - ennemy.pos.y()));
        if (dist < min)
        {
            min = dist;
            nearest = ennemy;
        }
    }
    return nearest;
}

Position getNearestEnnemy()
{ // renvoie l'énemie le plus proche
    ennemies.clear();
    const std::array<PositionEx, 4> robots = fetchEnnemyRobotsData();
    for (auto rpos : robots)
    {
        if (rpos.isEnnemy && rpos.life_ennemies)
        {
            // gladiator->log("état vie adverse :%f", rpos.life_ennemies);
            ennemies.push_back(rpos);
        }
    }

    if (ennemies.size() > 0)
    {
        // gladiator->log("ennemies size :%f", ennemies.size());
        PositionEx nearestEnnemy = findNearest(ennemies);
        return Position{nearestEnnemy.pos.x(), nearestEnnemy.pos.y()};
    }
    return {0.f};
}

bool limite_maze(Position target)
{
    if ((float)target.x > 0 &&
        (float)target.x < gladiator->maze->getSize() &&
        (float)target.y > 0 && (float)target.y < gladiator->maze->getSize())
    {
        gladiator->log("Cible dans le terrain");
        return true;
    }

    return false; // Added this line
}

void reset()
{
    gladiator->log("Robot reset successfully");
    Position startPosition = gladiator->robot->getData().position;
}


/******************************************/


void go_to(Position cons, Position pos)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    // Si la position cible est suffisamment éloignée
    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double angleDifference = reductionAngle(rho - pos.a);
        
        // Si l'angle entre la direction actuelle et la direction cible est trop grand, tourner sur place
        if (fabs(angleDifference) > angleThreshold) // angleThreshold est un seuil que vous définissez
        {
            // Contrôle de la vitesse des roues pour faire tourner le robot sur place
            double consw = kw * angleDifference;
            consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;

            consvl = -consw;  // Roue gauche tourne dans une direction
            consvr = consw;   // Roue droite tourne dans la direction opposée
        }
        else
        {
            // Si l'angle est suffisamment petit, le robot peut avancer
            double consw = kw * angleDifference;
            double consv = kv * d * cos(angleDifference);
            
            consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
            consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

            consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
            consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        }
    }
    else
    {
        // Si la position est proche de la cible, arrêter le robot
        consvr = 0;
        consvl = 0;
    }

    // Appliquer les vitesses aux roues
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        Position myPosition = gladiator->robot->getData().position;
        Position target = getNearestEnnemy();
        if (target.x == 0 && target.y == 0) // plus d'enemie
        {
            target = {1.5, 1.5, 0}; // rajouter code ley
        }

        if (limite_maze(target)) { // Ennemie dans la map
            Position goal{target.x, target.y, 0};
            go_to(goal, myPosition);

            // Vérification de la distance
            double dx = target.x - myPosition.x;
            double dy = target.y - myPosition.y;
            double distance = sqrt(dx * dx + dy * dy);

            // Si l'ennemi est suffisamment proche, déclenche l'arme folle
            if (distance < 0.2) // Vous pouvez ajuster ce seuil
            {
                gladiator->log("Ennemi proche ! Activation de l'arme folle. 2");
                arme_fou(300);
                retreat(); // Vous pouvez ajuster la durée de l'arme folle si nécessaire
            }
        }
    }
    delay(10);
}
