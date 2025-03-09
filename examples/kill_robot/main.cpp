#include "gladiator.h"
#include "Vector2.h"
#include <vector>
#include <array>
#include <math.h>
#include <cmath>
#undef abs


Gladiator *gladiator;

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 1;
float erreurPos = 0.25;

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}

void retreat()
{
    // Reculer pendant 0.5 secondes
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, -0.8);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -0.8);
    delay(800);

    // Réinitialiser les variables ou effectuer d'autres actions nécessaires
    // ...

    // Reprendre la mission de base
    /*Position myPosition = gladiator->robot->getData().position;
    Position target = getNearestEnnemy();
    if (target.x == 0 && target.y == 0)
    {
        target = {2.1, 1.8, 0};
    }
    if (limite_maze(target))
    {
        Vector2 goal = Vector2(target.x, target.y);
        bool showLogs = false;
        aim(gladiator, goal, showLogs);
    }*/
}
void arme_fou(int duree)
{
    gladiator->log("Appel de la fonction arme fou");
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
        arme_fou(200);
        //delay(400);
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
    else if (gladiator->weapon->canLaunchRocket() && posError.norm2() >= POS_REACHED_THRESHOLD && posError.norm2() < 1)
    {
        delay(250);//le temps de viser
        gladiator->weapon->launchRocket();
        gladiator->log("After launch rocket call");
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
    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
    gladiator->weapon->initWeapon(WeaponPin::M1, WeaponMode::SERVO);
     gladiator->weapon->setTarget(WeaponPin::M1, 90);
    // gladiator->game->enableFreeMode(RemoteMode::OFF); // GFA 4.3.2
}
/******************************************/






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
       if( limite_maze(target)){ //enemie dans la map
        Vector2 goal = Vector2(target.x, target.y);
        bool showLogs = 0;
        aim(gladiator, goal, showLogs);}

        // turn(target, myPosition);
        // delay(500);
        // go_to(target, myPosition);
    }
    delay(10);
}