#include "gladiator.h"
#include "Vector2.hpp"

Gladiator *gladiator;
bool robot1 = false;

void reset();
void loop();
void retreat();
void arme_fou(int duree);
void shouldGo();

std::chrono::time_point<std::chrono::system_clock> start, end;
float time_elapsed;
int Num_tour=0;

void setup()
{
    // Instanciation de l'objet Gladiator
    gladiator = new Gladiator();
    // Enregistrement de la fonction reset avant chaque match
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
        gladiator->log("Danger détécté", dangerLevel);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
        delay(2000);
    }
}

bool CheckFuturCase(int i, int j){
    Num_tour = floor((time_elapsed + 6) / 20.f); // Arrondi à l'inférieur

    // Récupérer la taille réelle du labyrinthe en cases
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = floor(MazeSize / squareSize)-1;

    // Log pour débogage
    gladiator->log("Numéro tour %d,m MazeSize %d", Num_tour,MazeSize_int);
    gladiator->log("Limites : (%d, %d)", MazeSize_int - Num_tour, Num_tour);

    // Vérification des limites du labyrinthe
    if (i == Num_tour || i == MazeSize_int - Num_tour  || j == Num_tour || j == MazeSize_int - Num_tour ) {
        //gladiator->log("Bombe or limite");
        return false;  // Ne pas explorer cette case
    }
    return true;  // La case est valide
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        shouldGo();
        
        if (gladiator->weapon->canDropBombs(1)) {
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
                if (enemyData.lifes > 0) {
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
                while (enemyData.lifes > 0) 
                {
                    arme_fou(200);
                    enemyData = gladiator->game->getOtherRobotData(enemyData.id);
                }
            } 
            else 
            {
                double targetAngle = atan2(dy, dx);
                double angleDiff = targetAngle - myData.position.a;
                if (angleDiff > M_PI)
                    angleDiff -= 2 * M_PI;
                if (angleDiff < -M_PI)
                    angleDiff += 2 * M_PI;

                double speed = 0.5;
                double turnSpeed = angleDiff * 1.5;
                double vl = speed - turnSpeed;
                double vr = speed + turnSpeed;

                gladiator->control->setWheelSpeed(WheelAxis::LEFT, vl);
                gladiator->control->setWheelSpeed(WheelAxis::RIGHT, vr);

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
