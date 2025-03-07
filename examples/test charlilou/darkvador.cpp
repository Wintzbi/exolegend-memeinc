#include "gladiator.h"

Gladiator *gladiator;
bool robot1 = false;

void reset();
void loop();
void retreat();

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

    // Vérifier si le robot a bougé
    if (distance > 0.01) 
    {
        lastPosition = currentPosition;
        lastMoveTime = millis();
    }

    // Retourne vrai si le robot est bloqué depuis 2,5 secondes
    return (millis() - lastMoveTime) > 2500;
}

void retreat()
{
    if (isRobotStuck())
    {
        gladiator->log("Robot bloqué, activation de la retraite !");
        
        // Reculer pendant 1 seconde
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, -0.8);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -0.8);
        delay(500);

        // S'arrêter après avoir reculé
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
    }
}

void reset()
{
    // Initialisation avant le début d'un match
    gladiator->log("Robot reset successfully");
    Position startPosition = gladiator->robot->getData().position;
    robot1 = (startPosition.x < 0.4);
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        // Vérifier si on peut dropper une bombe
        if (gladiator->weapon->canDropBombs(1)) {
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }

        // Récupérer les données du robot actuel
        RobotData myData = gladiator->robot->getData();
        RobotList robotList = gladiator->game->getPlayingRobotsId();
        RobotData enemyData{};
        bool enemyFound = false;

        // Identifier un adversaire
        for (uint8_t id : robotList.ids)
        {
            if (id != 0 && id != myData.id)
            {
                enemyData = gladiator->game->getOtherRobotData(id);
                enemyFound = true;
                break; // On prend le premier adversaire trouvé
            }
        }

        if (enemyFound)
        {
            // Calculer la direction vers l'adversaire
            double dx = enemyData.position.x - myData.position.x;
            double dy = enemyData.position.y - myData.position.y;
            double targetAngle = atan2(dy, dx);

            // Ajuster la direction du robot
            double angleDiff = targetAngle - myData.position.a;
            if (angleDiff > M_PI)
                angleDiff -= 2 * M_PI;
            if (angleDiff < -M_PI)
                angleDiff += 2 * M_PI;

            double speed = 0.5;  // Valeur réduite pour éviter les excès
            double turnSpeed = angleDiff * 1.5; 

            double vl = speed - turnSpeed;
            double vr = speed + turnSpeed;

            // Appliquer les nouvelles vitesses aux roues
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, vl);
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, vr);

            gladiator->log("Tracking enemy at (%f, %f)", enemyData.position.x, enemyData.position.y);
        }
        else
        {
            gladiator->log("Aucun ennemi détecté");
        }

        // Vérifier si le robot est bloqué et activer la retraite si nécessaire
        retreat();

        delay(100);
    }
}
