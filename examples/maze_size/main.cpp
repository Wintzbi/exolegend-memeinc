#include "gladiator.h"
Gladiator *gladiator;
void reset();
void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void reset()
{
    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer

        // Récupérer la taille du terrain actuel 
        float mazeSize = gladiator->maze->getCurrentMazeSize();
        // Récupérer la position du robot
        Position robotPos = gladiator->robot->getData().position;
        // Récupérer la taille maximum du terrain
        float mazeTotalSize = gladiator->maze->getSquareSize() * 12;

        float limitMin = (mazeTotalSize - mazeSize) / 2;
        float limitMax = (mazeTotalSize - mazeSize) / 2 + mazeSize;

        // Regarder si le robot est en dehors du terrain
        if (robotPos.x > limitMax || robotPos.x < limitMin || robotPos.y > limitMax || robotPos.y < limitMin) {
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.5);
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.5);
        } else {
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.0);
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.0);
        }
    }
    delay(100);
}