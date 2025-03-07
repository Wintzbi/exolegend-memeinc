#include "gladiator.h"
#include "Vector2.hpp"
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



struct PositionEx
{
    Vector2 pos;
    bool isEnnemy;
    int life_ennemies;
    double vr;
    double vl;
    double angle;
};
Position predictEnemyPosition(const Position &current, double vl, double vr, float currentAngle)
{
    // Calculer la vitesse totale en utilisant la moyenne des vitesses des roues
    double totalSpeed = 0.5 * (vl + vr);

    // Utilisez une extrapolation linéaire avec ajustement en fonction de l'angle
    float predictionTime = 1.0; // Ajustez ce paramètre en fonction de votre situation
    float predictedX = current.x + totalSpeed * predictionTime * cos(currentAngle);
    float predictedY = current.y + totalSpeed * predictionTime * sin(currentAngle);

    return Position{predictedX, predictedY, 0};
}


void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        RobotList robotList = gladiator->game->getPlayingRobotsId(); // GFA 4.3.5
        gladiator->log("Robots playing : %d, %d, %d, %d ---", robotList.ids[0], robotList.ids[1], robotList.ids[2],
                       robotList.ids[3]); // GFA 4.5.1
        
        // Obtenir les données du robot
        RobotData data = gladiator->game->getOtherRobotData(id);
        Position cposition = data.cposition;
        return {cposition.x,cposition.y}
    }

}
