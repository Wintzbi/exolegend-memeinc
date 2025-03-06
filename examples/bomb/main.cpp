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
        // Récuperer le nombre de bombes restantes
        int bombCount = gladiator->weapon->getBombCount();
        
        // Si il reste plus de 2 bombes
        if (bombCount > 2) {
            // Dropper toutes les bombes sauf 2
            gladiator->weapon->dropBombs(bombCount - 2);
            gladiator->log("Drop bomb");
        // Il peux dropper au moins 1 bombe
        } else if (gladiator->weapon->canDropBombs(1)) {
            // Dropper une bombe
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }
    }
    delay(300);
}
