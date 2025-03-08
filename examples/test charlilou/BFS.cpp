#include "gladiator.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>


Gladiator *gladiator;

float kw = 0.15f;
float kv = 2.f;
float wlimit = 0.2f;
float vlimit = 1.;
float erreurPos = 0.05;
float angleThreshold=0.2;

// Déclaration des variables globales
std::queue<const MazeSquare*> q;
std::unordered_set<const MazeSquare*> visited;
std::unordered_map<const MazeSquare*, const MazeSquare*> parent;
std::vector<const MazeSquare*> path;

double reductionAngle(double x) {
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0)
        x += 2 * M_PI;
    return x - M_PI;
}

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
void resetLists() {
    // Réinitialiser les structures de données
    q = std::queue<const MazeSquare*>();
    visited.clear();
    parent.clear();
    path.clear();
}

void reset() {
    gladiator->log("Appel de la fonction de reset");
    resetLists();
}

void setup() {
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

bool isValid(const MazeSquare* square, std::unordered_set<const MazeSquare*>& visited) {
    return square != nullptr && visited.find(square) == visited.end();
}

bool hasReached(Position targetCoor, Position myPosition) {
    double dx = targetCoor.x - myPosition.x;
    double dy = targetCoor.y - myPosition.y;
    double d = sqrt(dx * dx + dy * dy);
    return d <= erreurPos;
}

void moveTo(const MazeSquare* target) {
    if (target == nullptr) {
        gladiator->log("Target square is null");
        return;
    }

    float squareSize = gladiator->maze->getSquareSize();
    Position targetCoor;
    Position myPosition = gladiator->robot->getData().position;
    targetCoor.x = (target->i + 0.5) * squareSize;
    targetCoor.y = (target->j + 0.5) * squareSize;

    while (!hasReached(targetCoor, myPosition)) {
        go_to(targetCoor, myPosition);
        myPosition = gladiator->robot->getData().position; // Mettre à jour la position actuelle
        delay(100); // Ajouter un délai pour permettre au robot de se déplacer
    }

    gladiator->log("Moving to square at (%d, %d)", target->i, target->j);
}

std::vector<const MazeSquare*> bfsToTarget(const MazeSquare* start, const MazeSquare* target) {
    if (start == nullptr || target == nullptr) {
        gladiator->log("Start or target square is null");
        return {};
    }

    std::queue<const MazeSquare*> q;
    std::unordered_set<const MazeSquare*> visited;
    std::unordered_map<const MazeSquare*, const MazeSquare*> parent;
    std::vector<const MazeSquare*> path;

    q.push(start);
    visited.insert(start);
    parent[start] = nullptr;

    while (!q.empty()) {
        const MazeSquare* current = q.front();
        q.pop();

        gladiator->log("Visiting square at (%d, %d)", current->i, current->j);

        if (current == target) {
            // Retrace the path
            const MazeSquare* step = current;
            while (step != nullptr) {
                path.push_back(step);
                step = parent[step];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (isValid(current->northSquare, visited)) {
            q.push(current->northSquare);
            visited.insert(current->northSquare);
            parent[current->northSquare] = current;
        }
        if (isValid(current->southSquare, visited)) {
            q.push(current->southSquare);
            visited.insert(current->southSquare);
            parent[current->southSquare] = current;
        }
        if (isValid(current->eastSquare, visited)) {
            q.push(current->eastSquare);
            visited.insert(current->eastSquare);
            parent[current->eastSquare] = current;
        }
        if (isValid(current->westSquare, visited)) {
            q.push(current->westSquare);
            visited.insert(current->westSquare);
            parent[current->westSquare] = current;
        }
    }

    return {};
}

void loop() {
    if (gladiator->game->isStarted()) {
        gladiator->log("Le jeu a commencé");

        const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
        if (nearestSquare == nullptr) {
            gladiator->log("Nearest square is null");
            return;
        }

        unsigned char targetI = 5, targetJ = 11;
        const MazeSquare* targetSquare = gladiator->maze->getSquare(targetI, targetJ);
        if (targetSquare == nullptr) {
            gladiator->log("Target square is null");
            return;
        }

        std::vector<const MazeSquare*> path = bfsToTarget(nearestSquare, targetSquare);
        if (!path.empty()) {
            for (const MazeSquare* step : path) {
                moveTo(step);
            }
            gladiator->log("Cible atteinte!");
            resetLists();
        } else {
            gladiator->log("Aucun chemin trouvé vers la cible.");
        }

        delay(150);
    }
}