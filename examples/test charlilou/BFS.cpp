#include "gladiator.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>

Gladiator *gladiator;

float kw = 1.2;
float kv = 1.0f;
float wlimit = 3.0f;
float vlimit = 0.6f;
float erreurPos = 0.07f;

double reductionAngle(double x) {
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0)
        x += 2 * M_PI;
    return x - M_PI;
}

void go_to(Position cons, Position pos) {
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos) {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);

        double consv = kv * d * cos(reductionAngle(rho - pos.a));
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw;
        consvr = consv + gladiator->robot->getRobotRadius() * consw;
    } else {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
}

void reset() {
    gladiator->log("Appel de la fonction de reset");
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

bool bfsToTarget(const MazeSquare* start, const MazeSquare* target) {
    if (start == nullptr || target == nullptr) {
        gladiator->log("Start or target square is null");
        return false;
    }

    std::queue<const MazeSquare*> q;
    std::unordered_set<const MazeSquare*> visited;
    std::unordered_map<const MazeSquare*, const MazeSquare*> parent;

    q.push(start);
    visited.insert(start);
    parent[start] = nullptr;

    while (!q.empty()) {
        const MazeSquare* current = q.front();
        q.pop();

        //gladiator->log("Visiting square at (%d, %d)", current->i, current->j);

        if (current == target) {
            const MazeSquare* step = current;
            while (step != nullptr) {
                moveTo(step);
                step = parent[step];
            }
            return true;
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

    return false;
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

        if (bfsToTarget(nearestSquare, targetSquare)) {
            gladiator->log("Cible atteinte!");
        } else {
            gladiator->log("Aucun chemin trouvé vers la cible.");
        }

        delay(100);
    }
}
