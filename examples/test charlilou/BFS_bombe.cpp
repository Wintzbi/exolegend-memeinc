#include "gladiator.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "vector2.hpp"
#include <chrono>
#define deltaX 0.0016
#define deltaY -0.0027
std::chrono::time_point<std::chrono::system_clock> start, end;
float time_elapsed;
int Num_tour = 0;
Gladiator *gladiator;
void reset();
void dropBomb();
bool UpdateNearestBomb = true;
Position LastBombToGet;
Position targetBomb = { -1, -1 };
#define MAX_BOMB 50
Position BombPos[MAX_BOMB];
bool CheckFutureCase(int i, int j,int futur_time) ;
std::vector<const MazeSquare*> bfsToTarget(const MazeSquare* start, const MazeSquare* target);



float kw = 0.15f;
float kv = 1.0f;
float wlimit = 0.25f;
float vlimit = 1.5f;
float erreurPos = 0.f;
float angleThreshold = 0.5f;

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

void go_to(Position cons, Position pos) {
    double consvl, consvr;
    double dx = cons.x+deltaX - pos.x;
    double dy = cons.y+deltaY - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    // Si la position cible est suffisamment éloignée
    if (d > erreurPos) {
        double rho = atan2(dy, dx);
        double angleDifference = reductionAngle(rho - pos.a);

        // Si l'angle entre la direction actuelle et la direction cible est trop grand, tourner sur place
        if (fabs(angleDifference) > angleThreshold) {
            double consw = kw * angleDifference;
            consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;

            consvl = -consw;  // Roue gauche tourne dans une direction
            consvr = consw;   // Roue droite tourne dans la direction opposée
        } else {
            // Si l'angle est suffisamment petit, le robot peut avancer
            double consw = kw * angleDifference;
            double consv = kv * d * cos(angleDifference);

            consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
            consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

            consvl = consv - gladiator->robot->getRobotRadius() * consw;
            consvr = consv + gladiator->robot->getRobotRadius() * consw;
        }
    } else {
        // Si la position est proche de la cible, arrêter le robot
        consvr = 0;
        consvl = 0;
    }

    // Appliquer les vitesses aux roues
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
}

void resetLists() {
    // Réinitialiser la file d'attente
    std::queue<const MazeSquare*> emptyQueue;
    std::swap(q, emptyQueue); // Échanger la queue actuelle avec une queue vide
    
    // Réinitialiser l'ensemble des cases visitées
    visited = std::unordered_set<const MazeSquare*>();
    
    // Réinitialiser le dictionnaire des parents
    parent = std::unordered_map<const MazeSquare*, const MazeSquare*>();
    
    // Réinitialiser le chemin
    path = std::vector<const MazeSquare*>();
}


void reset() {
    gladiator->log("Appel de la fonction de reset");
    start = std::chrono::system_clock::now();
    resetLists();
}

void setup() {
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

bool isValid(const MazeSquare* square, std::unordered_set<const MazeSquare*>& visited) {
    // Vérifier que la case n'est pas nullptr et n'a pas déjà été visitée
    if (square == nullptr || visited.find(square) != visited.end()) {
        return false;
    }

    // Vérifier si la valeur de danger est supérieure à 4, si c'est le cas, la case est invalide
    if (square->danger > 4) {
        return false;
    }
    if (!CheckFutureCase(square->i,square->j, 6)){
        return false;
    }

    return true;
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
        // Vérifier si la case suivante est invalide avant de se déplacer
        if (!CheckFutureCase(target->i, target->j, 6)) {
            // Si la case devient invalide, recalculer le chemin
            gladiator->log("Target case invalid, recalculating path.");
            path = bfsToTarget(gladiator->maze->getNearestSquare(), target);  // Recalcul du chemin
            if (path.empty()) {
                gladiator->log("No valid path found.");
                return;  // Si le chemin est complètement invalide, arrêter
            }
            // Prendre la première étape du nouveau chemin
            target = path[0];
            continue;  // Passer à l'étape suivante
        }

        // Se déplacer vers la case cible
        go_to(targetCoor, myPosition);
        myPosition = gladiator->robot->getData().position; // Mettre à jour la position actuelle
        delay(100); // Ajouter un délai pour permettre au robot de se déplacer
    }
}


std::vector<const MazeSquare*> bfsToTarget(const MazeSquare* start, const MazeSquare* target) {
    visited.clear();
    if (start == nullptr || target == nullptr) {
        gladiator->log("Start or target square is null");
        return {};
    }

    q.push(start);
    visited.insert(start);
    parent[start] = nullptr;

    while (!q.empty()) {
        const MazeSquare* current = q.front();
        q.pop();

        //gladiator->log("Visiting square at (%d, %d)", current->i, current->j);

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

void convert(unsigned int i, unsigned int j) {
    float squareSize = gladiator->maze->getSquareSize();

    Position centerCoor;

    centerCoor.x = (i + 0.5) * squareSize;
    centerCoor.y = (j + 0.5) * squareSize;

    Position myPosition = gladiator->robot->getData().position;
    Position goal{centerCoor.x, centerCoor.y, 0};
    go_to(goal, myPosition);
}

bool CheckFutureCase(int i, int j,int futur_time) {
    Num_tour = floor((time_elapsed + futur_time) / 20.0f); // Arrondi à l'inférieur

    // Récupérer la taille réelle du labyrinthe en cases
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = floor(MazeSize / squareSize) - 1;

    // Log pour débogage
    gladiator->log("Time: %f | Tour Number: %d | MazeSize: %d | Limits: (%d, %d)", time_elapsed, Num_tour, MazeSize_int, Num_tour, 11 - Num_tour);

    // Vérification des limites du labyrinthe
    if (i <= Num_tour - 1 || i >= 12 - Num_tour || j <= Num_tour - 1 || j >= 12 - Num_tour) {
        return false;  // Ne pas explorer cette case
    }
    return true;  // La case est valide
}

void BombListing() {
    int index = 0;

    // Réinitialiser la liste des bombes
    for (int i = 0; i < MAX_BOMB; i++) {
        BombPos[i].x = -1;
        BombPos[i].y = -1;
    }
    float squareSize = gladiator->maze->getSquareSize();

    // Parcours du labyrinthe (ajustez la taille si nécessaire)
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = static_cast<int>(MazeSize / squareSize);
    gladiator->log("MazeSize %d | Limits: (%d, %d)", MazeSize_int, Num_tour, 11 - Num_tour);

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            const MazeSquare* indexedSquare = gladiator->maze->getSquare(i, j);
            Coin coin = indexedSquare->coin;
            int danger = indexedSquare->danger;
            if (coin.value < 1) {
                continue;
            }
            // Calcul de la position réelle de la bombe
            int i_bomb = static_cast<int>((coin.p.x / squareSize) - 0.5);
            int j_bomb = static_cast<int>((coin.p.y / squareSize) - 0.5);

            // Vérification si la case est une limite
            if (!CheckFutureCase(i_bomb, j_bomb,6)) {
                gladiator->log("Bomb out of bounds (%d, %d)", i_bomb, j_bomb);
                continue;  // Ignorer les cases sur les bords
            }

            // Vérification si la bombe est valide
            gladiator->log("Bomb data (%d, %d)", coin.value, danger);

            if (coin.value > 0 && danger < 1) {
                Position posCoin = coin.p;
                if (index < MAX_BOMB) {
                    BombPos[index] = posCoin;  // Ajouter la bombe à la liste
                    index++;  // Incrémenter l'indice
                }
            }
        }
    }
}

Position FindNearestBomb() {
    RobotData myData = gladiator->robot->getData();
    double minDistance = 9999;
    int compteurBombes = 0;
    BombListing();
    for (int i = 0; i < MAX_BOMB; i++) {
        if (BombPos[i].x != -1 && BombPos[i].y != -1) {
            compteurBombes++;
            float squareSize = gladiator->maze->getSquareSize();

            // Calcul de la distance en nombre de cases
            int i_bomb = static_cast<int>((BombPos[i].x / squareSize) - 0.5);
            int j_bomb = static_cast<int>((BombPos[i].y / squareSize) - 0.5);
            int myPos_i = static_cast<int>((myData.position.x / squareSize) - 0.5);
            int myPos_j = static_cast<int>((myData.position.y / squareSize) - 0.5);

            int distance = abs(i_bomb - myPos_i) + abs(j_bomb - myPos_j);

            if (distance < minDistance) {
                minDistance = distance;
                targetBomb = BombPos[i];
            }
        }
    }

    float squareSize = gladiator->maze->getSquareSize();

    // Calcul de la position réelle de la bombe
    int i_bomb = static_cast<int>((targetBomb.x / squareSize) - 0.5);
    int j_bomb = static_cast<int>((targetBomb.y / squareSize) - 0.5);

    int myPos_i = static_cast<int>((myData.position.x / squareSize) - 0.5);
    int myPos_j = static_cast<int>((myData.position.y / squareSize) - 0.5);
    gladiator->log("Bombs detected: %d | Nearest bomb at (%d, %d) | My position: (%d, %d) | Distance: %f", compteurBombes, i_bomb, j_bomb, myPos_i, myPos_j, minDistance);

    return targetBomb;
}

void dropBomb() {
    int bombcount = gladiator->weapon->getBombCount();
    if (gladiator->weapon->canDropBombs(1)) {
        // Drop a bomb
        gladiator->weapon->dropBombs(bombcount);
    }
}

void CheckBombStatus() {
    float squareSize = gladiator->maze->getSquareSize();

    // Calcul de la position de la bombe
    int i_bomb = (LastBombToGet.x / squareSize) - 0.5;
    int j_bomb = (LastBombToGet.y / squareSize) - 0.5;
    const MazeSquare* indexedSquare = gladiator->maze->getSquare(i_bomb, j_bomb);
    Coin coin = indexedSquare->coin;
    int danger = indexedSquare->danger;

    if (coin.value < 1 || danger > 2) {
        UpdateNearestBomb = true;
    }
}

void loop() {
    if (gladiator->game->isStarted()) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        time_elapsed = elapsed_seconds.count();
        if (time_elapsed >1000){
            time_elapsed=0;
        }

        dropBomb();

        if (UpdateNearestBomb) {
            targetBomb = FindNearestBomb();
        }

        if (targetBomb.x != -1 && targetBomb.y != -1) {
            LastBombToGet = targetBomb;
            UpdateNearestBomb = false;
        }

        CheckBombStatus(); // Vérifie si on peut toujours aller à la bombe cible

        gladiator->log("Le jeu a commencé");

        const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
        if (nearestSquare == nullptr) {
            gladiator->log("Nearest square is null");
            return;
        }
        float squareSize = gladiator->maze->getSquareSize();

        // Calcul de la position de la bombe
        int i_LastBombToGet = (LastBombToGet.x / squareSize) - 0.5;
        int j_LastBombToGet = (LastBombToGet.y / squareSize) - 0.5;
        unsigned char targetI = i_LastBombToGet, targetJ = j_LastBombToGet;
        gladiator->log("Target square with nearest bomb = %d,%d", i_LastBombToGet, j_LastBombToGet);

        const MazeSquare* targetSquare = gladiator->maze->getSquare(targetI, targetJ);
        if (targetSquare == nullptr) {
            gladiator->log("Target square is null");
            return;
        }

        path = bfsToTarget(nearestSquare, targetSquare);
        if (!path.empty()) {
            for (const MazeSquare* step : path) {
                moveTo(step);
            }
            gladiator->log("Cible atteinte!");
        } else {
            gladiator->log("Aucun chemin trouvé vers la cible.");
            Position Goal{1.5,1.5,0};
            Position myPosition = gladiator->robot->getData().position;
            go_to(Goal,myPosition);
        }

        // Réinitialiser les structures de données après chaque tentative
        resetLists();

        delay(100);
    }
}
