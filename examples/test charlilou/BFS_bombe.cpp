#include "gladiator.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <chrono>

#define deltaX 0.0016
#define deltaY -0.0027
#define FUTURE_TIME 6

std::chrono::time_point<std::chrono::system_clock> start, end;
float time_elapsed;
int Num_tour = 0;
Gladiator *gladiator;

void reset();
void dropBomb();
bool UpdateNearestBomb = true;
Position LastBombToGet;
Position targetBomb = { -1, -1 };
Position targetSafe = { -1, -1 };

#define MAX_BOMB 50
#define MAX_SAFE 50

Position BombPos[MAX_BOMB];
Position SafePos[MAX_SAFE];

bool CheckFutureCase(int i, int j, int futur_time);
std::vector<const MazeSquare*> bfsToTarget(const MazeSquare* start, const MazeSquare* target);

float kw = 0.12f;
float kv = 0.25f;
float wlimit = 0.20f;
float vlimit = 1.f;
float erreurPos = 0.10f;
float angleThreshold = 0.5f;

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

void go_to(Position cons, Position pos, float vitesse_const) {
    double consvl, consvr;
    double dx = cons.x + deltaX - pos.x;
    double dy = cons.y + deltaY - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos) {
        double rho = atan2(dy, dx);
        double angleDifference = reductionAngle(rho - pos.a);

        if (fabs(angleDifference) > angleThreshold) {
            double consw = kw * angleDifference;
            consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;

            consvl = -consw;
            consvr = consw;
        } else {
            consvr = vitesse_const;
            consvl = vitesse_const;
            dropBomb();
        }
    } else {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
}

void resetLists() {
    std::queue<const MazeSquare*> emptyQueue;
    std::swap(q, emptyQueue);
    visited.clear();
    parent.clear();
    path.clear();
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
    if (square == nullptr || visited.find(square) != visited.end() || square->danger > 4 || !CheckFutureCase(square->i, square->j, FUTURE_TIME)) {
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
        if (!CheckFutureCase(target->i, target->j, FUTURE_TIME)) {
            gladiator->log("Target case invalid, recalculating path.");
            path = bfsToTarget(gladiator->maze->getNearestSquare(), target);
            if (path.empty()) {
                gladiator->log("No valid path found.");
                return;
            }
            target = path[0];
            continue;
        }

        go_to(targetCoor, myPosition, kv);
        myPosition = gladiator->robot->getData().position;
        delay(100);
    }
}

std::vector<const MazeSquare*> bfsToTarget(const MazeSquare* start, const MazeSquare* target) {
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

        if (current == target) {
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
    Position centerCoor = {(i + 0.5) * squareSize, (j + 0.5) * squareSize, 0};
    Position myPosition = gladiator->robot->getData().position;
    go_to(centerCoor, myPosition, kv);
}

bool CheckFutureCase(int i, int j, int futur_time) {
    Num_tour = floor((time_elapsed + futur_time) / 20.0f);
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = floor(MazeSize / squareSize) - 1;

    if (i <= Num_tour - 1 || i >= 12 - Num_tour || j <= Num_tour - 1 || j >= 12 - Num_tour) {
        return false;
    }
    return true;
}

void BombListing() {
    int index = 0;
    for (int i = 0; i < MAX_BOMB; i++) {
        BombPos[i].x = -1;
        BombPos[i].y = -1;
    }
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = static_cast<int>(MazeSize / squareSize);
    gladiator->log("MazeSize %d | Limits: (%d, %d)", MazeSize_int, Num_tour, 11 - Num_tour);

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            const MazeSquare* indexedSquare = gladiator->maze->getSquare(i, j);
            if (indexedSquare == nullptr) continue;

            Coin coin = indexedSquare->coin;
            int danger = indexedSquare->danger;
            if (coin.value < 1) continue;

            int i_bomb = static_cast<int>((coin.p.x / squareSize) - 0.5);
            int j_bomb = static_cast<int>((coin.p.y / squareSize) - 0.5);

            if (!CheckFutureCase(i_bomb, j_bomb, FUTURE_TIME)) {
                gladiator->log("Bomb out of bounds (%d, %d)", i_bomb, j_bomb);
                continue;
            }

            gladiator->log("Bomb data (%d, %d)", coin.value, danger);

            if (coin.value > 0 && danger < 1) {
                if (index < MAX_BOMB) {
                    BombPos[index] = coin.p;
                    index++;
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
        gladiator->weapon->dropBombs(bombcount);
    }
}

void CheckBombStatus() {
    float squareSize = gladiator->maze->getSquareSize();
    int i_bomb = (LastBombToGet.x / squareSize) - 0.5;
    int j_bomb = (LastBombToGet.y / squareSize) - 0.5;
    const MazeSquare* indexedSquare = gladiator->maze->getSquare(i_bomb, j_bomb);
    if (indexedSquare == nullptr) return;

    Coin coin = indexedSquare->coin;
    int danger = indexedSquare->danger;

    if (coin.value < 1 || danger > 2) {
        UpdateNearestBomb = true;
    }
}

void SafeListing() {
    int index = 0;
    for (int i = 0; i < MAX_SAFE; i++) {
        SafePos[i].x = -1;
        SafePos[i].y = -1;
    }
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = static_cast<int>(MazeSize / squareSize);
    gladiator->log("MazeSize %d | Limits: (%d, %d)", MazeSize_int, Num_tour, 11 - Num_tour);

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            const MazeSquare* indexedSquare = gladiator->maze->getSquare(i, j);
            if (indexedSquare == nullptr) continue;

            int danger = indexedSquare->danger;
            if (danger < 1) {
                int i_safe = indexedSquare->i;
                int j_safe = indexedSquare->j;

                if (!CheckFutureCase(i_safe, j_safe, 1)) {
                    gladiator->log("Safe out of bounds (%d, %d)", i_safe, j_safe);
                    continue;
                }

                if (index < MAX_SAFE) {
                    SafePos[index].x = i_safe;
                    SafePos[index].y = j_safe;
                    index++;
                }
            }
        }
    }
}

Position FindNearestSafe() {
    RobotData myData = gladiator->robot->getData();
    double minDistance = 9999;
    int compteurSafe = 0;
    SafeListing();
    for (int i = 0; i < MAX_SAFE; i++) {
        if (SafePos[i].x != -1 && SafePos[i].y != -1) {
            compteurSafe++;
            float squareSize = gladiator->maze->getSquareSize();

            int i_safe = SafePos[i].x;
            int j_safe = SafePos[i].y;
            int myPos_i = static_cast<int>((myData.position.x / squareSize) - 0.5);
            int myPos_j = static_cast<int>((myData.position.y / squareSize) - 0.5);

            int distance = abs(i_safe - myPos_i) + abs(j_safe - myPos_j);

            if (distance < minDistance) {
                minDistance = distance;
                targetSafe.x = i_safe;
                targetSafe.y = j_safe;
            }
        }
    }

    float squareSize = gladiator->maze->getSquareSize();
    int i_safe = targetSafe.x;
    int j_safe = targetSafe.y;
    int myPos_i = static_cast<int>((myData.position.x / squareSize) - 0.5);
    int myPos_j = static_cast<int>((myData.position.y / squareSize) - 0.5);
    gladiator->log("Safe detected: %d | Nearest safe at (%d, %d) | My position: (%d, %d) | Distance: %f", compteurSafe, i_safe, j_safe, myPos_i, myPos_j, minDistance);

    return targetSafe;
}

bool checkRobotDanger() {
    float squareSize = gladiator->maze->getSquareSize();
    Position myPosition = gladiator->robot->getData().position;
    int myPos_i = static_cast<int>((myPosition.x / squareSize) - 0.5);
    int myPos_j = static_cast<int>((myPosition.y / squareSize) - 0.5);
    const MazeSquare* mySquare = gladiator->maze->getSquare(myPos_i, myPos_j);
    if (mySquare == nullptr) return true;

    int danger = mySquare->danger;
    return !CheckFutureCase(myPos_i, myPos_j, 1) || danger > 1;
}

void loop() {
    if (gladiator->game->isStarted()) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        time_elapsed = elapsed_seconds.count();
        if (time_elapsed > 1000) {
            time_elapsed = 0;
        }

        if (UpdateNearestBomb) {
            targetBomb = FindNearestBomb();
        }

        if (targetBomb.x != -1 && targetBomb.y != -1) {
            LastBombToGet = targetBomb;
            UpdateNearestBomb = false;
        }

        CheckBombStatus();
        const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
        if (nearestSquare == nullptr) {
            gladiator->log("Nearest square is null");
            return;
        }

        float squareSize = gladiator->maze->getSquareSize();
        int i_LastBombToGet = (LastBombToGet.x / squareSize) - 0.5;
        int j_LastBombToGet = (LastBombToGet.y / squareSize) - 0.5;
        unsigned char targetI = i_LastBombToGet, targetJ = j_LastBombToGet;
        gladiator->log("Target square with nearest bomb = %d,%d", i_LastBombToGet, j_LastBombToGet);

        const MazeSquare* targetSquare = gladiator->maze->getSquare(targetI, targetJ);
        if (targetSquare == nullptr) {
            gladiator->log("Target square is null");
            return;
        }
        Position myPosition = gladiator->robot->getData().position;

        path = bfsToTarget(nearestSquare, targetSquare);
        if (!path.empty()) {
            for (const MazeSquare* step : path) {
                if (CheckFutureCase(step->i, step->j, 1) && !checkRobotDanger()) {
                    CheckBombStatus();
                    moveTo(step);
                } else {
                    gladiator->log("================++AAAAAAAHHHHH JE FUIS======================");
                    FindNearestSafe();
                    go_to({targetSafe.x, targetSafe.y}, myPosition, 0.8);
                    path = bfsToTarget(nearestSquare, targetSquare);
                    break;
                }
            }
            gladiator->log("Cible atteinte!");
        } else {
            gladiator->log("Aucun chemin trouvé vers la cible.");
            go_to({targetSafe.x, targetSafe.y}, myPosition, 0.8);
        }

        resetLists();
        delay(100);
    }
}
