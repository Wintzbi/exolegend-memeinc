#include "gladiator.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "vector2.hpp"
#include <chrono>

std::chrono::time_point<std::chrono::system_clock> start, end;
float time_elapsed;
int Num_tour=0;
Gladiator *gladiator;
void reset();
void dropBomb();
bool UpdateNearestBomb=true;
Position LastBombToGet;
Position targetBomb = { -1, -1 };
#define MAX_BOMB 50
Position BombPos[MAX_BOMB];


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
    start = std::chrono::system_clock::now();

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

void convert(unsigned int i, unsigned int j) {
    float squareSize = gladiator->maze->getSquareSize();

    Position centerCoor;

    centerCoor.x = (i + 0.5) * squareSize;
    centerCoor.y = (j + 0.5) * squareSize;

    Position myPosition = gladiator->robot->getData().position;
    Position goal{centerCoor.x, centerCoor.y, 0};
    go_to(goal, myPosition);
}

bool CheckFuturCase(int i, int j){
    Num_tour = floor((time_elapsed+6 ) / 20.f); // Arrondi à l'inférieur

    // Récupérer la taille réelle du labyrinthe en cases
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = floor(MazeSize / squareSize)-1;

    // Log pour débogage
    gladiator->log("TIme : %f | Numéro tour %d | MazeSize %d | Limites : (%d, %d)",time_elapsed, Num_tour,MazeSize_int,Num_tour,11 - Num_tour);

    // Vérification des limites du labyrinthe
    if (i <= Num_tour-1 || i >= 12 - Num_tour  || j <= Num_tour-1 || j >= 12 - Num_tour ) {
        //gladiator->log("Bombe or limite");
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
    gladiator->log( "MazeSize %d | Limites : (%d, %d)",MazeSize_int,Num_tour,11 - Num_tour);

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            const MazeSquare* indexedSquare = gladiator->maze->getSquare(i, j);
            Coin coin = indexedSquare->coin;
            int danger = indexedSquare->danger;
            if(coin.value<1){
                continue;
            }
            // Calcul de la position réelle de la bombe
            int i_bomb = static_cast<int>((coin.p.x / squareSize) - 0.5);
            int j_bomb = static_cast<int>((coin.p.y / squareSize) - 0.5);
            
                // Vérification si la case est une limite
                if (!CheckFuturCase(i_bomb, j_bomb) ) {
                gladiator->log("Bombe hors limites (%d, %d) ", i_bomb, j_bomb);

                continue;  // Ignorer les cases sur les bords
            }
            
            

            // Vérification si la bombe est valide
            gladiator->log("Donnée de filtres(%d, %d) ", coin.value , danger);

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


Position FindNearestBomb(){
    RobotData myData = gladiator->robot->getData();
    double minDistance = 9999;
    int compteurBombes =0;
    BombListing();
    for (int i = 0; i < MAX_BOMB; i++) {
        if (BombPos[i].x != -1 && BombPos[i].y != -1) {
            compteurBombes++;
            double dx = BombPos[i].x - myData.position.x;
            double dy = BombPos[i].y - myData.position.y;
            double distance = sqrt(dx * dx + dy * dy);
            
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

    int myPOs_i = static_cast<int>((myData.position.x / squareSize) - 0.5);
    int myPOs_j = static_cast<int>((myData.position.y / squareSize) - 0.5);
    gladiator->log("Bombs detecte : %d | Nearest bomb at (%d, %d)| Mypos : (%d,%d) | distance: %f", compteurBombes,i_bomb, j_bomb, myPOs_i,myPOs_j,minDistance);

    return targetBomb;
}

void dropBomb(){
        int bombcount = gladiator->weapon->getBombCount();
        //gladiator->log("Nombre bombe : %d",bombcount);
        if (gladiator->weapon->canDropBombs(1)) {
            // Dropper une bombe
            gladiator->weapon->dropBombs(bombcount);
            //gladiator->log("Drop bomb");
        }
}
void CheckBombStatuts(){
        float squareSize = gladiator->maze->getSquareSize();
        
        //Calcul position bombe
        int i_bomb= (LastBombToGet.x/squareSize)-0.5;
        int j_bomb= (LastBombToGet.y/squareSize)-0.5;
        const MazeSquare *indexedSquare = gladiator->maze->getSquare(i_bomb, j_bomb);
        Coin coin = indexedSquare->coin;
        int danger =indexedSquare->danger;
        //gladiator->log("Case visée: ( %d; %d ) , danger = %u", i_bomb, j_bomb,danger);
   

        if (coin.value < 1 || danger >2){
                UpdateNearestBomb=true;
            }
}

void loop() {
    if (gladiator->game->isStarted()) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        time_elapsed =elapsed_seconds.count();
        //gladiator->log("Temps écoulé %f",time_elapsed);



        dropBomb();

        if (UpdateNearestBomb){
                targetBomb=FindNearestBomb();
        }
        
        
        if (targetBomb.x != -1 && targetBomb.y != -1) {
            LastBombToGet=targetBomb;
            UpdateNearestBomb=false;
        }

        CheckBombStatuts(); // vérifie si on peut toujours aller à la bombe target




        gladiator->log("Le jeu a commencé");

        const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
        if (nearestSquare == nullptr) {
            gladiator->log("Nearest square is null");
            return;
        }
        float squareSize = gladiator->maze->getSquareSize();
        
        //Calcul position bombe
        int i_LastBombToGet= (LastBombToGet.x/squareSize)-0.5;
        int j_LastBombToGet= (LastBombToGet.y/squareSize)-0.5;
        unsigned char targetI = i_LastBombToGet, targetJ = j_LastBombToGet;
        gladiator->log("Target square with nearest bomb = %d,%d",i_LastBombToGet,j_LastBombToGet);

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