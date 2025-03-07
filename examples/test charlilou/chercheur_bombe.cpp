#include "gladiator.h"
#include "vector2.hpp"
Gladiator *gladiator;
void reset();
bool UpdateNearestBomb=true;
Position LastBombToGet;
// Constantes de contrôle

float kw = 4.f;
float kv = 4.f;
float wlimit = 3.f;
float vlimit = 2;
float erreurPos = 0.05;

typedef struct PathList {
    int x, y;
    int value;
} PathList;

PathList* pathList;
template <typename T1, typename T2>
auto my_max(T1 a, T2 b) -> decltype(a + b)
{
    return (a > b) ? a : b;
}
double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}
inline float moduloPi(float a) // return angle in [-pi; pi]
{
    return (a < 0.0) ? (std::fmod(a - M_PI, 2 * M_PI) + M_PI) : (std::fmod(a + M_PI, 2 * M_PI) - M_PI);
}


void go_to(Position cons, Position pos)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);

        double consv = kv * d * cos(reductionAngle(rho - pos.a));
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
    }
    else
    {
        consvr = 0;
        consvl = 0;
        UpdateNearestBomb=true;

    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
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
#define MAX_BOMB 20
Position BombPos[MAX_BOMB];

void BombListing() {
    
    int index = 0;
    // Réinitialisation de la liste
    for (int i = 0; i < MAX_BOMB; i++) {
        BombPos[i].x = -1;
        BombPos[i].y = -1;
    }
    for(int i=0;i<=11;i++){
        for(int j=0;j<=11;j++){
            const MazeSquare *indexedSquare = gladiator->maze->getSquare(i, j);
            Coin coin = indexedSquare->coin;
            int danger =indexedSquare->danger;
            gladiator->log("Case : ( %f; %f ) , danger = %u", coin.p.x, coin.p.y,danger);
            if (coin.value > 0 && danger <1){
                    Position posCoin = coin.p;
                if ( index < MAX_BOMB) {
                    BombPos[index] = posCoin;  // Ajout à la liste
                    index++;  // Incrémentation de l'indice
                }
                    gladiator->log("position bombe : ( %f; %f )", posCoin.x, posCoin.y);
            }
        }
    }
    gladiator->log("Bomb listing updated.");

}


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
    if (gladiator->game->isStarted()) {
        if (gladiator->weapon->canDropBombs(1)) {
            // Dropper une bombe
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }
        RobotData myData = gladiator->robot->getData();
        Position targetBomb = { -1, -1 };
        double minDistance = 9999;

        if (UpdateNearestBomb){
                BombListing();
                for (int i = 0; i < MAX_BOMB; i++) {
                    if (BombPos[i].x != -1 && BombPos[i].y != -1) {
                        double dx = BombPos[i].x - myData.position.x;
                        double dy = BombPos[i].y - myData.position.y;
                        double distance = sqrt(dx * dx + dy * dy);
                        
                        if (distance < minDistance) {
                            minDistance = distance;
                            targetBomb = BombPos[i];
                            
                        }
                    }
                }
        }
        
        if (targetBomb.x != -1 && targetBomb.y != -1) {
            LastBombToGet=targetBomb;
            UpdateNearestBomb=false;
            gladiator->log("Nearest bomb at (%f, %f), distance: %f", targetBomb.x, targetBomb.y, minDistance);
            
            double dx = targetBomb.x - myData.position.x;
            double dy = targetBomb.y - myData.position.y;
            double targetAngle = atan2(dy, dx);

            
        }

         float squareSize = gladiator->maze->getSquareSize();
        int i_bomb= (LastBombToGet.x/squareSize)-0.5;
        int j_bomb= (LastBombToGet.y/squareSize)-0.5;
        const MazeSquare *indexedSquare = gladiator->maze->getSquare(i_bomb, j_bomb);
        Coin coin = indexedSquare->coin;
        int danger =indexedSquare->danger;
        gladiator->log("Case visée: ( %d; %d ) , danger = %u", i_bomb, j_bomb,danger);

        if (coin.value < 1 || danger >2){
                UpdateNearestBomb=true;
            }

        Position myPosition = gladiator->robot->getData().position;
        go_to({LastBombToGet.x,LastBombToGet.y,0},myPosition);
        gladiator->log("Tracking bomb at (%f, %f)", LastBombToGet.x, LastBombToGet.y);
        delay(100);
    }
}
