#include "gladiator.h"
#include "vector2.hpp"
Gladiator *gladiator;
void reset();
bool UpdateNearestBomb=true;
Position LastBombToGet;
// Constantes de contrôle
float kw = 1.3;
float kv = 1.f;
float wlimit = 1.f;
float vlimit = 0.7;
float k_forward = 1.0;
float k_angular = 1.2;
float erreurPos = 0.07;
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
inline bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs)
{
    constexpr float ANGLE_REACHED_THRESHOLD = 0.1;
    constexpr float POS_REACHED_THRESHOLD = 0.05;

    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};

    Vector2 posError = target - pos;

    float targetAngle = posError.angle();
    float angleError = moduloPi(targetAngle - posRaw.a);

    bool targetReached = false;
    float leftCommand = 0.f;
    float rightCommand = 0.f;

    if (posError.norm2() < POS_REACHED_THRESHOLD) //
    {
        targetReached = true;
        UpdateNearestBomb=true;
    }
    else if (std::abs(angleError) > ANGLE_REACHED_THRESHOLD)
    {
        float factor = 0.2;
        if (angleError < 0)
            factor = -factor;
        rightCommand = factor;
        leftCommand = -factor;
    }
    else
    {
        float factor = 0.5;
        rightCommand = factor; //+angleError*0.1  => terme optionel, "pseudo correction angulaire";
        leftCommand = factor;  //-angleError*0.1   => terme optionel, "pseudo correction angulaire";
    }

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    if (showLogs || targetReached)
    {
        gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError,
                       target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}

void go_to(const Position &cons, const Position &pos)
{
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);
        double consv = kv * d * cos(reductionAngle(rho - pos.a));

        consw = std::min(static_cast<double>(my_max(consw, -wlimit)), static_cast<double>(wlimit)) * k_angular;
        consv = std::min(static_cast<double>(my_max(consv, -vlimit)), static_cast<double>(vlimit)) * k_forward;

        double consvl = consv - gladiator->robot->getRobotRadius() * consw;
        double consvr = consv + gladiator->robot->getRobotRadius() * consw;

        if (consvl == 0.0 && consvr == 0.0)
        {
            consvr = wlimit;
        }

        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
    }
    else
    {
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0, false);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0, false);
        UpdateNearestBomb=true;

    }
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
            if (coin.value > 0){
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
{   if (gladiator->weapon->canDropBombs(1)) {
            // Dropper une bombe
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }
    if (gladiator->game->isStarted()) {
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
        //aim(gladiator,{LastBombToGet.x,LastBombToGet.y},false);
        Position myPosition = gladiator->robot->getData().position;

        go_to({LastBombToGet.x,LastBombToGet.y,0},myPosition);
        gladiator->log("Tracking bomb at (%f, %f)", LastBombToGet.x, LastBombToGet.y);
        delay(100);
    }
}
