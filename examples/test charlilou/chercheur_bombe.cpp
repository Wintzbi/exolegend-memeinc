#include "gladiator.h"
Gladiator *gladiator;
void reset();

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.6;
float erreurPos = 0.07;
typedef struct PathList {
    int x, y;
    int value;
} PathList;

PathList* pathList;

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
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
            if (coin.value > 0){
                    Position posCoin = coin.p;
                if ( index < MAX_BOMB) {
                    BombPos[index] = posCoin;  // Ajout à la liste
                    index++;  // Incrémentation de l'indice
                }
                    gladiator->log("position pièce : ( %f; %f )", posCoin.x, posCoin.y);
            }
        }
    }
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
        RobotData myData = gladiator->robot->getData();
        Position targetBomb = { -1, -1 };
        double minDistance = 9999;

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

        if (targetBomb.x != -1 && targetBomb.y != -1) {
            double dx = targetBomb.x - myData.position.x;
            double dy = targetBomb.y - myData.position.y;
            double targetAngle = atan2(dy, dx);

            double angleDiff = targetAngle - myData.position.a;
            if (angleDiff > M_PI) angleDiff -= 2 * M_PI;
            if (angleDiff < -M_PI) angleDiff += 2 * M_PI;

            double speed = myData.speedLimit * 0.8;
            double turnSpeed = angleDiff * 2.0;

            double vl = speed - turnSpeed;
            double vr = speed + turnSpeed;

            gladiator->control->setWheelSpeed(WheelAxis::LEFT, vl);
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, vr);
        }

        gladiator->log("Tracking bomb at (%f, %f)", targetBomb.x, targetBomb.y);
        delay(100);
    }
}
