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

void BombListing() {
    for(int i=0;i<=11;i++){
        for(int j=0;j<=11;j++){
            const MazeSquare *indexedSquare = gladiator->maze->getSquare(i, j);
            Coin coin = indexedSquare->coin;
            if (coin.value > 0){
                    Position posCoin = coin.p;
            }
        }
    }
}

int getDistance(unsigned int i, unsigned int j, unsigned int x, unsigned int y){
    int distance = sqrt((i -x) * (i - x) +
                                    (j - y) * (j - y));
    return distance;
}

void move_nearest(){
    Position myPosition = gladiator->robot->getData().position;
    const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();

    MazeSquare* neighbors[4] = {nearestSquare->northSquare, nearestSquare->westSquare,
                                    nearestSquare->eastSquare, nearestSquare->southSquare};

    int minIndex = -1;
    int minDistance = std::numeric_limits<unsigned int>::max();
        
    for (int i = 0; i < 4; ++i) {
        if (neighbors[i] != nullptr) {
            int distance = getDistance(neighbors[i]->i, neighbors[i]->j, 5, 6);
            if (distance < minDistance) {
                    minDistance = distance;
                    minIndex = i;
                }
            } else if (minIndex == -1) {
                minIndex = i;
            }
        }

        // Déplacement vers la case la plus proche
        if (minIndex != -1) {
            convert(neighbors[minIndex]->i,neighbors[minIndex]->j);
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
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        float mazeSize = gladiator->maze->getCurrentMazeSize();
        const MazeSquare *nearestSquare = gladiator->maze->getNearestSquare();

        gladiator->log("Hello world - Game Started"); // GFA 4.5.1
        gladiator->log("mazeSize: %f", mazeSize);
        gladiator->log("cordonnée actuelle: %u, %u", nearestSquare->i, nearestSquare->j);
        move_nearest();

    }
    else
    {
        gladiator->log("Hello world - Game not Startd yet"); // GFA 4.5.1
    }
    delay(300);
}
