#include "gladiator.h"
#include <chrono>
#include <cmath>
#include <iostream>
//////////////CODE THOMAS////////////////////
#include "vector2.hpp"
#include <vector>
#include <array>

void retreat();
void arme_fou(int duree);
/////////////////////////////////////////////

std::chrono::time_point<std::chrono::system_clock> start, end;
float time_elapsed;
int Num_tour=0;
Gladiator *gladiator;
void reset();
void dropBomb();
bool UpdateNearestBomb=true;
Position LastBombToGet;
Position targetBomb = { -1, -1 };
int forwardDanger=-1;
int myDanger=0;
#define MAX_BOMB 50
Position BombPos[MAX_BOMB];

// Constantes de contrôle



float kw = 1.75;
float kv = 1.f;
float wlimit = 3.5f;
float vlimit = 0.8;
float erreurPos = 0.10;
float angleThreshold=0.6;


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
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

Position convert(unsigned int i, unsigned int j) {
    float squareSize = gladiator->maze->getSquareSize();

    Position centerCoor;

    centerCoor.x = (i + 0.5) * squareSize;
    centerCoor.y = (j + 0.5) * squareSize;

    Position myPosition = gladiator->robot->getData().position;
    Position goal{centerCoor.x, centerCoor.y, 0};
    return goal;
    }

bool CheckFuturCase(int i, int j,int time){
    Num_tour = floor((time_elapsed+time ) / 20.f); // Arrondi à l'inférieur

    // Récupérer la taille réelle du labyrinthe en cases
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = floor(MazeSize / squareSize)-1;

    // Log pour débogage
    //gladiator->log("TIme : %f | Numéro tour %d | MazeSize %d | Limites : (%d, %d)",time_elapsed, Num_tour,MazeSize_int,Num_tour,11 - Num_tour);

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

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            const MazeSquare* indexedSquare = gladiator->maze->getSquare(i, j);
            Coin coin = indexedSquare->coin;
            int danger = indexedSquare->danger;
            if(coin.value < 1){
                continue;
            }
            // Calcul de la position réelle de la bombe
            int i_bomb = static_cast<int>((coin.p.x / squareSize) - 0.5);
            int j_bomb = static_cast<int>((coin.p.y / squareSize) - 0.5);

            // Vérification si la case est une limite
            if (!CheckFuturCase(i_bomb, j_bomb, 10)) {
                continue;  // Ignorer les cases sur les bords
            }

            // Vérification si la bombe est valide
            if (coin.value > 0 && danger < 1) {
                Position posCoin = coin.p;

                // Vérification si la bombe est à une case de la dernière bombe déposée
                int lastBomb_i = static_cast<int>((LastBombToGet.x / squareSize) - 0.5);
                int lastBomb_j = static_cast<int>((LastBombToGet.y / squareSize) - 0.5);
                if (abs(i_bomb - lastBomb_i) <= 1 && abs(j_bomb - lastBomb_j) <= 1) {
                    continue;  // Ignorer les bombes à une case de la dernière bombe déposée
                }

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
    //gladiator->log("Bombs detecte : %d | Nearest bomb at (%d, %d)| Mypos : (%d,%d) | distance: %f", compteurBombes,i_bomb, j_bomb, myPOs_i,myPOs_j,minDistance);

    return targetBomb;
}

bool avoidDanger(const MazeSquare* neighbor, int value){
    if(neighbor->danger < value ){
        return 1;
    }
    return 0;
}

bool canGo(MazeSquare* neighbor){
    if(neighbor != nullptr && avoidDanger(neighbor, 1)){
        return 1;
    }
    return 0;
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
    // Initialisation du servo
    gladiator->weapon->initWeapon(WeaponPin::M1, WeaponMode::SERVO);
    gladiator->weapon->setTarget(WeaponPin::M1, 80);
}

void reset()
{

    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
    start = std::chrono::system_clock::now();


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

void boom(const MazeSquare* nearestSquare, unsigned char teamId){
    if (nearestSquare->possession != teamId && avoidDanger(nearestSquare,4)){
        //gladiator->log("TeamId: %u || Possession: %u", teamId, nearestSquare->possession);
        dropBomb();
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

void flee(const MazeSquare* nearestSquare, Position position){

    if(avoidDanger(nearestSquare, 3) != 1){
        go_to({1.5,1.5,0}, position);
    }
}
// Fonction pour obtenir la direction vers la cible
std::string getDirection(Position myPosition, Position targetPosition) {
    float deltaX = targetPosition.x - myPosition.x;
    float deltaY = targetPosition.y - myPosition.y;

    if (std::fabs(deltaX) > std::fabs(deltaY)) {
        return deltaX > 0 ? "right" : "left";
    } else {
        return deltaY > 0 ? "up" : "down";
    }
}

// Fonction pour obtenir la position devant le robot
Position getPositionDevant(Position myPosition, float squareSize, const std::string& direction) {
    Position positionDevant;

    // Calculer les indices de la case actuelle
    int myPosition_i = static_cast<int>(myPosition.x / squareSize);
    int myPosition_j = static_cast<int>(myPosition.y / squareSize);

    // Calculer les indices de la case devant le robot
    if (direction == "up") {
        positionDevant.x = myPosition_i;
        positionDevant.y = myPosition_j + 1;
    } else if (direction == "down") {
        positionDevant.x = myPosition_i;
        positionDevant.y = myPosition_j - 1;
    } else if (direction == "left") {
        positionDevant.x = myPosition_i - 1;
        positionDevant.y = myPosition_j;
    } else if (direction == "right") {
        positionDevant.x = myPosition_i + 1;
        positionDevant.y = myPosition_j;
    } else {
        return {-1, -1}; // Retourner une position invalide
    }
    //gladiator->log("Position du robot = %d,%d et devant le robot : (%f,%f)",myPosition_i,myPosition_j,positionDevant.x,positionDevant.y);

    positionDevant=convert(positionDevant.x,positionDevant.y);
    return positionDevant;
}

void Save(){
    Position myPosition = gladiator->robot->getData().position;

    float squareSize = gladiator->maze->getSquareSize();
        
    //Calcul position bombe
    int myPosition_i= (myPosition.x/squareSize)-0.5;
    int myPosition_j= (LastBombToGet.y/squareSize)-0.5;
    const MazeSquare* mySquare = gladiator->maze->getSquare(myPosition_i, myPosition_j);
    myDanger = mySquare->danger;
    std::string direction = getDirection(myPosition, LastBombToGet);
    Position positionDevant = getPositionDevant(myPosition, squareSize, direction);
    const MazeSquare* forwardSquare = gladiator->maze->getSquare(positionDevant.x, positionDevant.y);
    forwardDanger = forwardSquare->danger;
    
    //gladiator->log("Danger de ma case : %d | Danger de la case devant %d",myDanger,forwardDanger);
    if(forwardDanger>myDanger || !CheckFuturCase(positionDevant.x,positionDevant.y,2)){
        /*targetBomb=FindNearestBomb();
        LastBombToGet=targetBomb;
        UpdateNearestBomb=false;
        go_to({LastBombToGet.x,LastBombToGet.y,0},myPosition);
*/
        gladiator->log("ATTENTION LES JEUNES, GRENAAAADE");
        go_to({1.5,1.5,0},myPosition);

    }

}


////////////////////////////CODE THOMAS///////////////////////////////////

bool isRobotStuck()
{
    static Position lastPosition = gladiator->robot->getData().position;
    static unsigned long lastMoveTime = millis();

    Position currentPosition = gladiator->robot->getData().position;
    double dx = currentPosition.x - lastPosition.x;
    double dy = currentPosition.y - lastPosition.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance > 0.1)
    {
        lastPosition = currentPosition;
        lastMoveTime = millis();
    }

    return (millis() - lastMoveTime) > 2000;
}

void retreat()
{
    if (isRobotStuck())
    {
        gladiator->log("Robot bloqué, activation de la retraite !");
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, -0.4);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -0.4);
        delay(200);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, -0.4);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
        delay(200);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
    }
}
void arme_fou(int duree)
{
    while (duree > 0)
    {
        gladiator->weapon->setTarget(WeaponPin::M1, 135);
        delay(100);
        gladiator->weapon->setTarget(WeaponPin::M1, 50);
        delay(100);
        duree -= 200;
    }
    gladiator->weapon->setTarget(WeaponPin::M1, 95);
}



struct PositionEx
{
    Vector2 pos;
    bool isEnnemy;
    int life_ennemies;
};

std::vector<PositionEx> ennemies{};

std::array<PositionEx, 4>
fetchEnnemyRobotsData()
{
    std::array<PositionEx, 4> res{};
    RobotList robotsIds = gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; ++i)
    {
        RobotData other = gladiator->game->getOtherRobotData(robotsIds.ids[i]);
        res.at(i).isEnnemy = other.teamId != gladiator->robot->getData().teamId;
        res.at(i).pos = {other.position.x, other.position.y};
        res.at(i).life_ennemies = other.lifes;
    }
    return res;
}

PositionEx findNearest(std::vector<PositionEx> ennemies) // renvoie la chose la plus proche
{
    float min = 100000.f;
    PositionEx nearest{};
    const Position robotPos = gladiator->robot->getData().position;
    for (auto ennemy : ennemies)
    {
        const float dist = std::sqrt((robotPos.x - ennemy.pos.x()) * (robotPos.x - ennemy.pos.x()) + (robotPos.y - ennemy.pos.y()) * (robotPos.y - ennemy.pos.y()));
        if (dist < min)
        {
            min = dist;
            nearest = ennemy;
        }
    }
    return nearest;
}

Position getNearestEnnemy()
{ // renvoie l'énemie le plus proche
    ennemies.clear();
    const std::array<PositionEx, 4> robots = fetchEnnemyRobotsData();
    for (auto rpos : robots)
    {
        if (rpos.isEnnemy && rpos.life_ennemies)
        {
            // gladiator->log("état vie adverse :%f", rpos.life_ennemies);
            ennemies.push_back(rpos);
        }
    }

    if (ennemies.size() > 0)
    {
        // gladiator->log("ennemies size :%f", ennemies.size());
        PositionEx nearestEnnemy = findNearest(ennemies);
        return Position{nearestEnnemy.pos.x(), nearestEnnemy.pos.y()};
    }
    return {0.f};
}

bool limite_maze(Position target)
{
    if ((float)target.x > 0 &&
        (float)target.x < gladiator->maze->getSize() &&
        (float)target.y > 0 && (float)target.y < gladiator->maze->getSize())
    {
        gladiator->log("Cible dans le terrain");
        return true;
    }

    return false; // Added this line
}
void detectEnnemy(){
        Position myPosition = gladiator->robot->getData().position;

    /////////////////CODE THOMAS///////////////////////////////
            Position target = getNearestEnnemy();
    // Vérification de la distance
            double dx = target.x - myPosition.x;
            double dy = target.y - myPosition.y;
            double distance = sqrt(dx * dx + dy * dy);

            // Si l'ennemi est suffisamment proche, déclenche l'arme folle
            if (distance < 0.3) // Vous pouvez ajuster ce seuil
            {
                gladiator->log("Ennemi proche ! Activation de l'arme folle. 2");
                arme_fou(300);
                retreat(); // Vous pouvez ajuster la durée de l'arme folle si nécessaire
            }

    ///////////////////////////////////////////////////////////
}

void move_clean() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    time_elapsed =elapsed_seconds.count();
    //Save();
    //gladiator->log("Temps écoulé %f",time_elapsed);
    RobotData myData = gladiator->robot->getData();
    unsigned char teamId = myData.teamId;
    const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();

    boom(nearestSquare,teamId);

    if (UpdateNearestBomb){
            targetBomb=FindNearestBomb();
    }
    
    if (targetBomb.x != -1 && targetBomb.y != -1) {
        LastBombToGet=targetBomb;
        UpdateNearestBomb=false;
    }

    CheckBombStatuts(); // vérifie si on peut toujours aller à la bombe target
        Position myPosition = gladiator->robot->getData().position;

    detectEnnemy();


    go_to({LastBombToGet.x,LastBombToGet.y,0},myPosition);
    //gladiator->log("Tracking bomb at (%f, %f)", LastBombToGet.x, LastBombToGet.y);
}

void loop()
{   
    if (gladiator->game->isStarted()) {
        move_clean();
        delay(75);
    }
}
