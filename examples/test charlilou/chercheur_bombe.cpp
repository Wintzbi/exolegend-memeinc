#include "gladiator.h"
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
#define MAX_BOMB 20
Position BombPos[MAX_BOMB];

// Constantes de contrôle

float kw = 0.4f;
float kv = 2.f;
float wlimit = 0.5f;
float vlimit = 1.5;
float erreurPos = 0.05;
float angleThreshold=0.1;
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
    dropBomb();
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
        UpdateNearestBomb = true;
    }

    // Appliquer les vitesses aux roues
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

bool CheckFuturCase(int i, int j){
    Num_tour = floor((time_elapsed+6 ) / 20.f); // Arrondi à l'inférieur

    // Récupérer la taille réelle du labyrinthe en cases
    float squareSize = gladiator->maze->getSquareSize();
    float MazeSize = gladiator->maze->getCurrentMazeSize();
    int MazeSize_int = floor(MazeSize / squareSize)-1;

    // Log pour débogage
    gladiator->log("TIme : %f | Numéro tour %d | MazeSize %d | Limites : (%d, %d)",time_elapsed, Num_tour,MazeSize_int,11 - Num_tour, Num_tour);

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

    for (int i = 0; i < MazeSize_int; i++) {
        for (int j = 0; j < MazeSize_int; j++) {
            const MazeSquare* indexedSquare = gladiator->maze->getSquare(i, j);
            Coin coin = indexedSquare->coin;
            int danger = indexedSquare->danger;

            // Calcul de la position réelle de la bombe
            int i_bomb = static_cast<int>((coin.p.x / squareSize) - 0.5);
            int j_bomb = static_cast<int>((coin.p.y / squareSize) - 0.5);
            if(Num_tour<4){ 
                if (!CheckFuturCase(i_bomb, j_bomb) ) {
                gladiator->log("Bombe hors limites (%d, %d) ", i_bomb, j_bomb);

                continue;  // Ignorer les cases sur les bords
            }
            }
            // Vérification si la case est une limite
            

            // Vérification si la bombe est valide
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
        float squareSize = gladiator->maze->getSquareSize();

    // Calcul de la position réelle de la bombe
    int i_bomb = static_cast<int>((targetBomb.x / squareSize) - 0.5);
    int j_bomb = static_cast<int>((targetBomb.y / squareSize) - 0.5);

    gladiator->log("Nearest bomb at (%d, %d), distance: %f", i_bomb, j_bomb, minDistance);

    return targetBomb;
}

bool avoidDanger(MazeSquare* neighbor){
    if(neighbor->danger <= 3){
        return 1;
    }
    return 0;
}

bool canGo(MazeSquare* neighbor){
    if(neighbor != nullptr && avoidDanger(neighbor)){
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
    if (nearestSquare->possession != teamId){
        if (gladiator->weapon->canDropBombs(1)) {
            dropBomb();
            gladiator->log("Drop bomb");
        }
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


void loop()
{   
    if (gladiator->game->isStarted()) {

        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        time_elapsed =elapsed_seconds.count();
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
        go_to({LastBombToGet.x,LastBombToGet.y,0},myPosition);
        //gladiator->log("Tracking bomb at (%f, %f)", LastBombToGet.x, LastBombToGet.y);
        delay(75);
    }
}
