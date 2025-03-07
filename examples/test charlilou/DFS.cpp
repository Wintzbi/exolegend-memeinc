#include "gladiator.h"
#include <stack>
#include <set>

Gladiator *gladiator;

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.6;
float erreurPos = 0.07;

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

        consvl = consv - gladiator->robot->getRobotRadius() * consw;
        consvr = consv + gladiator->robot->getRobotRadius() * consw;
    }
    else
    {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
}

void dfs(const MazeSquare* start, const MazeSquare* target, std::stack<const MazeSquare*>& path)
{
    std::stack<const MazeSquare*> stack;
    std::set<const MazeSquare*> visited;
    std::unordered_map<const MazeSquare*, const MazeSquare*> came_from;
    stack.push(start);

    while (!stack.empty())
    {
        const MazeSquare* current = stack.top();
        stack.pop();

        if (current == target)
        {
            // Reconstruire le chemin
            for (const MazeSquare* at = current; at != nullptr; at = came_from[at])
            {
                path.push(at);
            }
            return;
        }

        visited.insert(current);

        // Explorer les voisins (nord, est, sud, ouest)
        const MazeSquare* neighbors[4] = {current->northSquare, current->eastSquare, current->southSquare, current->westSquare};
        for (const MazeSquare* neighbor : neighbors)
        {
            if (neighbor && visited.find(neighbor) == visited.end())
            {
                stack.push(neighbor);
                came_from[neighbor] = current;
            }
        }
    }
}

void reset()
{
    gladiator->log("Call of reset function");
}

void setup()
{
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        static std::stack<const MazeSquare*> path;
        static bool pathFound = false;

        if (!pathFound)
        {
            const MazeSquare *nearestSquare = gladiator->maze->getNearestSquare();
            const MazeSquare *targetSquare = gladiator->maze->getSquare(10, 10);

            if (targetSquare)
            {
                dfs(nearestSquare, targetSquare, path);
                pathFound = true;
            }
            else
            {
                gladiator->log("La case cible (10, 10) n'existe pas dans le labyrinthe.");
            }
        }

        if (!path.empty())
        {
            const MazeSquare* nextSquare = path.top();
            path.pop();

            Position myPosition = gladiator->robot->getData().position;
            float squareSize = gladiator->maze->getSquareSize();
            Position centerCoor;
            centerCoor.x = (nextSquare->i + 0.5) * squareSize;
            centerCoor.y = (nextSquare->j + 0.5) * squareSize;
            centerCoor.a = 0; // Assuming the angle is not needed for the target position

            go_to(centerCoor, myPosition);
        }

        delay(10);
    }
}
