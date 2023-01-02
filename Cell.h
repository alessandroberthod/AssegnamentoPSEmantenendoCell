#ifndef CELL_H
#define CELL_H

#include<ostream>
//#include "Obstacle.h"
#include<vector>
//#include "Robot.h"
#include "Obstacle.h"
using std::vector;

class Cell{

public:
    //costruttore e costruttore di default
    Cell(double xC, double yC);
    Cell();

    //funzioni membro, che non modificano l'oggetto
    double xCell() const { return xC; };
    double yCell() const { return yC; };
    double distance_btw_cell (const Cell& c) const;
    double distance_currentrobotcell_goalrobotcell(const Cell& cgoal) const;
    double min_distance_currentrobotcell_one_obstacle_cells(double dimGrid, const Obstacle& obst) const;
    double min_distance_currentrobotcell_all_obstacles_cells(double dimGrid, const vector<Obstacle>& vecobst) const;
    double potential_tot_btw_currentgoalrobcells_currentrobobstcells(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const Cell& cgoal, const vector<Obstacle>& vecobst_pot) const;
    Cell path_planning_robot(double _eta, double _zeta, double _max_dist_infl, double dimGrid, const Cell& cgoal, const vector<Obstacle>& vecobst_pp) const;

private:
    double xC, yC;


};


#endif
