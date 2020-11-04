#ifndef ANTCOLONYOPTIMIZATION_H
#define ANTCOLONYOPTIMIZATION_H

#include "AntColonyOptimization_global.h"
#include "ipathfinder.h"


class AntColonyOptimization : public IPathfinder
{
public:
    AntColonyOptimization();
    virtual ~AntColonyOptimization();
    int StartSearch() override;
    bool Init(vector<string> Parameters) override;

private:
    int FindSolutionForAnt(const int& antId);
    bool HasFreeNeighBour(const Point& temp);
    bool CheckAroundForDst(const Point& temp);
    int RouletteWheelSelect(const vector<double>& selectables);
    Point ChooseNextNode(const Point& temp);
    void SetNegativeValueAroundPoint(const int& i, const int& j);
    bool IsCellOkey();
    void CalculateProbability(vector<double>& selectables);
    void LocalPheromoneUpdate(const vector<Point> &path);
    void GlobalPheromoneUpdate(const vector<Point>& path);
    double UpdateEvaporationFormula(const int& fromNodeNum,const int& toNodeNum);
    void DrawSolution(const int& colornum,const vector<Point>& Path);
private:

    int dstNodeNum;
    int numberOfAnts;
    double tauzero;
    double phi;
    int numberOfIterations;
    int numberofgridcells;
    double* PHEROMONCOSTMATRIX;
    bool* VISITED;
    vector<Point> ShortestPath;
    int shortestPathAntId;

};

#endif // ANTCOLONYOPTIMIZATION_H
