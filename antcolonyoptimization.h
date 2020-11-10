#ifndef ANTCOLONYOPTIMIZATION_H
#define ANTCOLONYOPTIMIZATION_H

#include "AntColonyOptimization_global.h"
#include "ipathfinder.h"
#include <map>
#include <iostream>

using namespace std;

struct nodenumpair{

    public:
    nodenumpair(int from_,int to_):from(from_),to(to_){}
    nodenumpair(){}
    int from,to;

    bool operator==(const nodenumpair& rhs) const{
        if(this->from == rhs.from && this->to == rhs.to)
            return true;
        return false;
    }
    bool operator<(const nodenumpair &rhs)const {
       return this->from < rhs.from ||  this->to < rhs.to;
    }

};

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
    int RouletteWheelSelect(const vector<double>& ijprobabilites);
    Point ChooseNextNode(const Point& temp);
    void SetNegativeValueAroundPoint(const int& i, const int& j); //not needed
    bool IsCellOkey();
    void CalculateProbability(const vector<double>& selectables,const vector<Point>& SelectableNodes,vector<double> &ijprobabilites, const Point& from);
    void LocalPheromoneUpdate(const vector<Point> &path);
    void GlobalPheromoneUpdate();
    double UpdateEvaporationFormula(const int& fromNodeNum,const int& toNodeNum);
    void DrawSolution(const int& colornum,const vector<Point>& Path);
    double GlobalPheromoneFormula(double tauij,double sumdeltatauijk);
    void SumDeltaTauijk(const vector<Point>& Path);
    void ComulativeSum(vector<double>& selectables);
    void printPheromoneMatrix();
    void IterationBestPathUpdate();
    double OfflineUpdateFormula(double tauij, double deltatauij);
    void SaveSixBest(const vector<Point>& Path);

    double Eta(const Point& i,const Point& j);
    double eucledianDistance(const double& x1, const double& y1, const double& x2, const double& y2) {
        return sqrt(pow((x2-x1),2) + pow((y2 - y1), 2));
    }

private:
    /* Parameters */
    double tauzero;
    double phi;
    double alpha;
    double beta;
    double theta;
    double kappa;


    int dstNodeNum;
    int numberOfAnts;

    int numberOfIterations;
    int numberofgridcells;
    double* PHEROMONCOSTMATRIX;
    bool* VISITED;
    vector<Point> ShortestPath;
    vector<Point> IterationBestPath;
    int shortestPathAntId;
    map<nodenumpair,double> NodeNumPairMap;
    long int ShortestLenght;
    int bestAntId;
    int IterationBestLength;

};

#endif // ANTCOLONYOPTIMIZATION_H
