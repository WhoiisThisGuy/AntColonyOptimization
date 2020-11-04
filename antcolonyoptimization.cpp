#include "antcolonyoptimization.h"
#include <iostream>
#include "string.h"
#include <time.h>

using namespace std;

AntColonyOptimization::AntColonyOptimization()
{
}

int AntColonyOptimization::StartSearch()
{

    for(int i = 0; i<numberOfIterations;++i){

        cout<<i<<". iteration starts"<<endl;

        //Finding country roads
        for(int antId = 0;antId<numberOfAnts;++antId){
            int result = false;
            while(!result){
                result = FindSolutionForAnt(antId);
                if(result == -1){
                    cout<<"Could not leave src point. Walled in?"<<endl;
                }
            }//result iteration
            cout<<"Ant "<<antId<<" found solution!"<<endl;

            //Draw solution for ant
        }//AntId iteration

    } //Main iteration

    return 0;
}

bool AntColonyOptimization::Init(vector<string> Parameters) //0 - number of ants, 1 - initial pheromone level, 2 - pheromoneEvaporationLevel, 3 - number of iterations
{
    //Init variables
    dstNodeNum = gridcontroller->dst.y*gridcontroller->numberOfColumns+gridcontroller->dst.x;

    numberofgridcells = gridcontroller->numberOfRows*gridcontroller->numberOfColumns;
    PHEROMONCOSTMATRIX = new double[numberofgridcells*numberofgridcells];
    PROCESSED = new bool[numberofgridcells];

    memset(PHEROMONCOSTMATRIX,0,numberofgridcells*sizeof(double));
    memset(PROCESSED,0,numberofgridcells*sizeof(bool));

    numberOfAnts =  stoi(Parameters[0]);
    initialPheromoneLevel =  stoi(Parameters[1]);
    pheromoneEvaporationLevel =  stoi(Parameters[2]);
    numberOfIterations = stoi(Parameters[3]);

    for(int i = 0;i<gridcontroller->numberOfRows;++i){
        for(int j = 0;j<gridcontroller->numberOfColumns;++j){
            if(i == j)
                continue;
            else if(gridcontroller->grid[i*gridcontroller->numberOfColumns+j] == -1){
                //Means this is a blocked cell so every edge into this cell has -1 level
                SetNegativeValueAroundPoint(i,j);
            }
            PHEROMONCOSTMATRIX[i*gridcontroller->numberOfColumns+j] = initialPheromoneLevel;
        }
    }

return true;
}


void AntColonyOptimization::SetNegativeValueAroundPoint(const int& i, const int& j)
{
    Point temp(j,i);

    int col,row;
    int tempnodenum = 0;
    int neighbournodenum = 0;
    col = gridcontroller->numberOfColumns;
    row = gridcontroller->numberOfRows;
    tempnodenum = temp.y*col+temp.x;

    if(temp.x != 0 && temp.y != 0
        && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 1
        && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 3)//top left
    {
        neighbournodenum = (temp.y-1)*col+(temp.x-1);
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
    if(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1
        && gridcontroller->grid[(temp.y-1)*col+temp.x] != 3)//top
    {
        neighbournodenum = (temp.y-1)*col+temp.x;
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
    if(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1
        && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 3)//top right
    {
        neighbournodenum = (temp.y-1)*col+(temp.x+1);
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
    if(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1
        && gridcontroller->grid[temp.y*col+(temp.x+1)] != 3)//right
    {
        neighbournodenum = temp.y*col+(temp.x+1);
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
    if(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1
        && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 3)//down right
    {
        neighbournodenum = (temp.y+1)*col+(temp.x+1);
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
    if(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1
        && gridcontroller->grid[(temp.y+1)*col+temp.x] != 3)//down
    {
        neighbournodenum = (temp.y+1)*col+temp.x;
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
    if(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1
        && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 3)//down left
    {
        neighbournodenum = (temp.y+1)*col+temp.x-1;
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
    if(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1
        && gridcontroller->grid[temp.y*col+temp.x-1] != 3)//left
    {
        neighbournodenum = temp.y*col+temp.x-1;
        PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum] = -1;
        PHEROMONCOSTMATRIX[neighbournodenum*col+tempnodenum] = -1;
    }
}

int AntColonyOptimization::FindSolutionForAnt(const int& antId)
{
    Point temp;
    Point chosenOne;
    //it all began on the src
    vector<Point> Path;
    Path.push_back(gridcontroller->src);
    temp = gridcontroller->src;

    while(temp.x != gridcontroller->dst.x && temp.y != gridcontroller->dst.y){

        if(!HasFreeNeighBour(temp)){//DeadEnd
            if(temp == gridcontroller->src)
                return -1; //Deadlock src is walled in
            else
                return false;//simply restart searching
        }
        //check pheromone levels around
        chosenOne = ChooseNextNode(temp); //get the next node from temp
        //Locally Update pheromone level
        LocalPheromoneUpdate(temp,chosenOne);
        temp = chosenOne;
        Path.push_back(temp);
    }

    GlobalPheromoneUpdate(Path);

    if(Path.size()<ShortestPath.size()){ //Save Shortest Path
        ShortestPath = Path;
        shortestPathAntId = antId;
    }

    return true; //solution found

}

bool AntColonyOptimization::HasFreeNeighBour(const Point& temp) //Check if it has anywhere to go
{
    int col,row;
    col = gridcontroller->numberOfColumns;
    row = gridcontroller->numberOfRows;

    if(
       (temp.x != 0 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 1)//top left
     ||(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1)//top
     ||(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1)//top right
     ||(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1)//right
     ||(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1)//down right
     ||(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1)//down
     ||(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1)//down left
     ||(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1)//left
       ) return true;

    return false;
}

int AntColonyOptimization::RouletteWheelSelect(const vector<double>& selectables)
{
    srand (time(NULL));
    double rnd = (rand() % 10 + 1)/(double)10;
    double offset = 0.0;

    for (int i = selectables.size(); i>=0 ; --i) {
        offset += selectables[i];
        if (rnd <= offset) {
            return i;
        }
    }
    //This point means all of them has the initial value so choose randomly a cell.
    int randint;

    randint = rand() % selectables.size() + 1;

    return randint;
}

Point AntColonyOptimization::ChooseNextNode(const Point& temp)
{
  vector<Point> SelectableNodes;
  vector<double> Selectables;

  int col,row;
  int tempnodenum = 0;
  int neighbournodenum = 0;

  col = gridcontroller->numberOfColumns;
  row = gridcontroller->numberOfRows;
  tempnodenum = temp.y*col+temp.x;

  if(temp.x != 0 && temp.y != 0
      && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 1)//top left
  {
      neighbournodenum = (temp.y-1)*col+(temp.x-1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y-1);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y-1));
  }
  if(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1)//top
  {
      neighbournodenum = (temp.y-1)*col+temp.x;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x,temp.y-1);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x,temp.y-1));
  }
  if(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1)//top right
  {
      neighbournodenum = (temp.y-1)*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y-1);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y-1));
  }
  if(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1)//right
  {
      neighbournodenum = temp.y*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y));
  }
  if(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1)//down right
  {
      neighbournodenum = (temp.y+1)*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y+1);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y+1));
  }
  if(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1)//down
  {
      neighbournodenum = (temp.y+1)*col+temp.x;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x,temp.y+1);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x,temp.y+1));
  }
  if(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1)//down left
  {
      neighbournodenum = (temp.y+1)*col+temp.x-1;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y+1);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y+1));
  }
  if(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1)//left
  {
      neighbournodenum = temp.y*col+temp.x-1;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y);
      Selectables.push_back(PHEROMONCOSTMATRIX[tempnodenum*col+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y));
  }

  CalculateProbability(Selectables); //Calculate the P(i,j) probabilites

  int theChosenOne = RouletteWheelSelect(Selectables); //Select one

  return SelectableNodes.at(theChosenOne); //Give it back

}

void AntColonyOptimization::CalculateProbability(vector<double> &Selectables)
{
    for (int i = 0; i < Selectables.size(); i++) { //Cumulative sum
        for (int j = i; j < Selectables.size(); j++) {
            if(i == j)
                continue;
             Selectables.at(i) += Selectables.at(j);
        }
    }
}


void AntColonyOptimization::LocalPheromoneUpdate(const Point& temp,const Point& chosen)
{
    int tempnodenum = temp.y*gridcontroller->numberOfColumns+temp.x;
    int neighbournodenum = chosen.y*gridcontroller->numberOfColumns+chosen.x;

    double tempvalue = PHEROMONCOSTMATRIX[tempnodenum*gridcontroller->numberOfColumns+neighbournodenum];
    double value = (1-pheromoneEvaporationLevel)*tempvalue+pheromoneEvaporationLevel*initialPheromoneLevel;

    PHEROMONCOSTMATRIX[tempnodenum*gridcontroller->numberOfColumns+neighbournodenum] = value;
    PHEROMONCOSTMATRIX[neighbournodenum*gridcontroller->numberOfColumns+tempnodenum] = value;
}

void AntColonyOptimization::GlobalPheromoneUpdate(const vector<Point> &path)
{

}

//Point AntColonyOptimization::checkAroundForDst(const Point &temp)
//{
//    int col,row;
//    Point dst = gridcontroller->dst;
//    col = gridcontroller->numberOfColumns;
//    row = gridcontroller->numberOfRows;

//    if(
//       (temp.x != 0 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] == 3)//top left
//     ||(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] == 3)//top
//     ||(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] == 3)//top right
//     ||(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] == 3)//right
//     ||(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] == 3)//down right
//     ||(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] == 3)//down
//     ||(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] == 3)//down left
//     ||(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] == 3)//left
//       ) return true;

//    return false;
//}

extern "C" ANTCOLONYOPTIMIZATION_EXPORT IPathfinder* InitPathfinderObject(){

    return new AntColonyOptimization();

}


