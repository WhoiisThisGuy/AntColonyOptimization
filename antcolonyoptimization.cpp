#include "antcolonyoptimization.h"
#include <iostream>
#include "string.h"
#include <time.h>

using namespace std;

AntColonyOptimization::AntColonyOptimization()
{
}

AntColonyOptimization::~AntColonyOptimization()
{
    if(VISITED)
        delete [] VISITED;
    if(PHEROMONCOSTMATRIX)
        delete [] PHEROMONCOSTMATRIX;
}

int AntColonyOptimization::StartSearch()
{

    cout<<"Starting search!"<<endl;
    for(int i = 0; i<numberOfIterations;++i){

        cout<<i<<". iteration starts"<<endl;

        //Finding country roads
        for(int antId = 0;antId<numberOfAnts;++antId){
            int result = false;
            while(!result){
                memset(VISITED,0,numberofgridcells*sizeof(bool));
                result = FindSolutionForAnt(antId);
                if(result == -1){
                    cout<<"Could not leave src point. Walled in?"<<endl;
                    return -1;
                }
            }//result iteration
            cout<<"Ant "<<antId<<" found solution!"<<endl;


        }//AntId iteration

        GlobalPheromoneUpdate(ShortestPath);
        cout<<"Pheromone Update Done!"<<endl;
    } //Main iteration

    //End

    DrawSolution(4,ShortestPath);
    cout<<"Search done!"<<endl;
    return true;
}

bool AntColonyOptimization::Init(vector<string> Parameters) //0 - number of ants, 1 - initial pheromone level, 2 - phi, 3 - number of iterations
{
    //Init variables
    dstNodeNum = gridcontroller->dst.y*gridcontroller->numberOfColumns+gridcontroller->dst.x;

    numberofgridcells = gridcontroller->numberOfRows*gridcontroller->numberOfColumns;
    PHEROMONCOSTMATRIX = new double[numberofgridcells*numberofgridcells];
    VISITED = new bool[numberofgridcells];

    memset(PHEROMONCOSTMATRIX,0,(numberofgridcells*numberofgridcells)*sizeof(double));
    memset(VISITED,0,numberofgridcells*sizeof(bool));

//    numberOfAnts =  stoi(Parameters[0]);
//    tauzero =  stoi(Parameters[1]);
//    phi =  stoi(Parameters[2]);
//    numberOfIterations = stoi(Parameters[3]);


    //PARAMETERS//
    numberOfAnts = 5;
    tauzero = 0.0;
    phi = 0.5;
    numberOfIterations = 1;
    //PARAMETERS//

    for(int i = 0;i<gridcontroller->numberOfRows;++i){
        for(int j = 0;j<gridcontroller->numberOfColumns;++j){
            if(i == j)
                continue;
            else if(gridcontroller->grid[i*gridcontroller->numberOfColumns+j] == -1){
                //Means this is a blocked cell so every edge into this cell has -1 level
                SetNegativeValueAroundPoint(i,j);
            }
            PHEROMONCOSTMATRIX[i*gridcontroller->numberOfColumns+j] = tauzero;
        }
    }
    cout<<"Init Successful"<<endl;
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

    while(temp.x != gridcontroller->dst.x || temp.y != gridcontroller->dst.y){

        if(!HasFreeNeighBour(temp)){//DeadEnd
            cout<<"Dead end, has to restart. AntId = "<<antId<<endl;
            if(temp == gridcontroller->src)
                return -1; //Deadlock src is walled in
            else
                return false;//simply restart searching
        }
        //check pheromone levels around
        chosenOne = ChooseNextNode(temp); //get the next node from temp
        //Locally Update pheromone level

        temp = chosenOne;
        VISITED[temp.y*gridcontroller->numberOfColumns+temp.x] = true;
        Path.push_back(temp);
    }

    if(Path.size()<ShortestPath.size()){ //Save Shortest Path
        ShortestPath = Path;
        shortestPathAntId = antId;
    }
    LocalPheromoneUpdate(Path);
    /////////////WARNING TAKE THIS OUT //////////////
    int colornum = antId+5; //from 5 - 9 is free to use
    /////////////WARNING TAKE THIS OUT //////////////

    DrawSolution(colornum,Path);

    return true; //solution found

}

void AntColonyOptimization::DrawSolution(const int &colornum, const vector<Point> &Path)
{
    try{
    for(int i = 1;i<(int)Path.size()-1;++i)
        gridcontroller->setGridValue(Path.at(i).y,Path.at(i).x,colornum);
    }    catch(std::out_of_range e){
    cout<<"drawsolution out of range"<<endl;
}
}

bool AntColonyOptimization::HasFreeNeighBour(const Point& temp) //Check if it has anywhere to go
{
    int col,row;
    col = gridcontroller->numberOfColumns;
    row = gridcontroller->numberOfRows;

    if(
       (temp.x != 0 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 1 && !VISITED[(temp.y-1)*col+(temp.x-1)])//top left
     ||(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1 && !VISITED[(temp.y-1)*col+temp.x])//top
     ||(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1 && !VISITED[(temp.y-1)*col+(temp.x+1)])//top right
     ||(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1 && !VISITED[temp.y*col+(temp.x+1)])//right
     ||(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1 && !VISITED[(temp.y+1)*col+(temp.x+1)])//down right
     ||(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1 && !VISITED[(temp.y+1)*col+temp.x])//down
     ||(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1 && !VISITED[(temp.y+1)*col+temp.x-1])//down left
     ||(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1 && !VISITED[temp.y*col+temp.x-1])//left
       ) return true;

    return false;
}

int AntColonyOptimization::RouletteWheelSelect(const vector<double>& selectables)
{
    srand (time(NULL));
    double rnd = (rand() % 10 + 1)/(double)10;
    double offset = 0.0;

    for (int i = selectables.size()-1; i>=0 ; --i) {
        offset += selectables[i];
        if (rnd <= offset) {
            return i;
        }
    }
    //This point means all of them has the initial value so choose randomly a cell.
    int randint;

    randint = rand() % selectables.size();

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

  int theChosenOne;

  theChosenOne = RouletteWheelSelect(Selectables); //Select one

  Point result;
  result = SelectableNodes.at(theChosenOne);
  return result; //Give it back

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


void AntColonyOptimization::LocalPheromoneUpdate(const vector<Point> &path)
{

    double tempvalue;
    //double value = (1-phi)*tempvalue+phi*tauzero;


    Point from,to;

    int fromNodeNum,toNodeNum;

    int L = path.size();

    for(int i = 0;i<L-1;++i){
        from = path.at(i);
        to = path.at(i+1);
        fromNodeNum = from.y*gridcontroller->numberOfColumns+from.x;
        toNodeNum = to.y*gridcontroller->numberOfColumns+to.x;
        tempvalue = PHEROMONCOSTMATRIX[fromNodeNum*gridcontroller->numberOfColumns+toNodeNum];
        PHEROMONCOSTMATRIX[fromNodeNum*gridcontroller->numberOfColumns+toNodeNum] = (1-phi)*tempvalue+phi*tauzero;
        PHEROMONCOSTMATRIX[toNodeNum*gridcontroller->numberOfColumns+fromNodeNum] = PHEROMONCOSTMATRIX[fromNodeNum*gridcontroller->numberOfColumns+toNodeNum];
    }
}

void AntColonyOptimization::GlobalPheromoneUpdate(const vector<Point> &path)
{

    Point from,to;

    int fromNodeNum,toNodeNum;

    int L = path.size();

    for(int i = 0;i<L-1;++i){
        from = path.at(i);
        to = path.at(i+1);
        fromNodeNum = from.y*gridcontroller->numberOfColumns+from.x;
        toNodeNum = to.y*gridcontroller->numberOfColumns+to.x;
        PHEROMONCOSTMATRIX[fromNodeNum*gridcontroller->numberOfColumns+toNodeNum] = UpdateEvaporationFormula(fromNodeNum,toNodeNum);
        PHEROMONCOSTMATRIX[toNodeNum*gridcontroller->numberOfColumns+fromNodeNum] = PHEROMONCOSTMATRIX[fromNodeNum*gridcontroller->numberOfColumns+toNodeNum];
    }
}

double AntColonyOptimization::UpdateEvaporationFormula(const int& fromNodeNum,const int& toNodeNum)
{

    return (1-phi)*PHEROMONCOSTMATRIX[fromNodeNum*gridcontroller->numberOfColumns+toNodeNum]+(phi*tauzero);
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


