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

    cout<<" ACO deletted"<<endl;
}

int AntColonyOptimization::StartSearch()
{

    for(int i = 0; i<numberOfIterations;++i){
        cout<<"Press a key to start"<<i<<". iteration"<<endl;
        std::cin.ignore();
        NodeNumPairMap.clear();

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

        GlobalPheromoneUpdate();
        cout<<"Pheromone Update Done!"<<endl;
    } //Main iteration

    //End

    DrawSolution(4,ShortestPath);
    cout<<"Search done!"<<endl;
    return true;
}

bool AntColonyOptimization::Init(vector<string> Parameters) //0 - number of ants, 1 - initial pheromone level, 2 - phi, 3 - number of iterations
{
    //PARAMETERS//
    numberOfAnts = 1;
    tauzero = 0.0;
    phi = 0.5;
    numberOfIterations = 10;
    alpha = 2.0;
    //PARAMETERS//

    //Init variables
    dstNodeNum = gridcontroller->dst.y*gridcontroller->numberOfColumns+gridcontroller->dst.x;

    numberofgridcells = gridcontroller->numberOfRows*gridcontroller->numberOfColumns;
    PHEROMONCOSTMATRIX = new double[numberofgridcells*numberofgridcells];
    VISITED = new bool[numberofgridcells];

    memset(PHEROMONCOSTMATRIX,tauzero,(numberofgridcells*numberofgridcells)*sizeof(double));
    memset(VISITED,0,numberofgridcells*sizeof(bool));

//    numberOfAnts =  stoi(Parameters[0]);
//    tauzero =  stoi(Parameters[1]);
//    phi =  stoi(Parameters[2]);
//    numberOfIterations = stoi(Parameters[3]);

    for(int i = 0;i<gridcontroller->numberOfRows;++i){
        for(int j = 0;j<gridcontroller->numberOfColumns;++j){
            if(i == j)
                continue;
            else if(gridcontroller->grid[i*gridcontroller->numberOfColumns+j] == -1){
                //Means this is a blocked cell so every edge into this cell has -1 level
                SetNegativeValueAroundPoint(i,j);
            }
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

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
    }
    if(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1
        && gridcontroller->grid[(temp.y-1)*col+temp.x] != 3)//top
    {
        neighbournodenum = (temp.y-1)*col+temp.x;

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
    }
    if(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1
        && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 3)//top right
    {
        neighbournodenum = (temp.y-1)*col+(temp.x+1);

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
    }
    if(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1
        && gridcontroller->grid[temp.y*col+(temp.x+1)] != 3)//right
    {
        neighbournodenum = temp.y*col+(temp.x+1);

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
    }
    if(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1
        && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 3)//down right
    {
        neighbournodenum = (temp.y+1)*col+(temp.x+1);

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
    }
    if(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1
        && gridcontroller->grid[(temp.y+1)*col+temp.x] != 3)//down
    {
        neighbournodenum = (temp.y+1)*col+temp.x;

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
    }
    if(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1
        && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 3)//down left
    {
        neighbournodenum = (temp.y+1)*col+temp.x-1;

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
    }
    if(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1
        && gridcontroller->grid[temp.y*col+temp.x-1] != 3)//left
    {
        neighbournodenum = temp.y*col+temp.x-1;

        PHEROMONCOSTMATRIX[neighbournodenum*numberofgridcells+tempnodenum] = -1;
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

        temp = chosenOne;
        VISITED[temp.y*gridcontroller->numberOfColumns+temp.x] = true;

        Path.push_back(temp);

    }
    ///////////////////////////SOLUTION FOUND FROM HERE /////////////////////////////////
    SumDeltaTauijk(Path); //Saving this for Global update

    if(antId == 0){
        ShortestLenght = Path.size();
        ShortestPath = Path;
    }
    else if(Path.size()<ShortestLenght){ //Save Shortest Path
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
       (temp.x != 0 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 1 && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 2 && !VISITED[(temp.y-1)*col+(temp.x-1)])//top left
     ||(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 2 && !VISITED[(temp.y-1)*col+temp.x])//top
     ||(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 2 && !VISITED[(temp.y-1)*col+(temp.x+1)])//top right
     ||(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1  && gridcontroller->grid[temp.y*col+(temp.x+1)] != 2 && !VISITED[temp.y*col+(temp.x+1)])//right
     ||(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 2 && !VISITED[(temp.y+1)*col+(temp.x+1)])//down right
     ||(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 2 && !VISITED[(temp.y+1)*col+temp.x])//down
     ||(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 2 && !VISITED[(temp.y+1)*col+temp.x-1])//down left
     ||(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1 && gridcontroller->grid[temp.y*col+temp.x-1] != 2 && !VISITED[temp.y*col+temp.x-1])//left
       ) return true;

    return false;
}

int AntColonyOptimization::RouletteWheelSelect(const vector<double>& ijprobabilites)
{
    srand (time(NULL));
    double rnd = (rand() % 10 + 1)/(double)10;
    double offset = 0.0;

    for (int i = ijprobabilites.size()-1; i>=0 ; --i) {
        offset += ijprobabilites[i];
        if (rnd <= offset) {
            return i;
        }
    }
    //This point means all of them has the initial value so choose randomly a cell.
    int randint;

    randint = rand() % ijprobabilites.size();

    return randint;
}

Point AntColonyOptimization::ChooseNextNode(const Point& temp)
{
  vector<Point> SelectableNodes;
  vector<double> ijpheromonevalues;

  int col,row;
  int tempnodenum = 0;
  int neighbournodenum = 0;

  col = gridcontroller->numberOfColumns;
  row = gridcontroller->numberOfRows;
  tempnodenum = temp.y*col+temp.x;

  if(temp.x != 0 && temp.y != 0
      && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 1
          && gridcontroller->grid[(temp.y-1)*col+(temp.x-1)] != 2)//top left
  {
      neighbournodenum = (temp.y-1)*col+(temp.x-1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y-1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y-1));
  }
  if(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1
          && gridcontroller->grid[(temp.y-1)*col+temp.x] != 2)//top
  {
      neighbournodenum = (temp.y-1)*col+temp.x;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x,temp.y-1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x,temp.y-1));
  }
  if(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1
          && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 2)//top right
  {
      neighbournodenum = (temp.y-1)*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y-1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y-1));
  }
  if(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1
          && gridcontroller->grid[temp.y*col+(temp.x+1)] != 2)//right
  {
      neighbournodenum = temp.y*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y));
  }
  if(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1
          && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 2)//down right
  {
      neighbournodenum = (temp.y+1)*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y+1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y+1));
  }
  if(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1
          && gridcontroller->grid[(temp.y+1)*col+temp.x] != 2)//down
  {
      neighbournodenum = (temp.y+1)*col+temp.x;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x,temp.y+1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x,temp.y+1));
  }
  if(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1
          && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 2)//down left
  {
      neighbournodenum = (temp.y+1)*col+temp.x-1;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y+1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y+1));
  }
  if(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1
          && gridcontroller->grid[temp.y*col+temp.x-1] != 2)//left
  {
      neighbournodenum = temp.y*col+temp.x-1;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y));
  }

  vector<double> ijprobabilites;

  try{
  CalculateProbability(ijpheromonevalues,tempnodenum,ijprobabilites); //Calculate the P(i,j) probabilites
  }catch(std::out_of_range e){
      cout<<"Calcproberror range"<<endl;
  }
    try{
  ComulativeSum(ijprobabilites);
}catch(std::out_of_range e){
    cout<<"Comulateivaisd sum range error"<<endl;
}
  int theChosenOne;


  theChosenOne = RouletteWheelSelect(ijprobabilites); //Select one

  Point result;
  result = SelectableNodes.at(theChosenOne);
  return result; //Give it back

}

void AntColonyOptimization::CalculateProbability(const vector<double> &ijpheromonevalues, int tempnodenum,vector<double> &ijprobabilites)
{
    double value = 0.0;

    for (int i = 0; i < (int)ijpheromonevalues.size(); i++) { //Cumulative sum
        value = 0.0;
        for (int j = 0; j < (int)ijpheromonevalues.size(); j++) {
            if(i == j)
                continue;
            value += pow(ijpheromonevalues.at(j),alpha);
        }
        ijprobabilites.push_back((pow(ijpheromonevalues.at(i),alpha))/value);
    }
}

void AntColonyOptimization::ComulativeSum(vector<double> &ijprobabilites)
{
    for (int i = 0; i < (int)ijprobabilites.size(); i++) { //Cumulative sum
        for (int j = i; j < (int)ijprobabilites.size(); j++) {
            if(i == j)
                continue;
             ijprobabilites.at(i) += ijprobabilites.at(j);
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
        tempvalue = PHEROMONCOSTMATRIX[fromNodeNum*numberofgridcells+toNodeNum];
        PHEROMONCOSTMATRIX[fromNodeNum*numberofgridcells+toNodeNum] = (1-phi)*tempvalue+phi*tauzero;
    }
}

void AntColonyOptimization::GlobalPheromoneUpdate()
{
    double tauij,sumdeltatauijk;
    std::map<nodenumpair,double>::const_iterator it;

    for (int i = 0; i < numberofgridcells; i++) { //Cumulative sum
        for (int j = 0; j < numberofgridcells; j++) {

            tauij = 0.0;
            sumdeltatauijk = 0.0;

            if(i == j || PHEROMONCOSTMATRIX[i*numberofgridcells+j] == -1)
                continue;

            tauij = PHEROMONCOSTMATRIX[i*numberofgridcells+j];
            it = NodeNumPairMap.find(nodenumpair(i,j));

            if(it != NodeNumPairMap.end())
                sumdeltatauijk = it->second;
            PHEROMONCOSTMATRIX[i*numberofgridcells+j] = GlobalPheromoneFormula(tauij,sumdeltatauijk);

        }
    }
}
////////////////////BEST PATH UPDATE
//    Point from,to;

//    int fromNodeNum,toNodeNum;

//    int L = path.size();

//    for(int i = 0;i<L-1;++i){
//        from = path.at(i);
//        to = path.at(i+1);
//        fromNodeNum = from.y*gridcontroller->numberOfColumns+from.x;
//        toNodeNum = to.y*gridcontroller->numberOfColumns+to.x;
//        PHEROMONCOSTMATRIX[fromNodeNum*gridcontroller->numberOfColumns+toNodeNum] = UpdateEvaporationFormula(fromNodeNum,toNodeNum);
//    }

double AntColonyOptimization::GlobalPheromoneFormula(double tauij, double sumdeltatauijk)
{
    return (1-phi)*tauij+sumdeltatauijk;
}

void AntColonyOptimization::SumDeltaTauijk(const vector<Point> &Path)
{
    std::map<nodenumpair,double>::const_iterator it;
    //vector<nodenumpair> VisitedEdges;
    nodenumpair np;
    double L = 0;

    if(Path.size()<=0)
        return;

    for(int i = 0;i<(int)Path.size()-1;++i){
        L=0;
        np.from = Path.at(i).y*gridcontroller->numberOfColumns+Path.at(i).x;
        np.to = Path.at(i+1).y*gridcontroller->numberOfColumns+Path.at(i+1).x;

        it = NodeNumPairMap.find(np);
        if(it!=NodeNumPairMap.end())
            L = it->second;
        L += 1/Path.size();

        NodeNumPairMap.insert(std::pair<nodenumpair,double>(np,L));

    }
}

double AntColonyOptimization::UpdateEvaporationFormula(const int& fromNodeNum,const int& toNodeNum) //FOR BEST PATH only
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


