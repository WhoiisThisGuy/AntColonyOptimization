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
   // cout<<"Deleted ACO obj"<<endl;
}

void AntColonyOptimization::IterationBestPathUpdate()
{
    nodenumpair np;
    int L = ShortestPath.size();
    double deltatauijk = 1/(double)L;
    double pheromoneValue;

    deltatauijk = pow(deltatauijk,theta);

    for(int i = 0;i< L-1;++i){

        pheromoneValue = 0;
        np.from = ShortestPath.at(i).y*gridcontroller->numberOfColumns+ShortestPath.at(i).x;
        np.to = ShortestPath.at(i+1).y*gridcontroller->numberOfColumns+ShortestPath.at(i+1).x;

        pheromoneValue = PHEROMONCOSTMATRIX[np.from*numberofgridcells+np.to];

        PHEROMONCOSTMATRIX[np.from*numberofgridcells+np.to] = OfflineUpdateFormula(pheromoneValue,deltatauijk);
    }
}

double AntColonyOptimization::OfflineUpdateFormula(double tauij, double deltatauij)
{
    return (1-phi)*tauij+(phi*deltatauij);
}

int AntColonyOptimization::StartSearch(bool *abortFlag)
{
    for(int i = 0; i<numberOfIterations;++i){
        if((*abortFlag) == true){//catch abortFlag
            if(VISITED)
                delete [] VISITED;
            if(PHEROMONCOSTMATRIX)
                delete [] PHEROMONCOSTMATRIX;
            return -1;
        }

        //Finding country roads in West Virginia from here
        //Search starts from here
        colorNum = 5;
        for(int antId = 0;antId<numberOfAnts;++antId){
            if(*abortFlag == true){ //catch abortFlag
                if(VISITED)
                    delete [] VISITED;
                if(PHEROMONCOSTMATRIX)
                    delete [] PHEROMONCOSTMATRIX;
                return -1;
            }

            int result = 0;
            while(!result && (*abortFlag) != true){
                memset(VISITED,0,numberofgridcells*sizeof(bool));
                result = FindSolutionForAnt(antId);
                if(result == -1){

                    return -1;
                }
                if(numberOfAnts != 5 && antId==numberOfAnts-5)
                    gridcontroller->clearPathColors();
            }//result iteration end 
        }//AntId iteration end

        IterationBestPathUpdate();
        //GlobalPheromoneUpdate();


//        if(i>=numberOfIterations-5){ //Used to draw the last 5 iteration best.
//             DrawSolution(colorNum,IterationBestPath);
//             ++colorNum;
//        }

    } //Main iteration end

    //End
    DrawSolution(4,ShortestPath);

    if(VISITED)
        delete [] VISITED;
    if(PHEROMONCOSTMATRIX)
        delete [] PHEROMONCOSTMATRIX;

    return 0;
}

vector<int> AntColonyOptimization::getPath()
{
    vector<int> asd;
    return asd;
}


bool AntColonyOptimization::Init(const vector<string> &Parameters)
{
    if(Parameters.size()<8)
        return false;
    //PARAMETERS: numberOfAnts, numberOfIterations, tauzero, phi, alpha, beta, theta, kappa//
    numberOfAnts = std::stoi(Parameters.at(0));
    numberOfIterations = std::stoi(Parameters.at(1));

    tauzero = std::stod(Parameters.at(2));
    phi = std::stod(Parameters.at(3));
    alpha = std::stod(Parameters.at(4));
    beta = std::stod(Parameters.at(5));
    theta = std::stod(Parameters.at(6));
    kappa = std::stod(Parameters.at(7));
    //PARAMETERS//

     ShortestPath.clear();
    //Init variables
    bestAntId = 0;
    dstNodeNum = gridcontroller->dst.y*gridcontroller->numberOfColumns+gridcontroller->dst.x;

    numberofgridcells = gridcontroller->numberOfRows*gridcontroller->numberOfColumns;

    try{
    PHEROMONCOSTMATRIX = new double[numberofgridcells*numberofgridcells];
    }catch(std::error_code e){
      //out of memory

    }
    try{
    VISITED = new bool[numberofgridcells];
    }catch(std::error_code e){

    }

    for(int i =0;i<numberofgridcells;++i){
        for(int j =0;j<numberofgridcells;++j){
             PHEROMONCOSTMATRIX[i*numberofgridcells+j] = tauzero;
        }
    }

    memset(VISITED,0,numberofgridcells*sizeof(bool));
return true;
}

int AntColonyOptimization::FindSolutionForAnt(const int& antId)
{
    Point temp;
    Point chosenOne;
    //it all began on the src
    vector<Point> Path;
    Path.push_back(gridcontroller->src);
    temp = gridcontroller->src;
    VISITED[gridcontroller->src.y*gridcontroller->numberOfColumns+gridcontroller->src.x] = true;


    while(temp.x != gridcontroller->dst.x || temp.y != gridcontroller->dst.y){

        if(!HasFreeNeighBour(temp)){//DeadEnd

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
    //SumDeltaTauijk(Path); //Saving the sum of lengths for global update
    if(antId == 0){  //Used only for visualizing purposes
        DrawSolution(10,Path);
    }
    else if(antId>=numberOfAnts-5){

        //IterationBestPath = Path;
        DrawSolution(colorNum,Path);
        ++colorNum;
    }
    if(ShortestPath.size() == 0 || (int)Path.size()<ShortestLenght){
        bestAntId = antId;
        ShortestPath = Path;
        shortestPathAntId = antId;
        ShortestLenght = Path.size();

    }

    LocalPheromoneUpdate(Path);



    return true; //solution found

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
          && !VISITED[(temp.y-1)*col+(temp.x-1)])//top left
  {
      neighbournodenum = (temp.y-1)*col+(temp.x-1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y-1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y-1));

  }
  if(temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+temp.x] != 1
          && !VISITED[(temp.y-1)*col+temp.x])//top
  {
      neighbournodenum = (temp.y-1)*col+temp.x;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x,temp.y-1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x,temp.y-1));

  }
  if(temp.x != col-1 && temp.y != 0 && gridcontroller->grid[(temp.y-1)*col+(temp.x+1)] != 1
          && !VISITED[(temp.y-1)*col+(temp.x+1)])//top right
  {
      neighbournodenum = (temp.y-1)*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y-1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y-1));

  }
  if(temp.x != col-1 && gridcontroller->grid[temp.y*col+(temp.x+1)] != 1
          && !VISITED[temp.y*col+(temp.x+1)])//right
  {
      neighbournodenum = temp.y*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y));

  }
  if(temp.x != col-1 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+(temp.x+1)] != 1
          && !VISITED[(temp.y+1)*col+(temp.x+1)])//down right
  {
      neighbournodenum = (temp.y+1)*col+(temp.x+1);
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x+1,temp.y+1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x+1,temp.y+1));
  }
  if(temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x] != 1
          && !VISITED[(temp.y+1)*col+temp.x])//down
  {
      neighbournodenum = (temp.y+1)*col+temp.x;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x,temp.y+1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x,temp.y+1));
  }
  if(temp.x != 0 && temp.y != row-1 && gridcontroller->grid[(temp.y+1)*col+temp.x-1] != 1
          && !VISITED[(temp.y+1)*col+temp.x-1])//down left
  {
      neighbournodenum = (temp.y+1)*col+temp.x-1;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y+1);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y+1));
  }
  if(temp.x != 0 && gridcontroller->grid[temp.y*col+temp.x-1] != 1
          && !VISITED[temp.y*col+temp.x-1])//left
  {
      neighbournodenum = temp.y*col+temp.x-1;
      if(dstNodeNum == neighbournodenum)
          return Point(temp.x-1,temp.y);
      ijpheromonevalues.push_back(PHEROMONCOSTMATRIX[tempnodenum*numberofgridcells+neighbournodenum]);
      SelectableNodes.push_back(Point(temp.x-1,temp.y));
  }

  vector<double> ijprobabilites;

  CalculateProbability(ijpheromonevalues,SelectableNodes,ijprobabilites,temp); //Calculate the P(i,j) probabilites

  ComulativeSum(ijprobabilites);

  int theChosenOne;


  theChosenOne = RouletteWheelSelect(ijprobabilites); //Select one

  Point result;
  result = SelectableNodes.at(theChosenOne);
  return result; //Give it back

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

    for (int i = ijprobabilites.size()-1; i>=0 ; --i) { //parsing backwards because the smallest value is the last one
        offset = ijprobabilites[i];
        if (rnd <= offset) {
            return i;
        }
    }
    //This point means all of them has the initial value so choose randomly a cell.
    int randint;

    randint = rand() % ijprobabilites.size();

    return randint;
}


double AntColonyOptimization::Eta(const Point& i,const Point& j)
{

    double id,ij,jd,eta;

    id = eucledianDistance(i.x,i.y,gridcontroller->dst.x,gridcontroller->dst.y);
    ij = eucledianDistance(i.x,i.y,j.x,j.y);
    jd = eucledianDistance(j.x,j.y,gridcontroller->dst.x,gridcontroller->dst.y);

    eta = id/(ij+jd);

    return eta;
}



void AntColonyOptimization::CalculateProbability(const vector<double> &ijpheromonevalues,const vector<Point>& SelectableNodes,vector<double> &ijprobabilites,const Point& from)
{
    double value = 0.0;
    double eta = 0.0;

    for (int i = 0; i < (int)ijpheromonevalues.size(); i++) { //Cumulative sum
        value = 0.0;
        for (int j = 0; j < (int)ijpheromonevalues.size(); j++) { //calculate denominators

            eta = 0.0;

            eta = Eta(from,SelectableNodes.at(j));

            value += (pow(ijpheromonevalues.at(j),alpha)*pow(eta,beta));
        }
        eta = Eta(from,SelectableNodes.at(i));
        ijprobabilites.push_back(((pow(ijpheromonevalues.at(i),alpha))*pow(eta,beta))/value); //calculate numerators
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
    double newValue;

    Point from,to;

    int fromNodeNum,toNodeNum;

    int L = path.size();

    for(int i = 0;i<L-1;++i){

        from = path.at(i);
        to = path.at(i+1);
        fromNodeNum = from.y*gridcontroller->numberOfColumns+from.x;
        toNodeNum = to.y*gridcontroller->numberOfColumns+to.x;
        tempvalue = PHEROMONCOSTMATRIX[fromNodeNum*numberofgridcells+toNodeNum];
        newValue = (1-phi)*tempvalue+(phi*kappa);
        PHEROMONCOSTMATRIX[fromNodeNum*numberofgridcells+toNodeNum] = newValue;

    }
}

//void AntColonyOptimization::GlobalPheromoneUpdate()
//{
//    double tauij,sumdeltatauijk;
//    std::map<nodenumpair,double>::const_iterator it;

//    for (int i = 0; i < numberofgridcells; i++) { //Cumulative sum
//        for (int j = 0; j < numberofgridcells; j++) {

//            tauij = 0.0;
//            sumdeltatauijk = 0.0;

//            if(i == j || PHEROMONCOSTMATRIX[i*numberofgridcells+j] == -1)
//                continue;

//            tauij = PHEROMONCOSTMATRIX[i*numberofgridcells+j];
//            it = NodeNumPairMap.find(nodenumpair(i,j));

//            if(it != NodeNumPairMap.end())
//                sumdeltatauijk = it->second;
//            PHEROMONCOSTMATRIX[i*numberofgridcells+j] = GlobalPheromoneFormula(tauij,sumdeltatauijk);

//        }
//    }
//}
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

//void AntColonyOptimization::SumDeltaTauijk(const vector<Point> &Path)
//{
//    std::map<nodenumpair,double>::const_iterator it;

//    nodenumpair np;
//    double L = 0;

//    if(Path.size()<=0)
//        return;

//    for(int i = 0;i<(int)Path.size()-1;++i){
//        L=0;
//        np.from = Path.at(i).y*gridcontroller->numberOfColumns+Path.at(i).x;
//        np.to = Path.at(i+1).y*gridcontroller->numberOfColumns+Path.at(i+1).x;

//        it = NodeNumPairMap.find(np);
//        if(it!=NodeNumPairMap.end())
//            L = it->second;
//        L += 1/(double)Path.size();

//        NodeNumPairMap.insert(std::pair<nodenumpair,double>(np,L));

//    }
//}

void AntColonyOptimization::DrawSolution(const int &colornum, const vector<Point> &Path)
{

    for(int i = 1;i<(int)Path.size()-1;++i)
        gridcontroller->setGridValue(Path.at(i).y,Path.at(i).x,colornum);

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


