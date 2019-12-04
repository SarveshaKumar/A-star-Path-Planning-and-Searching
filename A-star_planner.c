#include <math.h>
#include "mex.h"
#include<string.h> 
#include<time.h> 
/* Input Arguments */
#define MAP_IN      prhs[0]
#define ROBOT_IN    prhs[1]
#define GOAL_IN     prhs[2]


/* Output Arguments */


#define ACTION_OUT  plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

clock_t start, end; 
int temp = 0;
int value;
int pathArray[999999] = {-1};   //Stores the index values for backpointing  
int count = 0;          //Total pathArray values in the array 
int tempVal, sizeArray; 



typedef struct {
    double fval;
    double gval;
    double hval;
    int index;
    bool close;  
    int bp; 
} node_t;       
 
typedef struct {
    node_t *nodes;
    int len;
    int size;
} heap_t;       



void push (heap_t *h, node_t* node) {           //index and the priority alone 
    double fval = node->fval;
    int index = node->index;

    if (h->len + 1 >= h->size) {
        h->size = h->size ? h->size * 2 : 4;
        h->nodes = (node_t *)realloc(h->nodes, h->size * sizeof (node_t));
    }
    int i = h->len + 1;
    int j = i / 2;                               //choosing parent node, and equating it to j
    while (i > 1 && h->nodes[j].fval > fval) {        //until priority values of the parent nodes chosen are greater than the new one
        h->nodes[i] = h->nodes[j];                    //keep moving up the tree
        i = j;                                        
        j = j / 2;  
    }
    h->nodes[i].fval = fval;    //h is a pointer and we access the nodes[i] member
    h->nodes[i].index = index;  //assigning values to the new node position 
    h->len++;
}





int pop (heap_t *h) {
    int i, j, k;
    int index = h->nodes[1].index;
 
    h->nodes[1] = h->nodes[h->len];
    double fval = h->nodes[1].fval;
 
    h->len--;
 
    i = 1;
    while (1) {
        k = i;
        j = 2 * i;
        if (j <= h->len && h->nodes[j].fval < fval) {
            k = j;
        }
        if (j + 1 <= h->len && h->nodes[j + 1].fval < h->nodes[k].fval) {
            k = j + 1;
        }
        if (k == i) {
            break;
        }
        h->nodes[i] = h->nodes[k];
        i = k;
    }
    h->nodes[i] = h->nodes[h->len + 1];
    return index; 
}




int checkValidity(int newx, int newy,int x_size,int y_size)
{
    if(newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
    {
        // printf("%d,",1 ); 
        value = 1; 
    }
    else
    {
        value = 0;
    }
    return value; 
}




double findheuristic(int robotposeX,int robotposeY,int goalposeX,int goalposeY)
{                          
    double hval = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));   
    return hval; 
}



static void planner(
           double*  map,
           int x_size,
           int y_size,
           int robotposeX,
            int robotposeY,
            int goalposeX,
            int goalposeY,
            char *p_actionX,
            char *p_actionY  
           )
{
    start = clock(); 
   //8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};    
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    int newx, newy, newInd, check;
    double costFunc; 
    printf("call=%d\n", temp);
    temp = temp+1;

    heap_t *h = (heap_t *)calloc(1, sizeof (heap_t));       



    int goalIndex = (goalposeY-1)*x_size + goalposeX -1; 
     
    node_t *totalnodes[x_size*y_size]; 
    node_t *(*p)[]= &totalnodes; 
            
    for(int m=0; m<y_size; m++)
    {
        for(int n=0; n<x_size; n++)
        {
            totalnodes[m*x_size + n] = malloc(sizeof(node_t)); 
        }
    }

    for(int i=0; i<y_size; i++)
    {
        for(int j=0; j<x_size; j++)
        { 
            totalnodes[i*x_size + j]->gval = 999999;
            totalnodes[i*x_size + j]->hval = findheuristic(i,j, goalposeX, goalposeY);
            totalnodes[i*x_size + j]->fval = totalnodes[i*x_size + j]->gval + totalnodes[i*x_size +j]->hval; 
            totalnodes[i*x_size + j]->close = 0; 
            totalnodes[i*x_size + j]->index = i*x_size + j; 
            totalnodes[i*x_size + j]->bp = -1; 
            
        }
    }
    int startInd = (robotposeY-1)*x_size + robotposeX-1;
    totalnodes[startInd]->gval = 0;
    push(h,totalnodes[startInd]);
    
   
    while(totalnodes[goalIndex]->close==0 && h->len!=0)  
    {
        int currInd = pop(h);                  
        totalnodes[currInd]->close =1; 
        int currX = (currInd)%(x_size) +1;
        int currY = (currInd)/(x_size) +1;   
        
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {

            newx = currX + dX[dir];   
            newy = currY + dY[dir];    

            newInd = (newy-1)*x_size + newx-1; 
            costFunc = sqrt(dX[dir]*dX[dir] + dY[dir]*dY[dir]);   
                                

            check = checkValidity( newx, newy, x_size, y_size);
            if((check==1) && (totalnodes[newInd]->close==0))
            {
                if((int)map[GETMAPINDEX(newx, newy,x_size, y_size)]==0)
                {
                    if((totalnodes[newInd]->gval) > totalnodes[currInd]->gval + costFunc)
                    {
                        totalnodes[newInd]->gval = totalnodes[currInd]->gval + costFunc;
                        totalnodes[newInd]->hval = findheuristic(newx, newy, goalposeX,goalposeY); 
                        totalnodes[newInd]->fval = totalnodes[newInd]->gval + totalnodes[newInd]->hval; 
                        totalnodes[newInd]->index = newInd; 
                        totalnodes[newInd]->bp = totalnodes[currInd]->index; 
                        push(h, totalnodes[newInd]);
                    }
                }
            }
        } 
    } 
    tempVal = goalIndex;  

    while(totalnodes[tempVal]->bp !=-1)          //BackTracking 
    {  
        pathArray[count] = tempVal; 
        tempVal = totalnodes[tempVal]->bp;
        count = count+1;   
    }

    pathArray[count] = tempVal; //The current robot pose Index value stored as last element in the array! 
    int ind1 = pathArray[count];
    int ind2 = pathArray[count-1]; 
    int ind1X = (ind1)%(x_size) +1;
    int ind1Y = (ind1)/(x_size) +1;   
    int ind2X = (ind2)%(x_size) +1;                        
    int ind2Y = (ind2)/(x_size) +1;
    *p_actionX = ind2X - ind1X;
    *p_actionY = ind2Y - ind1Y;  
    //printf("action: %d %d; \n", *p_actionX, *p_actionY);
    //printf("robot: %d %d; ", robotposeX, robotposeY);
    //printf("goal: %d %d;", goalposeX, goalposeY);
    memset(pathArray, -1, 999999 );
    count =0; 
    end =clock(); 
    double time_spent = (double)(end - start)/1000;
    printf("time taken in seconds is %f \n", time_spent);
}



//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector <x,y> for the robot pose
//3rd is a row vector <x,y> for the target pose
//plhs should contain output parameters (1): 
//1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[], 
          int nrhs, const mxArray*prhs[] )
     
{ 
    
    // Check for proper number of arguments //    
    if (nrhs != 3) { 
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    // get the dimensions of the map and the map matrix itself//     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    // get the dimensions of the robotpose and the robotpose itself//     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    // get the dimensions of the goalpose and the goalpose itself//     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 2.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    int goalposeX = (int)goalposeV[0];
    int goalposeY = (int)goalposeV[1];
        
    // Create a matrix for the return action // 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxINT8_CLASS, mxREAL); 
    char* action_ptr = mxGetPr(ACTION_OUT);  
            
    // Do the actual planning in a subroutine //
    planner(map, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY, &action_ptr[0], &action_ptr[1]); 
    return;
}