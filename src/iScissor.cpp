/* iScissor.cpp */
/* Main file for implementing project 1.  See TODO statments below
* (see also correlation.cpp and iScissor.h for additional TODOs) */

#include <assert.h>
#include <vector>
#include <limits>
#include <iostream>

#include "correlation.h"
#include "iScissor.h"

const double linkLengths[8] = { 1.0, SQRT2, 1.0, SQRT2, 1.0, SQRT2, 1.0, SQRT2 };
const double derivLengths[8] = { 2.0, SQRT2, 2.0, SQRT2, 2.0, SQRT2, 2.0, SQRT2 };

// two inlined routines that may help;

inline Node& NODE(Node* n, int i, int j, int width)
{
    return *(n + j * width + i);
}

inline unsigned char PIXEL(const unsigned char* p, int i, int j, int c, int width)
{
    return *(p + 3 * (j * width + i) + c);
}

/************************ TODO 1 ***************************
*InitNodeBuf
*	INPUT:
*		img:	a RGB image of size imgWidth by imgHeight;
*		nodes:	a allocated buffer of Nodes of the same size, one node corresponds to a pixel in img;
*  OUPUT:
*      initializes the column, row, and linkCost fields of each node in the node buffer.
*/

void InitNodeBuf(Node* nodes, const unsigned char* img, int imgWidth, int imgHeight)
{

    // Blur the image before computing the derivatives
    double * blurredImg = new double[imgWidth * imgHeight];

    // Filter the image to compute intensity direvatives across links
    std::vector<std::vector<double> > linkRGBDeriv = std::vector<std::vector<double> >(8);
    std::vector<std::vector<double> > linkDeriv = std::vector<std::vector<double> >(8);

    int imgSize = imgWidth * imgHeight;

    //std::cout << "Get link deriv" << std::endl;
    for(int link = 0; link < 8; link ++)
    {
        linkRGBDeriv[link] = std::vector<double>(3 * imgSize);
        linkDeriv[link] = std::vector<double>(imgSize);
        //std::cout << "img filter" << std::endl;
        image_filter(&((linkRGBDeriv[link])[0]), img, NULL, imgWidth, imgHeight, kernels[link], 3, 3, 2.0*derivLengths[link], 0);
        //std::cout << "link derv" << std::endl;
        getLinkDerivFromRGBDeriv(&((linkDeriv[link])[0]), &((linkRGBDeriv[link])[0]), imgWidth, imgHeight);
    }

    // Loop over intensity derivates to find max derivativ
    //std::cout << "find max d" << std::endl;
    double maxD = 0;
    for(int link = 0; link < 8; link++)
    {
        for(int pixel = 0; pixel < imgSize; pixel++)
        {
            maxD = (maxD < linkDeriv[link][pixel])?linkDeriv[link][pixel]:maxD;
        }
    }

    // Calculate the cost of each link
    //std::cout << "Calculate cost of each link" << std::endl;
    std::vector<std::vector<double> > linkCost = std::vector<std::vector<double> >(imgSize);
    for(int pixel = 0; pixel < imgSize; pixel++)
    {
        linkCost[pixel] = std::vector<double>(8);

        for(int link = 0; link < 8; link++)
        {
            linkCost[pixel][link] = (maxD - linkDeriv[link][pixel]) * linkLengths[link];
        }
    }

    // Finally initialize all the nodes!
    //std::cout << "Init all nodes" << std::endl;
    int index = 0;
    for(int row = 0; row < imgHeight; row++)
    {
        for(int col = 0; col < imgWidth; col++)
        {
            index = row * imgWidth + col;
            nodes[index].column = col;
            nodes[index].row = row;
            
            for(int link = 0; link < 8; link++)
            {
                nodes[index].linkCost[link] = linkCost[index][link];
            }
        }
    }
    //std::cout << "Exiting initNodeBuf" << std::endl;
}
/************************ END OF TODO 1 ***************************/

void getLinkDerivFromRGBDeriv(double * deriv, double * rgbDeriv, int width, int height)
{
    for(int row = 0; row < height; row++)
    {
        for(int col = 0; col < width; col++)
        {
            deriv[row * width + col] = sqrt((rgbDeriv[3*(row*width + col)]*rgbDeriv[3*(row*width + col)] + 
                rgbDeriv[3*(row*width + col) + 1]*rgbDeriv[3*(row*width + col) + 1] + 
                rgbDeriv[3*(row*width + col) + 2]*rgbDeriv[3*(row*width + col) + 2]) / 3);
        }
    }
}

static int offsetToLinkIndex(int dx, int dy)
{
    int indices[9] = { 3, 2, 1, 4, -1, 0, 5, 6, 7 };
    int tmp_idx = (dy + 1) * 3 + (dx + 1);
    assert(tmp_idx >= 0 && tmp_idx < 9 && tmp_idx != 4);
    return indices[tmp_idx];
}

/************************ TODO 4 ***************************
*LiveWireDP:
*	INPUT:
*		seedX, seedY:	seed position in nodes
*		nodes:			node buffer of size width by height;
*      width, height:  dimensions of the node buffer;
*		selection:		if selection != NULL, search path only in the subset of nodes[j*width+i] if selection[j*width+i] = 1;
*						otherwise, search in the whole set of nodes.
*		numExpanded:		compute the only the first numExpanded number of nodes; (for debugging)
*	OUTPUT:
*		computes the minimum path tree from the seed node, by assigning
*		the prevNode field of each node to its predecessor along the minimum
*		cost path from the seed to that node.
*/

void LiveWireDP(int seedX, int seedY, Node* nodes, int width, int height, const unsigned char* selection, int numExpanded)
{
    // Initialize expansion counter
    int expansionCount = 0;

    // Initialize the priority queue pq to be empty
    CTypedPtrHeap<Node> nodePriorityQueue;

    // Initialize each node to the INITIAL state
    int nodeBufferSize = width * height;
    for(int i = 0; i < nodeBufferSize; i++)
    {
        nodes[i].state = INITIAL;
    }

    // Set the total cost of seed to zero
    Node* seed = &nodes[seedY * width + seedX];
    seed->totalCost = 0.0;
    seed->prevNode = NULL;

    // Insert seed into pq
    nodePriorityQueue.Insert(seed);

    // While priority queue is not empty
    while(!nodePriorityQueue.IsEmpty())
    {
        // Extract the node q with the minimum total cost in the priority queue
        Node * currentNode = nodePriorityQueue.ExtractMin();

        // Break once we reach out debug expansion limit
        if(expansionCount == numExpanded)
            break;

        // Mark current node as expanded
        currentNode->state = EXPANDED;
        expansionCount++;

        // neighbor offsets
        int xOffset, yOffset, neighborX, neighborY;
        // Current neighbor
        Node * currentNeighbor;

        // For each neighbor node
        for(unsigned int i = 0; i < 8; i++)
        {
            // Get neighbor offsets
            currentNode->nbrNodeOffset(xOffset, yOffset, i);

            // Skip border cases
            neighborX = currentNode->column + xOffset;
            neighborY = currentNode->row + yOffset;

            if(neighborX < 0 || neighborY < 0 || neighborX >= width || neighborY >= height)
                continue;

            // Set the current neighbor
            currentNeighbor = &nodes[neighborY * width + neighborX];

            // Skip if we shouldn't search this node
            if(selection && selection[(currentNeighbor->row) * width + currentNeighbor->column])
                continue;

            // Check if previously expanded
            if(currentNeighbor->state != EXPANDED)
            {
                if(currentNeighbor->state == INITIAL)
                {
                    // Set this neighbors total cost
                    currentNeighbor->totalCost = currentNode->totalCost + currentNode->linkCost[i];
                    // Set neighbor as active
                    currentNeighbor->state = ACTIVE;
                    // Set current node as this neighbors predecessor
                    currentNeighbor->prevNode = currentNode;
                    // Insert this neighbor into the priority queue
                    nodePriorityQueue.Insert(currentNeighbor);
                }
                else if(currentNeighbor->state == ACTIVE)
                {
                    // Update total cost of neighbor
                    if(currentNode->totalCost + currentNode->linkCost[i] < currentNeighbor->totalCost)
                    {
                        currentNeighbor->totalCost = currentNode->totalCost + currentNode->linkCost[i];
                        // Set current node as this neighbors predecessor
                        //currentNeighbor->prevNode = currentNode;
                        nodePriorityQueue.Update(currentNeighbor);
                    }
                }
            }
        }

    }

}
/************************ END OF TODO 4 ***************************/

/************************ TODO 5 ***************************
*MinimumPath:
*	INPUT:
*		nodes:				a node buffer of size width by height;
*		width, height:		dimensions of the node buffer;
*		freePtX, freePtY:	an input node position;
*	OUTPUT:
*		insert a list of nodes along the minimum cost path from the seed node to the input node.
*		Notice that the seed node in the buffer has a NULL predecessor.
*		And you want to insert a *pointer* to the Node into path, e.g.,
*		insert nodes+j*width+i (or &(nodes[j*width+i])) if you want to insert node at (i,j), instead of nodes[nodes+j*width+i]!!!
*		after the procedure, the seed should be the head of path and the input code should be the tail
*/

void MinimumPath(CTypedPtrDblList <Node>* path, int freePtX, int freePtY, Node* nodes, int width, int height)
{
    Node* initial = &nodes[freePtY*width+freePtX];
    while(initial != NULL){
        path->AddHead( initial ) ;
        initial = initial->prevNode;
    }
}
/************************ END OF TODO 5 ***************************/

/************************ An Extra Credit Item ***************************
*SeeSnap:
*	INPUT:
*		img:				a RGB image buffer of size width by height;
*		width, height:		dimensions of the image buffer;
*		x,y:				an input seed position;
*	OUTPUT:
*		update the value of x,y to the closest edge based on local image information.
*/

void SeedSnap(int& x, int& y, unsigned char* img, int width, int height)
{
    printf("SeedSnap in iScissor.cpp: to be implemented for extra credit!\n");
}

//generate a cost graph from original image and node buffer with all the link costs;
void MakeCostGraph(unsigned char* costGraph, const Node* nodes, const unsigned char* img, int imgWidth, int imgHeight)
{
    int graphWidth = imgWidth * 3;
    int graphHeight = imgHeight * 3;
    int dgX = 3;
    int dgY = 3 * graphWidth;

    int i, j;
    for (j = 0; j < imgHeight; j++) {
        for (i = 0; i < imgWidth; i++) {
            int nodeIndex = j * imgWidth + i;
            int imgIndex = 3 * nodeIndex;
            int costIndex = 3 * ((3 * j + 1) * graphWidth + (3 * i + 1));

            const Node* node = nodes + nodeIndex;
            const unsigned char* pxl = img + imgIndex;
            unsigned char* cst = costGraph + costIndex;

            cst[0] = pxl[0];
            cst[1] = pxl[1];
            cst[2] = pxl[2];

            //r,g,b channels are grad info in seperate channels;
            DigitizeCost(cst	   + dgX, node->linkCost[0]);
            DigitizeCost(cst - dgY + dgX, node->linkCost[1]);
            DigitizeCost(cst - dgY      , node->linkCost[2]);
            DigitizeCost(cst - dgY - dgX, node->linkCost[3]);
            DigitizeCost(cst	   - dgX, node->linkCost[4]);
            DigitizeCost(cst + dgY - dgX, node->linkCost[5]);
            DigitizeCost(cst + dgY	   ,  node->linkCost[6]);
            DigitizeCost(cst + dgY + dgX, node->linkCost[7]);
        }
    }
}