//  $Id: fuzzy_ai_path_tree.cpp 10225 2011-11-26 12:31:33Z Kinsu $
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2006 SuperTuxKart-Team
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#include <iostream>

#include "tracks/fuzzy_ai_path_tree.hpp"
#include "tracks/quad_graph.hpp"
#include "items/item_manager.hpp"

using namespace std;

// TODO : make comments doxygen compliant

//==============================================================================
// Constructor & destructor related functions

//------------------------------------------------------------------------------
/** Constructor
 *  Initializes a tree of paths based on a driveline fork. The constructor first
 *  calls the buildTree function to create the tree architecture, and then the
 *  setPathData function to fill in the path data.
 */
FuzzyAiPathTree::FuzzyAiPathTree(unsigned int rootNodeId = 0)
{
    m_treeRoot = setPathData(buildTree(rootNodeId));
    m_compareData = getComparableData(m_treeRoot);

// Debug output
//    cout << "COMPARE DATA is AFTER BUILDING : " << endl;
//    for(unsigned int i=0; i<m_compareData->size() ; i++)
//    {
//        cout << "vector " << i << ":" << endl;
//        for(unsigned int j=0; j<m_compareData->at(i)->size() ; j++)
//        {
//            cout << "pathData " << j << " = " << m_compareData->at(i)->at(j)->pathLength << endl;
//        }
//    }
} // Constructor

//------------------------------------------------------------------------------
/** Build tree TODO comment
 */
FuzzyAiPathTree::TreeNode* FuzzyAiPathTree::buildTree(unsigned int startNodeId)
{
    unsigned int         curNodeId = startNodeId;
    TreeNode*            rootNode = new TreeNode(startNodeId, NULL, NULL);
    vector<unsigned int> nextGraphNodes(1, curNodeId); // Init to enter loop

    while(nextGraphNodes.size() == 1 && nextGraphNodes[0] == curNodeId)
    {
        nextGraphNodes.clear();
        QuadGraph::get()->getSuccessors(curNodeId, nextGraphNodes, true);
        curNodeId ++;
    }
    rootNode->nodeId = curNodeId - 1;

    if(nextGraphNodes.size() > 1) // if fork
    {
        rootNode->children = new vector<TreeNode*>();
        for(unsigned int i=0 ; i<nextGraphNodes.size() ; i++)
            rootNode->children->push_back(buildTree(nextGraphNodes[i])); // Build sub-trees
    }
    return rootNode;
} // buildTree

//------------------------------------------------------------------------------
/** This function is used to compute and store path data in the tree nodes, once
 *  the tree has been built (by the buildTree function).
 *
 *  It first updates the tree, making sure that every path stored in it has the
 *  same starting quadGraph node and the same ending quadGraph node. This is
 *  useful in some complex cases, when the track has a 2nd fork after the 1st
 *  fork, which creates a branch that ends after the other 1st fork branch, on
 *  the main path (see below).
 *
 *              ___0.1___           alt. path 0.1 (left at 2nd fork)
 *  -> _____0__/___0.0___\_____  main path (always 0)
 *        \___1___/                 alt. path 1 (right at 1st fork)
 *
 *  To compare a path 0.x with the path 1, the end point of the path 1 must be
 *  set to the point where the path 0.1 joins back the main path. So the path 1
 *  is "rallongé" (todo translate).
 *
 *  When all the paths have the same ending point, data about each path is
 *  recursively computed and added to the nodes (length, bonus & malus count).
 *  After this, the tree corresponding to the below track will be :
 *                                   
 *           ___x__o__x___      Path1   |               Root (node=42,
 *   -> ____/___x_____o___\____Path0    |             /  |  \   data=NULL)
 *         ^\___o__/^      ^    Path2   |           /    |    \
 *         |        |      |            |      Path1   Path0   Path2
 *  Node:  42      66     84            |   node=84   node=84   node=84
 *                             o: box   |   o=1,x=2   o=1,x=1   o=2,x=0
 *                          x: banana   |   len=17    len=15    len=17
 */
FuzzyAiPathTree::TreeNode* FuzzyAiPathTree::setPathData(TreeNode* rootNode,
                                                        PathData* rootData)
{
    unsigned int rootNodeId = rootNode->nodeId;
    unsigned int treeEndNodeId = getFarthestNode(rootNode);

    if(rootData)
        rootNode->data = rootData;

    if(rootNode->children)
    {
        unsigned int          branchEndNodeId;
        TreeNode*             curChild;
        vector<unsigned int>  firstPathNodes;

        QuadGraph::get()->getSuccessors(rootNodeId, firstPathNodes, true);
#ifdef AI_DEBUG
        assert(firstPathNodes.size() == 1); // Cannot be 1 (see buildTree)
#endif
        for(unsigned int i=0; i<rootNode->children->size() ; i++)
        {
            unsigned int          curNodeId = firstPathNodes[i];
            unsigned int          lastNodeId = rootNodeId;
            PathData*             childData = new PathData();
            vector<unsigned int>  nextGraphNodes;
            
            curChild = rootNode->children->at(i);
            
            if(curChild->children == NULL)  // if the current node is a leave
                curChild->nodeId = treeEndNodeId; //The end quad of all paths must be the same
            
            branchEndNodeId = curChild->nodeId;
//            float    startD = QuadGraph::get()->getDistanceFromStart(curNodeId);
            
            while(curNodeId != branchEndNodeId)
            {
                nextGraphNodes.clear();
                QuadGraph::get()->getSuccessors(curNodeId, nextGraphNodes,true);
#ifdef AI_DEBUG
                assert(nextGraphNodes.size() == 1); // Can only be 1 (see buildTree)
#endif
                const Quad q = QuadGraph::get()->getQuadOfNode(curNodeId);
                const Quad q2 = QuadGraph::get()->getQuadOfNode(lastNodeId); 
                float dist = (q.getCenter() - q2.getCenter()).length();
                childData->bonusCount += item_manager->getQuadBonusCount(q);
                childData->malusCount += item_manager->getQuadMalusCount(q);
                childData->pathLength += dist;
                
                lastNodeId = curNodeId;
                curNodeId = nextGraphNodes[0];
            } // while branch end has not been reached yet
            
            setPathData(curChild, childData);
        } // for each child
    } // if node has children
    
    return rootNode;
} // setPathData

//------------------------------------------------------------------------------
/** Actually returns the node id of the successor of the farthest node from the 
 *  root.
 *  The last quad of the main path must not be a divergent fork for this
 *  function to work well.
 *  TODO improve comment
 */
unsigned int FuzzyAiPathTree::getFarthestNode(const TreeNode* rootNode) const
{
    vector<unsigned int> branchLastNodes = vector<unsigned int>();

    if(!rootNode->children)
    {
        vector<unsigned int> nextGraphNodes;
        unsigned int curNodeId = rootNode->nodeId;
        unsigned int mainPathLength = QuadGraph::get()->getLapLength();
        if(curNodeId != mainPathLength - 1) // Don't take in account the last node of the main path
        {
            QuadGraph::get()->getSuccessors(curNodeId, nextGraphNodes, true);
#ifdef AI_DEBUG
            assert(nextGraphNodes.size() == 1); // Can only be 1 (see buildTree)
#endif
            branchLastNodes.push_back(nextGraphNodes[0]);
        }
    }
    else
    {
        for(unsigned int i=0; i<rootNode->children->size(); i++)
        {
            TreeNode* currentChild = rootNode->children->at(i);
            branchLastNodes.push_back(getFarthestNode(currentChild));
        }
    }
    return *max_element(branchLastNodes.begin(), branchLastNodes.end());
} // getFarthestNode

//------------------------------------------------------------------------------
/** This function builds and returns a vector of PathData* vectors. Each
 *  PathData instance gives information about a specific possible path that can
 *  be taken by a vehicle when it is on the tree root (fork quadGraph node).
 * 
 *  The returned vector contains a PathData* vector for each choice one has
 *  when being at the rootNode.
 *                        For instance, if the fork lets 3 choices, it will
 *        ___1___         contain 3 vectors. The first of these 3 vectors will
 * -> ___/___0___\___->   be the vector of the possible paths if the taken
 *       \___2___/        choice is to follow the first successor (i.e. the
 *                        successor 0). In this case, the returned vector is
 *  [sub-vector 0, sub-vector 1, sub-vector 2]
 *  Each one of the sub-vectors contains one pathData*, as there is only one
 *  path to follow once the choice has been taken (go left, right or straight).
 *
 *  Most of the time, the sub-vectors will only hold 1 pathData*. To support
 *  more complex tracks, it can store data for several paths, that would be the
 *  result of a track having other forks on one of the paths after the 1st fork.
 *                              In such a case, the returned vector would
 *         ___1________         contain 2 sub-vectors : one for the path which
 *  -> ___/_0___0.0____\___->   begins at the 1st fork successor, and another
 *            \__0.1___/        for the path which begins at the 2nd.
 *              \_0.2_/         The first sub-vector will hold 3 PathData* :
 *                              one for the path 0.0, one for the path 0.1,
 *  and another for the path 0.2.
 *  The AI will then be able to compare these 3 paths with the only PathData*
 *  stored in the 2nd sub-vector (path 1), to take the decision (right or left)
 */
vector<vector<FuzzyAiPathTree::PathData*>*> *FuzzyAiPathTree::getComparableData
                                                   (const TreeNode* root) //const
{
    vector<vector<PathData*>*> *data = new vector<vector<PathData*>*>();
//    cout << "getComparableData debug :";
    
    if(root->children)
    {
//        cout << "has Children : " << root->nodeId << endl;
        vector<vector<PathData*>*> *childData;
        for(unsigned int i=0 ; i<root->children->size() ; i++)
        {
//            cout<<"child "<<i<<", node "<<root->children->at(i)->nodeId<<endl; 
            childData = getComparableData(root->children->at(i));
            for(unsigned int j=0 ; j<childData->size() ; j++)
            {
//                cout << endl << "\tvector " << j << " : " << endl;
                vector<PathData*>* childCurChoiceData = new vector<PathData*>();
                for(unsigned int k=0 ; k<childData->at(j)->size() ; k++)
                {
//                    cout << "\t\t pathData " << k;
                    PathData* curData = childData->at(j)->at(k);
                    PathData* rootData = root->data;

                    if(rootData)
                    {
//                        cout<<" : rootHasData (" << rootData->pathLength;
//                        cout<<")adding childData "<<curData->pathLength<<endl;
                        rootData = new PathData(root->data->pathLength,
                                                root->data->bonusCount,
                                                root->data->malusCount);
                        sumPathData(rootData, rootData, curData);
                    } // if rootData is not NULL
                    else
                    {
//                        cout<<" : no data in root...just copying childData (";
//                        cout << curData->pathLength << endl;
                        rootData = new PathData(curData->pathLength,
                                                curData->bonusCount,
                                                curData->malusCount);
                    } // if rootData is NULL
                    
//                    cout << "\t\t ... adding data to choice vector";
                    childCurChoiceData->push_back(rootData);

                } // for each data object in the current child data vector
                
//                cout << "... adding choice vector to global vector" << endl;
                data->push_back(childCurChoiceData);
            } // for each children data vector
        } // for each children
    } // if has children
    else  // If this is a leave, just add a single data vector to the big vector
    {
//        cout << "no children.. just copying : " << root->nodeId << endl;

        vector<PathData*> *currentData = new vector<PathData*>();
        currentData->push_back(new PathData(root->data->pathLength,
                                            root->data->bonusCount,
                                            root->data->malusCount));
        data->push_back(currentData);
    } // if does not have any children
//    cout << "Function end" << endl;
    return data;
    
} // getComparableData

//------------------------------------------------------------------------------
/** Destructor
 *  Releases the allocated memory for the whole tree and the m_compareData
 *  vectors. */
FuzzyAiPathTree::~FuzzyAiPathTree()
{
    for(unsigned int i=m_compareData->size() ; i > 0 ; i--)
    {
        for(unsigned int j=0 ; j < m_compareData->at(i)->size() ; j++)
            delete m_compareData->at(i)->at(j);
        delete m_compareData->at(i);
    }
    delete m_compareData;
    
    deleteTree(m_treeRoot);
} // ~FuzzyAiPathTree

// -- Recursive delete function called by the destructor --
void FuzzyAiPathTree::deleteTree(TreeNode* rootNode)
{
    if(rootNode->children)
    {
        for(unsigned int i=0; i<rootNode->children->size() ; i++)
            deleteTree(rootNode->children->at(i));
        delete rootNode->children;
    }
    
    if(rootNode->data)
        delete rootNode->data;
    
    delete rootNode;
} // deleteTree

// End of constructor-related functions
//==============================================================================

//------------------------------------------------------------------------------
/** Debug print functions TODO comment
 */
const void FuzzyAiPathTree::printNode(const TreeNode* rootNode)
{
    if(rootNode)
    {
        cout << "[node " << rootNode->nodeId << ", ";
        if(rootNode->data)
            cout << "b" << rootNode->data->bonusCount << ", m" << rootNode->data->malusCount << ", L" << rootNode->data->pathLength << " ";
        if(rootNode->children != NULL)
        {
            for(unsigned int i=0; i<rootNode->children->size() ; i++)
                printNode(rootNode->children->at(i));
        }
        cout << "]";
    }
} // printNode

/** Print initialisation
 */
const void FuzzyAiPathTree::print()
{
    printNode(m_treeRoot);
    cout << endl;
} // print

//------------------------------------------------------------------------------
/** Adds the bonus count, malus count and length of the PathData2 to the
 *  PathData1.
 */
void FuzzyAiPathTree::sumPathData( FuzzyAiPathTree::PathData* result,
                                   const FuzzyAiPathTree::PathData* data1,
                                   const FuzzyAiPathTree::PathData* data2 )
{
    result->bonusCount = data1->bonusCount + data2->bonusCount;
    result->malusCount = data1->malusCount + data2->malusCount;
    result->pathLength = data1->pathLength + data2->pathLength;
}
