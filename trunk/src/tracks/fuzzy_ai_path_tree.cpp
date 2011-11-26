//  $Id: fuzzy_ai_path_tree.cpp 10225 2011-11-26 12:31:33Z Kinsu $
// TODO ID...
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

using namespace std;

//------------------------------------------------------------------------------
/** Constructor
 */
FuzzyAiPathTree::FuzzyAiPathTree(unsigned int rootNodeId = 0)
{
    m_pathRoot = buildTree(rootNodeId);
}

//------------------------------------------------------------------------------
/**
 */
FuzzyAiPathTree::TreeNode* FuzzyAiPathTree::buildTree(unsigned int rootNodeId)
{
    unsigned int        curNodeId = rootNodeId;
    TreeNode*           rootNode = new TreeNode(rootNodeId, NULL, 0.0f, 1, 1);

//    TreeNode               *curTree;
    vector<unsigned int>    nextGraphNodes(1, curNodeId); // Init to enter loop
    
    while(nextGraphNodes.size() == 1 && nextGraphNodes[0] == curNodeId)
    {
        nextGraphNodes.clear();
        QuadGraph::get()->getSuccessors(curNodeId, nextGraphNodes, true);
        curNodeId ++;
    }
    
    if(nextGraphNodes.size() > 1) // if fork
    {
        rootNode->children = new vector<TreeNode*>();
        for(unsigned int i=0 ; i<nextGraphNodes.size() ; i++)
            rootNode->children->push_back(buildTree(nextGraphNodes[i])); // Build sub-trees
    }
    return rootNode;
}

//------------------------------------------------------------------------------
/**
 */
const void FuzzyAiPathTree::printTree(const TreeNode* rootNode)
{
    if(rootNode)
    {
        cout << "[node " << rootNode->quadGraphNodeId << " ";
        if(rootNode->children != NULL)
        {
            for(unsigned int i=0; i<rootNode->children->size() ; i++)
                printTree(rootNode->children->at(i));
        }
        cout << "]";
    }
    else
        cout << endl;
}

