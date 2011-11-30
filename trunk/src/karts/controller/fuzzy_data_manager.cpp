//  $Id: fuzzy_data_manager.cpp 10225 2011-11-22 15:21:33Z Kinsu $
// TODO ID
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
#include <vector>
#define AI_DEBUG

#include "karts/controller/fuzzy_data_manager.hpp"
#include "tracks/quad_graph.hpp"
#include "tracks/fuzzy_ai_path_tree.hpp"
#include "items/item_manager.hpp"

using namespace std;

FuzzyDataManager* fuzzy_data_manager = NULL;

//------------------------------------------------------------------------------
/** Constructor
 */
FuzzyDataManager::FuzzyDataManager()
{
    // These variables are set to -1 until the manager is updated by the
    // PlayerController
    m_player_crash_count  = -1;
    m_player_average_rank = -1;
    
    m_pathTrees = vector<FuzzyAiPathTree*>();
} // Constructor 


//------------------------------------------------------------------------------
/** Destructor  
 */
FuzzyDataManager::~FuzzyDataManager()
{
    for(unsigned int i=0 ; i<m_pathTrees.size() ; i++)
        delete m_pathTrees[i];
}

//------------------------------------------------------------------------------
/** Creates a FuzzyAiPath tree for each main path fork.
 *  TODO better comment
 */
void FuzzyDataManager::createPathTrees()
{
    unsigned int         curNode = 0;
    unsigned int         lapLength = QuadGraph::get()->getLapQuadCount();
    vector<unsigned int> nextNodes;
    
    // Go forward on the main driveline
    while(curNode < lapLength )
    {
        nextNodes.clear();
        QuadGraph::get()->getSuccessors(curNode, nextNodes, true);

        if(nextNodes.size() > 1) // If there is a path fork, create a path tree
        {
            m_pathTrees.push_back(new FuzzyAiPathTree(curNode));
        }
        curNode ++;
    } // while
    
    for(unsigned int i=0; i<m_pathTrees.size() ; i++)
    {
        m_pathTrees[i]->print();
    }
} // createPathTrees

//------------------------------------------------------------------------------
/** Returns the pathData vectors from the FuzzyAiPathTree which root has the
 *  given quadGraph node Id. TODO better comment
 *  Returns NULL if no such tree is found.
 */
const vector<vector<PathData*>*>* FuzzyDataManager::getPathData(
                                                      unsigned int nodeId) const
{
    for(unsigned int i=0; i<m_pathTrees.size() ; i++)
    {
        if(m_pathTrees[i]->getRootId() == nodeId)
            return m_pathTrees[i]->getComparableData();
    }
    return NULL;
} // getPathData

//------------------------------------------------------------------------------
/** Returns the world player kart ID. -1 if no player is found.
 *   Currently considers there is only one player.
 */
//int   FuzzyDataManager::getPlayerKartId()
//{
//    for(unsigned int id=0; id<m_kart_status.size(); id++)
//    {
//        if(m_kart_status[id].m_local_player_id != -1)
//            return id;
//    }
//    return -1;
//}
