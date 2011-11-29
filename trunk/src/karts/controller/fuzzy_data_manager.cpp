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
#include "items/item_manager.hpp"
#include "tracks/fuzzy_ai_path_tree.hpp"

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
    
    // -- Path data --
    m_possible_paths = new vector<FuzzyAiPath*>();
}

//------------------------------------------------------------------------------
/** 
 *  
 */
void FuzzyDataManager::computePossiblePaths()
{
    assert(QuadGraph::get()); // Cannot compute path if QuadGraph does not exist
    assert(m_possible_paths->size() == 0);// This method must be called only once per race
    
    std::vector<unsigned int> next;
    
    // Prepare main path
    vector<unsigned int> * mainPathNodes = new vector<unsigned int>();
    mainPathNodes->push_back(0); // begins with the 0 indexed node
    FuzzyAiPath newPath(mainPathNodes, false, 0, 0);
    m_possible_paths->push_back(&newPath);
    
    // Discover the paths in the path list (the main path first, alt.paths then)
    for(unsigned int i=0 ; i < m_possible_paths->size() ; i++)
    {
        // Don't discover already discovered paths
        if(m_possible_paths->at(i)->discovered)
            continue;
        
        vector<unsigned int> *currentPathNodes = m_possible_paths->at(i)->node_indexes;
        unsigned int last_node_index = currentPathNodes->back();
        unsigned int next_node_index;
        
        do  // Follow the current path until its end & detect forks (=new paths)
        {
            next_node_index = 0;
            next.clear();
            
            // Get the successors of the last known node of the current path
            QuadGraph::get()->getSuccessors(last_node_index, next, true);
            
            if(next.size() > 1)      // In case of fork (ie. several successors)
            {
                // Create an undiscovered path in the list for each successor...
                for(unsigned int k=0 ; k < next.size() ; k++)
                {
                    // ...except for the successor of the current path
                    if(next[k] == last_node_index+1)
                    {
                        next_node_index = k;
                        continue;
                    }
                    
                    // Add the detected (but undiscovered yet) path to the list
                    vector<unsigned int> *newPathNodes = new vector<unsigned int>();
                    newPathNodes->push_back(last_node_index); // 1st node (fork)
                    newPathNodes->push_back(next[k]);         // 2nd node
                    FuzzyAiPath newPath(newPathNodes, false, 0, 0);
                    m_possible_paths->push_back(&newPath);
                } // Add the newly detected path(s) to the path list
            } // If the current node is a path fork
            
            // Append the node to the current path, and go to the next node
            currentPathNodes->push_back(next[next_node_index]);            
            last_node_index++;

        // Stop when the next node index is not the current node index + 1
        } while(next[next_node_index] == last_node_index);

        m_possible_paths->at(i)->discovered = true;
    } // For each entry in the path list, discover the path

    setPathsItemCount();
#ifdef AI_DEBUG
    cout << "Found paths : " << endl;
    for(unsigned int i2=0 ; i2 < m_possible_paths->size() ; i2++)
    {
        cout << "\t path " << i2;
        cout << " : bonus_count = " << m_possible_paths->at(i2)->bonus_count;
        cout << ", malus_count = " << m_possible_paths->at(i2)->malus_count << endl;
        for(unsigned int j2=0 ; j2 < m_possible_paths->at(i2)->node_indexes->size(); j2++)
        {
            cout << m_possible_paths->at(i2)->node_indexes->at(j2) << ", ";
        }
        cout << endl;
    }
#endif

    m_fork_trees = new FuzzyAiPathTree(0);
    m_fork_trees->print();
}

//------------------------------------------------------------------------------
/**
 *
 */
void FuzzyDataManager::setPathsItemCount()
{
    for(unsigned int i=0 ; i<m_possible_paths->size() ; i++)
    {
        FuzzyAiPath *currentPath = m_possible_paths->at(i);
        for(unsigned int j=0 ; j<currentPath->node_indexes->size() ; j++)
        {
            const Quad& q = QuadGraph::get()->getQuadOfNode(currentPath->node_indexes->at(j));
            currentPath->bonus_count += item_manager->getQuadBonusCount(q);
            currentPath->malus_count += item_manager->getQuadMalusCount(q);
        }
    }
}

// Returns the world player kart ID. -1 if no player is found.
// Currently considers there is only one player.
//int   FuzzyDataManager::getPlayerKartId()
//{
//    for(unsigned int id=0; id<m_kart_status.size(); id++)
//    {
//        if(m_kart_status[id].m_local_player_id != -1)
//            return id;
//    }
//    return -1;
//}
