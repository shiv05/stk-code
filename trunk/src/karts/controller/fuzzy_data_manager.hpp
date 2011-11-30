//  $Id: fuzzy_data_manager.hpp 10225 2011-11-22 15:21:33Z Kinsu $
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

#ifndef HEADER_FUZZYDATAMANAGER_HPP
#define HEADER_FUZZYDATAMANAGER_HPP

#include <vector>

class  QuadNode;
class  FuzzyAiPathTree;
struct PathData;
struct TreeNode;

/** The FuzzyDataManager is used by multiple instances of the FuzzyAIController,
 *  to store and retrieve common data used for the AI modules' computation.
 *  - Player data (currently 1 player supported)
 *      - player crash count, updated by the PlayerController,
 *      - player average rank, updated by the PlayerController,
 *      - TODO : player traveled distance
 *  - Racetrack data
 *      - The different possible paths
 *      - Bonus count on each path
 *      - Malus count on each path
 *      - 
 */

class FuzzyDataManager
{
private :
    // -- Player data --
    int         m_player_crash_count;   // TODO correct value : not true in game
    float       m_player_average_rank;
    // TODO : traveled distance
    // TODO : store player data for multiple players

    // -- Racetrack data --
    std::vector<FuzzyAiPathTree*> m_pathTrees;
        
    // Get player kart index -- Not useful currently as only 1 player is handled
//    int         getPlayerKartId();

public :

    FuzzyDataManager();
    ~FuzzyDataManager(); // TODO clean everything (Fuzzy AI Path)
    // -- Setters --
    // Set the number of crashes (called by PlayerController)
    void        setPlayerCrashCount(int c)    { m_player_crash_count = c; }
    
    // Set player average rank (called by PlayerController)
    void        setPlayerAverageRank(float r) { m_player_average_rank = r; }
    
    // -- Getters --
    int         getPlayerCrashCount()         { return m_player_crash_count; }
    float       getPlayerAverageRank()        { return m_player_average_rank; }
    
    const std::vector<std::vector<PathData*>*>* getPathData(
                                                    unsigned int nodeId ) const;
    
    // Test
    void        createPathTrees();
    
};

extern FuzzyDataManager *fuzzy_data_manager;
#endif /* HEADER_FUZZYDATAMANAGER_HPP */

/* EOF */
