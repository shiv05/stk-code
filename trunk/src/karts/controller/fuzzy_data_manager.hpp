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
class  FuzzyAIPathTree;
struct PathData;
struct TreeNode;

/**----------------------------------------------------------------------------- 
 * The FuzzyDataManager is used by multiple instances of the FuzzyAIController,
 *  to store and retrieve common data :
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
    int         m_player_eval;
    // TODO : store player data for multiple players

    // -- Racetrack data --
    std::vector<FuzzyAIPathTree*> m_pathTrees;
        
    // Get player kart index -- Not useful currently as only 1 player is handled
//    int         getPlayerKartId();

public :

    FuzzyDataManager();
    ~FuzzyDataManager(); // TODO clean everything (Fuzzy AI Path)
    
    // Reset (race restart)
    void        reset();
    
    // -- Setters --
    void        setPlayerEvaluation(int e) { m_player_eval = e; }
    
    // -- Getters --
    int         getPlayerEvaluation()         { return m_player_eval; }
    
    const std::vector<std::vector<PathData*>*>*
                getPathData(unsigned int nodeId ) const;

    // Todo comment
    void        createPathTrees();    
};

extern FuzzyDataManager *fuzzy_data_manager;
#endif /* HEADER_FUZZYDATAMANAGER_HPP */

/* EOF */
