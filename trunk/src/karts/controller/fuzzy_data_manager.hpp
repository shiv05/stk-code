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

class QuadNode;
class FuzzyAiPathTree;
/** The FuzzyDataManager is used by multiple instances of the FuzzyAIController,
 *  to store and retrieve data used for the AI modules' computation.
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
    int         m_player_crash_count;
    float       m_player_average_rank;
    // TODO : traveled distance
    // TODO : store player data for multiple players


    // -- Racetrack data --
    
    /** This structure represents a branch of a FuzzyAiPath.
     *  It is used by the fuzzy ai to compare the possible paths in case of
     *  fork, storing data about the paths.
     *  The given bonus and malus count are the ones of the "main path" (ie. the
     *  branch which has its 2nd quad index = 1st quad index + 1 ), and are to
     *  be compared with the values in the recursively stored FuzzyAiPathBranch.
     * TODO better comment, explain why there is recursivity.
     */
//    struct FuzzyAiPathBranch
//    {
//        unsigned int       fork_quad_id;
//        unsigned int       end_quad_id;
//        unsigned int       main_path_bonus_count;
//        unsigned int       main_path_malus_count;
//        FuzzyAiPathBranch* alt_paths;
//        
//        FuzzyAiPathBranch(unsigned int fork_quad_id,
//                          unsigned int main_path_bonus_count,
//                          unsigned int main_path_malus_count,
//                          FuzzyAiPathBranch pathBranch) :
//                   fork_quad_id(0),
//                   main_path_bonus_count(0),
//                   main_path_malus_count(0),
//                   pathBranch(NULL)
//                {}
//    };
    
    /** This structure extends the quadgraph to store data about paths (eg.
     *  bonus count). This data is used by the fuzzy ai controller to choose
     *  which path to take. */
     // TODO what is called "node_indexes" might actually be the quad indexes
    struct FuzzyAiPath
    {
        std::vector<unsigned int>  *node_indexes;
        bool                        discovered;
        unsigned int                bonus_count;
        unsigned int                malus_count;
//        std::vector<FuzzyAiSubPath> subpaths;
        // TODO turn count, zipper_count ?
        
        FuzzyAiPath(std::vector<unsigned int> *n_indexes,
                      bool has_been_discovered, unsigned int bonus_count,
                      unsigned int malus_count) :
                node_indexes(n_indexes),
                discovered(false),
                bonus_count(0),
                malus_count(0)
            {}
    };
    
    // Possible paths vector
    std::vector<FuzzyAiPath *> *m_possible_paths;
    
    FuzzyAiPathTree            *m_fork_trees;
    
    // -- Methods --
//    void ComputePossiblePaths();
    // Get player kart index -- Not useful currently as only 1 player is handled
//    int         getPlayerKartId();

public :

    FuzzyDataManager();
    //virtual ~FuzzyDataManager(); // TODO clean everything (Fuzzy AI Path)
                                   // TODO init function, and clear function
                                   // compute paths in the init function
                                   // call clear when the track is unloaded.
    // -- Setters --
    // Set the number of crashes (called by PlayerController)
    void        setPlayerCrashCount(int c)    { m_player_crash_count = c; }
    
    // Set player average rank (called by PlayerController)
    void        setPlayerAverageRank(float r) { m_player_average_rank = r; }
    
    // -- Getters --
    // Get player crash count
    int         getPlayerCrashCount()         { return m_player_crash_count; }
    
    // Get player average rank
    float       getPlayerAverageRank()        { return m_player_average_rank; }
    
    // Test
    void        computePossiblePaths();
    void        setPathsItemCount();
};

extern FuzzyDataManager *fuzzy_data_manager;
#endif /* HEADER_FUZZYDATAMANAGER_HPP */

/* EOF */
