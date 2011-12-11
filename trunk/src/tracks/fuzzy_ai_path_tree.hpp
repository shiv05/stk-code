//  $Id: fuzzy_ai_path.hpp 10225 2011-11-22 15:21:33Z Kinsu $
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

//#define AI_DEBUG

#ifndef HEADER_FUZZYAIPATHTREE_HPP
#define HEADER_FUZZYAIPATHTREE_HPP

#include <vector>

//------------------------------------------------------------------------------
/** The PathData structure is simply used to store data about a path from the
  * path tree (made of multiple TreeNodes). This data is what will be used by
  * the fuzzy AI to compare the paths :
  *     - Path length,
  *     - bonus item count,
  *     - malus item count.
  *
  * This structure can be extended for the AI to take in account more parameters
  * in its path choice (eg. turnCount, as in the future heavy karts may be more
  * efficient on straigther paths).
  */
struct PathData
{
    float                   pathLength;
    unsigned int            bonusCount;
    unsigned int            malusCount;
//    unsigned int            zipperCount; TODO
//    unsigned int            turnCount; TODO

    PathData(float path_len = 0.f, 
             unsigned int bonus_count = 0,
             unsigned int malus_count = 0) :
        pathLength(path_len),
        bonusCount(bonus_count),
        malusCount(malus_count)
      {}
}; // struct PathData

//------------------------------------------------------------------------------
/** The TreeNode structure is a node of the FuzzyAIPathTree. It contains the Id
  * of a QuadGraph node, pointers toward the potential children nodes, and a
  * PathData instance which holds data about the direct upper part of the tree.
  *
  * The referenced QuadGraph node in a TreeNode depends on the TreeNode
  * situation in the tree : if it is root (including a sub-tree root) the
  * referenced QuadGraph node is located just before a path fork, on the
  * driveline.
  * If it is a tree leave, it references the QuadGraph node on which the paths
  * are considered to end (because they join back the main driveline).
  */

struct TreeNode
{
    // -- Tree data --
    unsigned int            nodeId;
    std::vector<TreeNode*> *children;           // /!\ NULL if no children
    // -- Road data --
    PathData               *data;               // /!\ NULL if no data

    TreeNode(unsigned int     quad_graph_node_index,
             std::vector<TreeNode*>    *children_nodes,
             PathData                  *road_data) :
                nodeId(quad_graph_node_index),
                children(children_nodes),
                data(road_data)
           {}
}; // struct TreeNode


//------------------------------------------------------------------------------
/** This class is used to create a tree of paths based on a main driveline fork.
 *  Each node of the tree corresponds to a path fork, and stores data about the
 *  road section to reach this fork.
 *  All the paths from the tree finish at the first main-driveline node that one
 *  can reach after taking any of the possible paths.
 *      See functions comments for more information about the tree structure.
 *  
 *  The path tree is used by the fuzzy AI to compare paths and choose which one
 *  to take when it meets a fork.
 */
class FuzzyAIPathTree
{
private :
    // -- Variables --
    // Root node for the tree 
    TreeNode* m_treeRoot;
    // Data used to compare the possible paths. See getComparableData for more.
    std::vector<std::vector<PathData*>*> *m_compareData;
    // 
    unsigned int m_treeBottom;

    // -- Constructor & destructor related functions --
    TreeNode*        buildTree       (unsigned int rootNodeId);
    TreeNode*        setPathData     (TreeNode* rootNode,
                                      PathData* pathData = NULL);
    unsigned int     getFarthestNode (const TreeNode* rootNode) const;
    void             deleteTree      (TreeNode* rootNode);

    std::vector<std::vector<PathData*>*> *computeComparableData
                                                   (const TreeNode* root) const;

   
    // -- Misc functions --
    //void             sumPathData     ( const PathData* data1,
    //                                 const PathData* data2, PathData* result);
    const TreeNode*  getTreeNode(unsigned int nodeId, TreeNode* curNode) const;
    // -- Debug functions --
    void printNode(const TreeNode *rootNode) const;
    void printPossiblePaths();

public :

    // -- Constructor & Destructor --
    FuzzyAIPathTree (unsigned int rootNodeId);
    ~FuzzyAIPathTree(); 
    
    // -- Debug functions --
    void print() const;
    
    // -- Getters --    
    const unsigned int   getRootId()    const { return m_treeRoot->nodeId;}
    const TreeNode*      getRoot()      const { return m_treeRoot; }
    
    const TreeNode*      getTreeNode(unsigned int nodeId) const;
    
    void getForkNodes (std::vector<unsigned int> &result) const;
    void getForkNodes (const TreeNode* node,
                       std::vector<unsigned int> &result) const;

    const std::vector<std::vector<PathData*>*>* getComparableData()      const
                                                       { return m_compareData; }
    const std::vector<std::vector<PathData*>*>* getComparableData(
                                                      unsigned int from) const;


}; // class FuzzyAIPathTree

#endif /* HEADER_FUZZYAIPATHTREE_HPP */

/* EOF */
