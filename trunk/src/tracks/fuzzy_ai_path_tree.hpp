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

#ifndef HEADER_FUZZYAIPATHTREE_HPP
#define HEADER_FUZZYAIPATHTREE_HPP

#include <vector>

class FuzzyAiPathTree
{
public :
//--------------------------------------------------------------------------
/**
  *
  */
struct TreeNode
{
    // -- Tree data --
    unsigned int            quadGraphNodeId;
    std::vector<TreeNode*>* children;
    // -- Road data --
    float                   pathLength;
    unsigned int            bonusCount;
    unsigned int            malusCount;

    TreeNode(unsigned int quad_graph_node_index,
             std::vector<TreeNode*>* children, float path_length,
             unsigned int bonus_count, unsigned int malus_count) :
                quadGraphNodeId(quad_graph_node_index),
                children(NULL),
                pathLength(0.0f),
                bonusCount(0),
                malusCount(0)
            {}
};

private :
    
    TreeNode* m_pathRoot;
    
    TreeNode* buildTree(unsigned int rootNodeId);

public :

    // Constructor
    FuzzyAiPathTree(unsigned int rootNodeId);
    
    // Destructor
    virtual ~FuzzyAiPathTree() { /* TODO clean tree */ }; 
    
    // Debug function
    void const printTree(const TreeNode *rootNode);
    
    // Getter
    const TreeNode* getRoot() const {return m_pathRoot;}
};

#endif /* HEADER_FUZZYAIPATHTREE_HPP */

/* EOF */

