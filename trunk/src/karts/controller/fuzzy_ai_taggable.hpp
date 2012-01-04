// $Id: fuzzy_ai_controller.hpp 10039 2011-12-14 11:39:14Z kinsu $ 
// ID TODO KINSU
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2005 Steve Baker <sjbaker1@airmail.net>
//  Copyright (C) 2006-2007 Eduardo Hernandez Munoz
//  Copyright (C) 2010      Joerg Henrichs
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

#define AI_DEBUG
#ifndef HEADER_FUZZY_AI_TAGGABLE_HPP
#define HEADER_FUZZY_AI_TAGGABLE_HPP

#include <iostream>

#include "utils/vec3.hpp"

namespace irr
{
    namespace scene
    {
        class IBillboardTextSceneNode;
    }
}

/**-----------------------------------------------------------------------------
 * This partially virtual class is used by the FuzzyAIController to store
 * internal data about objects from the environment. This data is then used e.g.
 * to compute the direction the AI wants to take, depending on the interest of
 * each taggable.
 */

class FuzzyAITaggable
{
private :
//    float m_intTag;       // Interest tag
//    float m_diffTag;      // Difficulty tag (-> difficulty to reach the object)
//    float m_attraction;   // Attraction value of the object
    
public :
    FuzzyAITaggable();
    virtual ~FuzzyAITaggable();
    
    // Every taggable object must be able to give its location
    virtual const Vec3& getXYZ() const = 0;
    
    // -- Getters --
//    float   getInterest()          const {return m_intTag;       }
//    float   getDifficulty()        const {return m_diffTag;      }
//    float   getAttraction()        const {return m_attraction;   }
//
//    // -- Setters --
//    void    setInterest  (float newInt ) {m_intTag = newInt;     }
//    void    setDifficulty(float newDiff) {m_diffTag = newDiff;   }
//    void    setAttraction(float newAttr) {m_attraction = newAttr;}


// -- Debug tools --
#ifdef AI_DEBUG
private :
    // Debug text to display above the object
    irr::scene::IBillboardTextSceneNode* debugText;
    // Debug text Y offset (relatively to the object)
    float m_dbgTxtY;
    // Instance counter, used to make debug texts Y coord vary from an instance
    //  to another
    static unsigned int instanceCount;

public :
    void initDebug();
    void setDebugText(const std::string& newText);
    void updateDebugTextPosition();
//    void setTextXYZ(Vec3& newXYZ)      {text.set....}

#endif // AI_DEBUG (end of debug tools declaration)

}; // class FuzzyAITaggable

#endif // HEADER_FUZZY_AI_TAGGABLE_HPP

/* EOF */
