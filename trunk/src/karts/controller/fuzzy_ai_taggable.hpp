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


class FuzzyAITaggable
{
private :
    irr::scene::IBillboardTextSceneNode* debugText;
    static unsigned int instanceCount; // used to change debug text height to it's easy to see

    float m_height;
public :
    FuzzyAITaggable();
    virtual ~FuzzyAITaggable();
    
    void init();

    virtual const Vec3& getXYZ() const = 0;

    void setDebugText(const std::string& newText);
    void updatePosition();
//    void setTextXYZ(Vec3& newXYZ)      {text.set....}
};

#endif // HEADER_FUZZY_AI_TAGGABLE_HPP

/* EOF */
