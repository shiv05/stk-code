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

#include <IBillboardTextSceneNode.h>
#include <ISceneManager.h>
#include <vector3d.h>

#include "karts/controller/fuzzy_ai_taggable.hpp"
#include "graphics/irr_driver.hpp"


using namespace std;
using namespace irr;
using namespace core;
using namespace scene;

unsigned int FuzzyAITaggable::instanceCount = 0;

FuzzyAITaggable::FuzzyAITaggable()
{
    debugText = NULL;
    FuzzyAITaggable::instanceCount ++;
    m_height = 0.f;
}

FuzzyAITaggable::~FuzzyAITaggable()
{}


void FuzzyAITaggable::init()
{
    m_height = 1 + (FuzzyAITaggable::instanceCount + rand())%3;
    float x  = getXYZ().getX();
    float y  = getXYZ().getY() + m_height;
    float z  = getXYZ().getZ();
    vector3d<float> pos = vector3d<float>(x, y, z);
    debugText = irr_driver->getSceneManager()->addBillboardTextSceneNode( 0,0,0,
                                       core::dimension2d< f32 >(1.f, 1.f), pos);
}

void FuzzyAITaggable::setDebugText(const std::string& newText)
{
    if(debugText == NULL)
        init(); // TODO print warning?

    float width = newText.size()/2;
    std::wstring newTextw = std::wstring(newText.begin(), newText.end());
    debugText->setText(newTextw.c_str());
    debugText->setSize(core::dimension2d< f32 >(width, 1.f));
} // setDebugText

// TODO : does not work with moveable (kart)...
void FuzzyAITaggable::updatePosition()
{
    float x = getXYZ().getX();
    float y = getXYZ().getY() + m_height;
    float z = getXYZ().getZ();
    debugText->setPosition(vector3df(x, y, z));
}

