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


// -- Debug class variable --
#ifdef AI_DEBUG
    unsigned int FuzzyAITaggable::instanceCount = 0;
#endif

/**-----------------------------------------------------------------------------
 * Constructor & destructor
 */
FuzzyAITaggable::FuzzyAITaggable()
{
//    m_intTag     = -1;
//    m_diffTag    = -1;
//    m_attraction = -1;
    
// -- Debug stuff --
#ifdef AI_DEBUG
    debugText = NULL;
    m_dbgTxtY = 0.f;
    FuzzyAITaggable::instanceCount ++;
#endif
}

FuzzyAITaggable::~FuzzyAITaggable()
{}

//==============================================================================
// Debug functions

#ifdef AI_DEBUG
/**-----------------------------------------------------------------------------
 * Debug initialization function : adds a billboardTextSceneNode in the Irrlicht
 * scene, above the object. The final Y coord of the text depends on the
 * instance count to avoid close object debug texts to overlap.

 * TODO : make this comment Doxygen compliant
 */
void FuzzyAITaggable::initDebug()
{
    m_dbgTxtY = 1.0f + (FuzzyAITaggable::instanceCount + rand())%3;
    float x  = getXYZ().getX();
    float y  = getXYZ().getY() + m_dbgTxtY;
    float z  = getXYZ().getZ();
    vector3d<float> pos = vector3d<float>(x, y, z);
    debugText = irr_driver->getSceneManager()->addBillboardTextSceneNode( 0,0,0,
                                       core::dimension2d< f32 >(1.f, 1.f), pos);
} // initDebug

/**-----------------------------------------------------------------------------
 * Debug text setter, simply replaces the current displayed text by the one
 * given in parameter.
 */
void FuzzyAITaggable::setDebugText(const std::string& newText)
{
    if(debugText == NULL)
        initDebug(); // TODO print warning in this case?

    float width = newText.size()*0.5f;
    std::wstring newTextw = std::wstring(newText.begin(), newText.end());
    debugText->setText(newTextw.c_str());
    debugText->setSize(core::dimension2d< f32 >(width, 1.f));
} // setDebugText

/**-----------------------------------------------------------------------------
 */
// TODO : does not work with moveable (kart)...
void FuzzyAITaggable::updateDebugTextPosition()
{
    float x = getXYZ().getX();
    float y = getXYZ().getY() + m_dbgTxtY;
    float z = getXYZ().getZ();
    debugText->setPosition(vector3df(x, y, z));
} // updateDebugTextPosition

#endif // ifdef AI_DEBUG (end of debug functions definition)
