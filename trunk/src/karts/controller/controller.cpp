// $Id$
//
//  SuperTuxKart - a fun racing game with go-kart
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


//The AI debugging works best with just 1 AI kart, so set the number of karts
//to 2 in main.cpp with quickstart and run supertuxkart with the arg -N.

#include <iostream>

#include "io/file_manager.hpp"
#include "ffll/FFLLAPI.h"

#include "karts/controller/controller.hpp"
#include "karts/kart.hpp"

/** Constructor, saves the kart pointer and a pointer to the KartControl
 *  of the kart.
 */
Controller::Controller(Kart *kart, StateManager::ActivePlayer *player)
{
    m_controls = &(kart->getControls());
    m_kart     = kart;
    m_player   = player;
}   // Controller

// ----------------------------------------------------------------------------
const irr::core::stringw& Controller::getNamePostfix() const
{
    // Static to avoid returning the address of a temporary stringq
    static irr::core::stringw name("");
    return name;
}   // getNamePostfix

// ----------------------------------------------------------------------------

//------------------------------------------------------------------------------
/** Generic method to interface with FFLL and compute an output using fuzzy
 *  logic. The first given parameter is the .fcl file that FFLL has to use for
 *  the computation.
 *  The next parameters are the values that correspond to the parameters
 *  declared in the .fcl file (in the same order !).
 *  TODO : better handling of fcl file opening errors (see FuzzyModelBase->load_from_fcl_file())
 *  TODO : make this comment doxygen compliant
 */
/*
float Controller::computeFuzzyModel(const std::string&  file_name,
                                           std::vector<float> parameters )
{
    // Create FFLL model. TODO : make this model a class static variable
	int model = ffll_new_model();

    std::string full_name = file_manager->getFclFile(file_name);
    // Load .fcl file
	int ret_val = (int) ffll_load_fcl_file(model, full_name.c_str());

    // If ffll_load_fcl_file returns an error
	if(ret_val < 0)
	{
		std::cout << "FFLL : Error opening .fcl file '" << file_name << "'" << std::endl; // TODO use fprintf(stderr, msg);
		return ret_val;
	}
    
    // Create a child FFLL model
	int child = ffll_new_child(model);

    // Set parameters value.
	for (size_t i=0, size=parameters.size(); i < size; i++)
		ffll_set_value(model, child, i, parameters[i]); 

    // Compute and return output
	return (float) ffll_get_output_value(model, child);
}
*/
