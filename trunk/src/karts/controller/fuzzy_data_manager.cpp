//  $Id: fuzzy_data_manager.cpp 10225 2011-11-22 15:21:33Z Kinsu $
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

#include <iostream>

#include "karts/controller/fuzzy_data_manager.hpp"

FuzzyDataManager* fuzzy_data_manager = NULL;

FuzzyDataManager::FuzzyDataManager()
{
    // These variables are set to -1 until the manager is updated by the
    // PlayerController
    m_player_crash_count = -1;
    m_player_average_rank = -1;
}

// Returns the world player kart ID. -1 if no player is found.
// Currently considers there is only one player.
//int   FuzzyDataManager::getPlayerKartId()
//{
//    for(unsigned int id=0; id<m_kart_status.size(); id++)
//    {
//        if(m_kart_status[id].m_local_player_id != -1)
//            return id;
//    }
//    return -1;
//}
