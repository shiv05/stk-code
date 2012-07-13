//  $Id$
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2005 Steve Baker <sjbaker1@airmail.net>
//  Copyright (C) 2006-2009 Joerg Henrichs, Steve Baker
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

#include <iostream>

#include "karts/controller/player_controller.hpp"

#include "audio/sfx_base.hpp"
#include "audio/sfx_manager.hpp"
#include "config/player.hpp"
#include "graphics/camera.hpp"
#include "graphics/irr_driver.hpp"
#include "input/input_manager.hpp"
#include "items/item.hpp"
#include "modes/world.hpp"
#include "race/history.hpp"
#include "states_screens/race_gui_base.hpp"
#include "utils/constants.hpp"
#include "utils/translation.hpp"
#include "karts/controller/fuzzy_data_manager.hpp"

using namespace std;

/** The constructor for a player kart.
 *  \param kart_name Name of the kart.
 *  \param position The starting position (1 to n).
 *  \param player The player to which this kart belongs.
 *  \param init_pos The start coordinates and heading of the kart.
 *  \param player_index  Index of the player kart.
 */
PlayerController::PlayerController(Kart *kart, StateManager::ActivePlayer *player, 
                                   unsigned int player_index)      
                : Controller(kart)
{
    assert(player != NULL);
    m_player       = player;
    m_player->setKart(kart);
    m_penalty_time = 0.0f;
    kart->setCamera(new Camera(player_index, kart));
    kart->getCamera()->setMode(Camera::CM_NORMAL);

    m_bzzt_sound  = sfx_manager->createSoundSource( "bzzt" );
    m_wee_sound   = sfx_manager->createSoundSource( "wee"  );
    m_ugh_sound   = sfx_manager->createSoundSource( "ugh"  );
    m_grab_sound  = sfx_manager->createSoundSource( "grab_collectable" );
    m_full_sound  = sfx_manager->createSoundSource( "energy_bar_full" );

    // Player evaluation variables
    m_timer        = 0.f;
    m_old_speed    = 0.f;
    
    reset();
}   // PlayerController

//-----------------------------------------------------------------------------
/** Destructor for a player kart.
 */
PlayerController::~PlayerController()
{
    sfx_manager->deleteSFX(m_bzzt_sound);
    sfx_manager->deleteSFX(m_wee_sound );
    sfx_manager->deleteSFX(m_ugh_sound );
    sfx_manager->deleteSFX(m_grab_sound);
    sfx_manager->deleteSFX(m_full_sound);
}   // ~PlayerController

//-----------------------------------------------------------------------------
/** Resets the player kart for a new or restarted race.
 */
void PlayerController::reset()
{
    m_steer_val_l  = 0;
    m_steer_val_r  = 0;
    m_steer_val    = 0;
    m_prev_brake   = 0;
    m_prev_accel   = 0;
    m_penalty_time = 0;
    m_crash_count  = 0;
    m_average_rank = (float)m_kart->getPosition();
    m_eval         = 0;
    Controller::reset();
}   // reset

// ----------------------------------------------------------------------------
/** Resets the state of control keys. This is used after the in-game menu to
 *  avoid that any keys pressed at the time the menu is opened are still 
 *  considered to be pressed.
 */
void PlayerController::resetInputState()
{
    m_steer_val_l           = 0;
    m_steer_val_r           = 0;
    m_steer_val             = 0;
    m_prev_brake            = 0;
    m_prev_accel            = 0;
    m_controls->m_accel     = 0.0f;
    m_controls->m_brake     = false;
    m_controls->m_drift     = false;
    m_controls->m_fire      = false;
    m_controls->m_look_back = false;
    m_controls->m_nitro     = false;
    m_controls->m_rescue    = false;
    m_controls->m_steer     = 0.0f;

}   // resetKeyState

// ----------------------------------------------------------------------------
/** This function interprets a kart action and value, and set the corresponding
 *  entries in the kart control data structure. This function handles esp. 
 *  cases like 'press left, press right, release right' - in this case after
 *  releasing right, the steering must switch to left again. Similarly it 
 *  handles 'press left, press right, release left' (in which case still
 *  right must be selected). Similarly for braking and acceleration.
 * \param action  The action to be executed.
 * \param value   If 32768, it indicates a digital value of 'fully set'
 *                if between 1 and 32767, it indicates an analog value,
 *                and if it's 0 it indicates that the corresponding button
 *                was released.
 */
void PlayerController::action(PlayerAction action, int value)
{
    switch (action)
    {
    case PA_STEER_LEFT:
        m_steer_val_l = value;
        if (value)
          m_steer_val = value;
        else
          m_steer_val = m_steer_val_r;

        break;
    case PA_STEER_RIGHT:
        m_steer_val_r = -value;
        if (value)
          m_steer_val = -value;
        else
          m_steer_val = m_steer_val_l;

        break;
    case PA_ACCEL:
        m_prev_accel = value;
        if (value && !(m_penalty_time > 0.0f))
        {
            m_controls->m_accel = value/32768.0f;
            m_controls->m_brake = false;
        }
        else
        {
            m_controls->m_accel = 0.0f;
            m_controls->m_brake = m_prev_brake;
        }
        break;
    case PA_BRAKE:
        m_prev_brake = value!=0;
        // let's consider below that to be a deadzone
        if(value > 32768/2)
        {
            m_controls->m_brake = true;
            m_controls->m_accel = 0.0f;
        }
        else
        {
            m_controls->m_brake = false;
            m_controls->m_accel = m_prev_accel/32768.0f;
        }
        break;
    case PA_NITRO:
        m_controls->m_nitro = (value!=0);
        break;
    case PA_RESCUE:
        m_controls->m_rescue = (value!=0);
        break;
    case PA_FIRE:
        m_controls->m_fire = (value!=0);
        break;
    case PA_LOOK_BACK:
        m_controls->m_look_back = (value!=0);
        break;
    case PA_DRIFT:
        m_controls->m_drift = (value!=0);
        break;
    case PA_PAUSE_RACE:
        if (value != 0) StateManager::get()->escapePressed();
        break;
    default: 
       break;
    }

}   // action

//-----------------------------------------------------------------------------
/** Handles steering for a player kart.
 */
void PlayerController::steer(float dt, int steer_val)
{
    if(UserConfigParams::m_gamepad_debug)
    {
        printf("steering: steer_val %d ", steer_val);
    }
    const float STEER_CHANGE = dt/m_kart->getTimeFullSteer();  // amount the steering is changed
    if (steer_val < 0)
    {
      // If we got analog values do not cumulate.
      if (steer_val > -32767)
        m_controls->m_steer = -steer_val/32767.0f;
      else
        m_controls->m_steer += STEER_CHANGE;
    }
    else if(steer_val > 0)
    {
      // If we got analog values do not cumulate.
      if (steer_val < 32767)
        m_controls->m_steer = -steer_val/32767.0f;
      else
        m_controls->m_steer -= STEER_CHANGE;
    }
    else
    {   // no key is pressed
        if(m_controls->m_steer>0.0f)
        {
            m_controls->m_steer -= STEER_CHANGE;
            if(m_controls->m_steer<0.0f) m_controls->m_steer=0.0f;
        }
        else
        {   // m_controls->m_steer<=0.0f;
            m_controls->m_steer += STEER_CHANGE;
            if(m_controls->m_steer>0.0f) m_controls->m_steer=0.0f;
        }   // if m_controls->m_steer<=0.0f
    }   // no key is pressed
    if(UserConfigParams::m_gamepad_debug)
    {
        printf("  set to: %f\n", m_controls->m_steer);
    }

    m_controls->m_steer = std::min(1.0f, std::max(-1.0f, m_controls->m_steer));

}   // steer

//-----------------------------------------------------------------------------
/** Updates the player kart, called once each timestep.
 */
void PlayerController::update(float dt)
{
#ifdef THIS_CAN_BE_REMOVED
    m_controls->m_look_back = false;
    m_controls->m_nitro     = false;

    static float min_x =  99999;
    static float max_x = -99999;
    static float min_z =  99999;
    static float max_z = -99999;
    const Vec3 &xyz = m_kart->getXYZ();
    min_x = std::min(min_x, xyz.getX());
    max_x = std::max(max_x, xyz.getX());
    min_z = std::min(min_z, xyz.getZ());
    max_z = std::max(max_z, xyz.getZ());
    m_controls->m_accel = 1.0f;
    m_controls->m_brake = false;
    m_controls->m_steer = 1.0f;
    printf("xyz %f %f %f  hpr  %f %f %f x %f %f y %f %f r %f %f\n",
        m_kart->getXYZ().getX(),
        m_kart->getXYZ().getY(),
        m_kart->getXYZ().getZ(),
        m_kart->getHeading(),
        m_kart->getPitch(),
        m_kart->getRoll(),
        min_x, max_x, min_z, max_z,
        0.5f*(max_x-min_x),
        0.5f*(max_z-min_z)
        );
    return;
#endif
    // Don't do steering if it's replay. In position only replay it doesn't 
    // matter, but if it's physics replay the gradual steering causes 
    // incorrect results, since the stored values are already adjusted.
    if(!history->replayHistory())
        steer(dt, m_steer_val);

    if(World::getWorld()->isStartPhase())
    {
        if(m_controls->m_accel || m_controls->m_brake ||
           m_controls->m_fire  || m_controls->m_nitro)
        {
            if (m_penalty_time == 0.0)//eliminates machine-gun-effect for SOUND_BZZT
            {
                RaceGUIBase* m=World::getWorld()->getRaceGUI();
                if(m)
                {
                    m->addMessage(_("Penalty time!!"), m_kart, 2.0f,
                                  video::SColor(255, 255, 128, 0));
                    m->addMessage(_("Don't accelerate before go"), m_kart, 2.0f,
                                  video::SColor(255, 210, 100, 50));
                }
                m_bzzt_sound->play();
            }   // if penalty_time = 0
            
            m_penalty_time      = stk_config->m_penalty_time;
            m_controls->m_brake = false;
            m_controls->m_accel = 0.0f;
            
        }   // if key pressed
        else
        {
            // The call to update is necessary here (even though the kart
            // shouldn't actually change) to update m_transform. Otherwise
            // the camera gets the wrong position. 
            Controller::update(dt);
        }
        return;
    }   // if isStartPhase

    if(m_penalty_time>0.0)
    {
        m_penalty_time-=dt;
        return;
    }

    if ( m_controls->m_fire && !m_kart->playingEmergencyAnimation())
    {
        if (m_kart->getPowerup()->getType()==PowerupManager::POWERUP_NOTHING) 
            m_kart->beep();
    }
    
    // look backward when the player requests or
    // if automatic reverse camera is active
    if (m_kart->getCamera()->getMode() != Camera::CM_FINAL)
    {
        if (m_controls->m_look_back || (UserConfigParams::m_reverse_look_threshold>0 && 
            m_kart->getSpeed()<-UserConfigParams::m_reverse_look_threshold))
        {
            m_kart->getCamera()->setMode(Camera::CM_REVERSE);
        }
        else
        {
            if (m_kart->getCamera()->getMode() == Camera::CM_REVERSE)
                m_kart->getCamera()->setMode(Camera::CM_NORMAL);
        }
    }
    
    // We can't restrict rescue to fulfil isOnGround() (which would be more like
    // MK), since e.g. in the City track it is possible for the kart to end
    // up sitting on a brick wall, with all wheels in the air :((
    if ( m_controls->m_rescue )
    {
        m_kart->forceRescue();
        m_controls->m_rescue=false;
    }
    if (m_kart->playingEmergencyAnimation() && 
        m_kart->getAttachment()->getType() != Attachment::ATTACH_TINYTUX)
    {
        m_bzzt_sound->play();
    }
    
    // -- Player evaluation for fuzzy ai controllers --
    m_timer += dt;
    if(m_timer >= 1.0f) // every second, compute the average rank
    {
        m_timer -= 1.0f;

        // This formula takes around 15sec to converge towards a new rank
        m_average_rank = (m_average_rank*4 + m_kart->getPosition())/5;

        if(m_crash_count > 0)
        {
            //Decrement crash count
            m_crash_count -= 0.1f * (floor(m_crash_count + 0.5f));
        }
        
        // Update player evaluation
        m_eval = computePlayerEvaluation(m_average_rank, m_crash_count);
        fuzzy_data_manager->setPlayerEvaluation(m_eval);
    }
 
    // When the kart speed goes under 10mps, this is considered as a crash
    if(m_kart->getSpeed() < 10.0f && m_old_speed > 10.0f)
        m_crash_count += 1.0f;

    m_old_speed = m_kart->getSpeed();

    Controller::update(dt);
}   // update

//-----------------------------------------------------------------------------
/** Checks if the kart was overtaken, and if so plays a sound
*/
void PlayerController::setPosition(int p)
{
    if(m_kart->getPosition()<p)
    {
        World *world = World::getWorld();
        //have the kart that did the passing beep.
        //I'm not sure if this method of finding the passing kart is fail-safe.
        for(unsigned int i = 0 ; i < world->getNumKarts(); i++ )
        {
            Kart *kart = world->getKart(i);
            if(kart->getPosition() == p + 1)
            {
                kart->beep();
                break;
            }
        }
    }
}   // setPosition

//-----------------------------------------------------------------------------
/** Called when a kart finishes race.
 *  /param time Finishing time for this kart.
 */
void PlayerController::finishedRace(float time)
{
    
}   // finishedRace

//-----------------------------------------------------------------------------
/** Called when a kart hits or uses a zipper.
 */
void PlayerController::handleZipper(bool play_sound)
{
    // Only play a zipper sound if it's not already playing, and
    // if the material has changed (to avoid machine gun effect
    // on conveyor belt zippers).
    if (play_sound || (m_wee_sound->getStatus() != SFXManager::SFX_PLAYING &&
                       m_kart->getMaterial()!=m_kart->getLastMaterial()      ) )
    {
        m_wee_sound->play();
    }
    
    // Apply the motion blur according to the speed of the kart
    irr_driver->getPostProcessing()->giveBoost();
    
    m_kart->showZipperFire();

}   // handleZipper

//-----------------------------------------------------------------------------
/** Called when a kart hits an item.
 *  \param item Item that was collected.
 *  \param add_info Additional info to be used then handling the item. If
 *                  this is -1 (default), the item type is selected 
 *                  randomly. Otherwise it contains the powerup or 
 *                  attachment for the kart. This is used in network mode to 
 *                  let the server determine the powerup/attachment for
 *                  the clients.
 */
void PlayerController::collectedItem(const Item &item, int add_info, float old_energy)
{
    if (old_energy < MAX_NITRO &&
        m_kart->getEnergy() == MAX_NITRO)
    {
        m_full_sound->play();
    }
    else if (race_manager->getCoinTarget() > 0 &&
             old_energy < race_manager->getCoinTarget() &&
             m_kart->getEnergy() == race_manager->getCoinTarget())
    {
        m_full_sound->play();
    }
    else
    {
        switch(item.getType())
        {
        case Item::ITEM_BANANA:
            m_ugh_sound->play();
            break;
        case Item::ITEM_BUBBLEGUM:
            //More sounds are played by the kart class
            //See Kart::collectedItem()
            m_ugh_sound->play();
            break;
        default:
            m_grab_sound->play();
            break; 
        }           
    }
}   // collectedItem

//------------------------------------------------------------------------------
/** Player evaluation computation method.
 *  TODO : make this comment doxygen compliant
 */
int PlayerController::computePlayerEvaluation(float playerAverageRank,
                                               float playerCrashCount)
{
//    const std::string &fileName = "player_evaluation.fcl";

	// Normalize rank (0 <= rank <= 10)
    float normRank = ((playerAverageRank-1)*10) /
                                           (World::getWorld()->getNumKarts()-1);
    
    //cout << "Kartn = " << kartCount << ", Av" << playerAverageRank << ", NAvRk = " << normalized_player_average_rank << ", CC = " << playerCrashCount << endl;
//    vector<float> evaluationParameters;
//    evaluationParameters.push_back(normalized_player_average_rank);
//    evaluationParameters.push_back(playerCrashCount);

    // -- player evaluation --
    int eval;
    if(normRank >= 7.65)                // Bad rank
        eval = 3;                       //     => bad player.
    else if(normRank >= 2.75)           // Average Rank...
    {                                   // and
        if(playerCrashCount >= 3.25)    // lots of crashes recently
            eval = 3;                   //     => bad player.
        else                            // few crashes
            eval = 2;                   //     => average player.
    }
    else //if(playerAverageRank >= 0)   // Good Rank...
    {                                   // and
        if(playerCrashCount >= 3.25)    // lots of crashes recently
            eval = 2;                   //     => average player.
        else                            // few crashes
            eval = 1;                   //     => good player.
    }
    
    return eval;// (int) computeFuzzyModel(fileName, evaluationParameters);
}