// $Id: fuzzy_ai_controller.cpp 10066 2011-10-30 20:37:14Z auria $
// TODO ID KINSU
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2005 Steve Baker <sjbaker1@airmail.net>
//  Copyright (C) 2006-2007 Eduardo Hernandez Munoz
//  Copyright (C) 2008      Joerg Henrichs
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

//#undef AI_DEBUG
#define AI_DEBUG

#include <cstdlib>
#include <ctime>
#include <cstdio>
#include <iostream>
#include <vector>
#include <math.h>

#include <vector2d.h>
#include <IBillboardTextSceneNode.h>

#ifdef AI_DEBUG
#  include "irrlicht.h"
#  include "graphics/irr_driver.hpp"
#endif

#include "ffll/FFLLAPI.h"
#include "io/file_manager.hpp"
#include "graphics/slip_stream.hpp"
#include "modes/linear_world.hpp"
#include "network/network_manager.hpp"
#include "race/race_manager.hpp" // TODO : check if really necessary
#include "karts/controller/fuzzy_ai_controller.hpp"
#include "karts/controller/fuzzy_ai_taggable.hpp"
#include "karts/controller/fuzzy_data_manager.hpp"
#include "items/item.hpp"
#include "items/item_manager.hpp"
#include "tracks/quad_graph.hpp"
#include "tracks/track.hpp"
#include "tracks/fuzzy_ai_path_tree.hpp"
#include "utils/constants.hpp"

using namespace std;
using namespace irr;
using namespace core;

unsigned int FuzzyAIController::instanceCount = 0;

FuzzyAIController::FuzzyAIController(Kart *kart) :
                                                 AIBaseController(kart, NULL, 1)
{
    reset();

    switch( race_manager->getDifficulty())
    {
    case RaceManager::RD_EASY:
        m_wait_for_players        = true;
        m_make_use_of_slipstream  = false;
        m_max_handicap_speed      = 0.9f;
        m_item_tactic             = IT_TEN_SECONDS;
        m_false_start_probability = 0.08f;
        m_min_start_delay         = 0.3f;
        m_max_start_delay         = 0.5f;
        m_min_steps               = 1;
        m_nitro_level             = NITRO_NONE;
        m_handle_bomb             = false;
        setSkiddingFraction(4.0f);
        break;
    case RaceManager::RD_MEDIUM:
        m_wait_for_players        = true;
        m_make_use_of_slipstream  = false;
        m_max_handicap_speed      = 0.95f;
        m_item_tactic             = IT_CALCULATE;
        m_false_start_probability = 0.04f;
        m_min_start_delay         = 0.25f;
        m_max_start_delay         = 0.4f;
        m_min_steps               = 1;
        m_nitro_level             = NITRO_SOME;
        m_handle_bomb             = true;
        setSkiddingFraction(3.0f);
        break;
    case RaceManager::RD_HARD:
        m_wait_for_players        = false;
        m_make_use_of_slipstream  = true;
        m_max_handicap_speed      = 1.0f;
        m_item_tactic             = IT_CALCULATE;
        m_false_start_probability = 0.01f;
        // See http://www.humanbenchmark.com/tests/reactiontime/stats.php
        // Average reaction time is around 0.215 s, so using .15 as minimum
        // gives an AI average slightly above the human average
        m_min_start_delay         = 0.15f;
        m_max_start_delay         = 0.28f;
        m_min_steps               = 2;
        m_nitro_level             = NITRO_ALL;
        m_handle_bomb             = true;
        setSkiddingFraction(2.0f);
        
        break;
    }
    
    // -- Fuzzy controller code --
    FuzzyAIController::instanceCount ++;
    m_instanceID = instanceCount;

    // Player evaluation
    // So that fuzzy computation is well distributed over iterations
    m_timer                 = (0.25f/instanceCount)*instanceCount;
    m_compet                = 1; // TODO Constants
//    m_aggress               = 1;
    m_attrPts               = vector<AttrPoint*>();
    m_item_count            = 0;
    m_chosenDir             = NULL;
    
    computeForkChoices(m_fork_choices);
    computePath();
    
    const Vec3 v=QuadGraph::get()->getQuadOfNode(m_next_node_index[m_track_node]).getCenter();
    m_mainAPt.x = v.getX();
    m_mainAPt.z = v.getZ();
    m_mainAPt.attraction = 6.4f;//6.4 is an "upper medium" value (see difficulty tagging)

    m_attrPts.push_back(&m_mainAPt);
    
    // -- Debug stuff -- TODO IN AI_DEBUG condition (just below)
    if(FuzzyAIController::instanceCount == 1)
        m_debug = true;
    else
        m_debug = false;
    
#ifdef AI_DEBUG
    if(m_debug)
        m_debug_sphere = irr_driver->getSceneManager()->addSphereSceneNode(1);
#endif
}   // FuzzyAIController

//-----------------------------------------------------------------------------
/** The destructor deletes the shared TrackInfo objects if no more FuzzyAIController
 *  instances are around.
 */
FuzzyAIController::~FuzzyAIController()
{
#ifdef AI_DEBUG
    if(m_debug)
        irr_driver->removeNode(m_debug_sphere);
#endif
}   // ~FuzzyAIController

//-----------------------------------------------------------------------------
void FuzzyAIController::reset()
{
    m_time_since_last_shot       = 0.0f;
    m_start_kart_crash_direction = 0;
    m_start_bad_item_crash_direction = 0;
    m_curve_target_speed         = m_kart->getCurrentMaxSpeed();
    m_curve_angle                = 0.0;
    m_start_delay                = -1.0f;
    m_crash_time                 = 0.0f;
    m_collided                   = false;
    m_time_since_stuck           = 0.0f;
    m_kart_ahead                 = NULL;
    m_distance_ahead             = 0.0f;
    m_kart_behind                = NULL;
    m_distance_behind            = 0.0f;
    
//    FuzzyAIController::instanceCount = 0;

    AIBaseController::reset();
    m_track_node               = QuadGraph::UNKNOWN_SECTOR;
    QuadGraph::get()->findRoadSector(m_kart->getXYZ(), &m_track_node);
    if(m_track_node==QuadGraph::UNKNOWN_SECTOR)
    {
        fprintf(stderr, 
                "Invalid starting position for '%s' - not on track"
                " - can be ignored.\n",
                m_kart->getIdent().c_str());
        m_track_node = QuadGraph::get()->findOutOfRoadSector(m_kart->getXYZ());
    }

}   // reset

//-----------------------------------------------------------------------------
const irr::core::stringw& FuzzyAIController::getNamePostfix() const 
{
    // Static to avoid returning the address of a temporary stringq
    static irr::core::stringw name="(fuzzy)";
    return name;
}   // getNamePostfix

//-----------------------------------------------------------------------------
/** Returns the pre-computed successor of a graph node.
 *  \parameter index The index of the graph node for which the successor
 *              is searched.
 */
unsigned int FuzzyAIController::getNextSector(unsigned int index)
{
    return m_successor_index[index];
}   // getNextSector

//-----------------------------------------------------------------------------
//TODO: if the AI is crashing constantly, make it move backwards in a straight
//line, then move forward while turning.
void FuzzyAIController::update(float dt)
{
    // This is used to enable firing an item backwards.
    m_controls->m_look_back = false;
    m_controls->m_nitro     = false;

    // Having a non-moving AI can be useful for debugging, e.g. aiming
    // or slipstreaming.
#undef AI_DOES_NOT_MOVE_FOR_DEBUGGING
#ifdef AI_DOES_NOT_MOVE_FOR_DEBUGGING
    m_controls->m_accel     = 0;
    m_controls->m_steer     = 0;
    return;
#endif

    // The client does not do any AI computations.
    if(network_manager->getMode()==NetworkManager::NW_CLIENT) 
    {
        AIBaseController::update(dt);
        return;
    }

    if( m_world->isStartPhase() )
    {
        handleRaceStart();
        AIBaseController::update(dt);
        return;
    }

    /* Get information that is needed by more than 1 of the handling funcs */
    // Detect if we are going to crash with the track and/or kart
    int steps = 0;

    steps = calcSteps();

    computeNearestKarts();
    checkCrashes( steps, m_kart->getXYZ() );
    findCurve();

    // Special behaviour if we have a bomb attach: try to hit the kart ahead 
    // of us.
    bool commands_set = false;
    if(m_handle_bomb && 
        m_kart->getAttachment()->getType()==Attachment::ATTACH_BOMB && 
        m_kart_ahead )
    {
        // Use nitro if the kart is far ahead, or faster than this kart
        m_controls->m_nitro = m_distance_ahead>10.0f || 
                             m_kart_ahead->getSpeed() > m_kart->getSpeed();
        // If we are close enough, try to hit this kart
        if(m_distance_ahead<=10)
        {
            Vec3 target = m_kart_ahead->getXYZ();

            // If we are faster, try to predict the point where we will hit
            // the other kart
            if(m_kart_ahead->getSpeed() < m_kart->getSpeed())
            {
                float time_till_hit = m_distance_ahead
                                    / (m_kart->getSpeed()-m_kart_ahead->getSpeed());
                target += m_kart_ahead->getVelocity()*time_till_hit;
            }
            float steer_angle = steerToPoint(m_kart_ahead->getXYZ());
            setSteering(steer_angle, dt);
            commands_set = true;
        }
        handleRescue(dt);
    }

    /*And obviously general kart stuff*/
    AIBaseController::update(dt);
    m_collided = false;
    
    //==========================================================================
    // -- Fuzzy controller code --
    Vec3 straight_point;
    findNonCrashingPoint(&straight_point);
    m_mainAPt.x = straight_point.getX();
    m_mainAPt.z = straight_point.getZ();
   
    if(m_last_seen_track_node != m_track_node)
    {
        computeForkChoices(m_fork_choices);
        
        vector<unsigned int> next = vector<unsigned int>();
        QuadGraph::get()->getSuccessors(m_track_node, next, true);
        if(next.size() > 1)
            m_fork_choices.erase(m_fork_choices.begin());
 
        computePath();
    }

    vector<Item*> closeItems;
    item_manager->getCloseItems(closeItems, m_kart, 24.f);
    // If item count has changed, recompute attraction
    if(m_item_count != closeItems.size())
    {
#ifdef AI_DEBUG
        if(m_debug)
            cout << "Item environment has changed, forced updating" << endl;
#endif
        tagItems((const vector<Item*>)closeItems, m_attrPts);
    }
    
    m_timer += dt;
    if(m_timer >= 0.25f)        // every ~1/2 second, do
    {
        m_timer -= 0.25f;
        
        // TODO update item number at each iteration, and recompute attraction if
        // it has changed
        // TODO avoid bananas
        // Item position & type

        // Update item attraction if not already done
        if(closeItems.size()>0 && m_item_count == closeItems.size())
        {
#ifdef AI_DEBUG
            if(m_debug)
                cout << "Item environment has not changed, normal updating" << endl;
#endif
            tagItems((const vector<Item*>)closeItems, m_attrPts);
        }
        
        // Player evaluation        
        int eval = fuzzy_data_manager->getPlayerEvaluation();

        // -- Choose the driving style : competitiveness and agressiveness --
        //Get the current ranking
        int current_ranking = m_kart->getPosition();

        // TODO: Kart classes (heavy, medium, light) are not implemented, so
        // for now we use "medium" for every kart.
        int kart_class = 2;
        m_compet = computeCompetitiveness(eval, current_ranking);
//        m_aggress = 2; //computeAggressiveness(kart_class, current_ranking);
        
        //Decide if it is interesting or not to use the current possessed weapon
        //Get current powerup
//        const Powerup* current_powerup = m_kart->getPowerup();
//        PowerupManager::PowerupType possessed_item = current_powerup->getType();

//        float hit_estimation = 0;
//        float weapon_interest = 0;

#ifdef AI_DEBUG
        if(m_debug)
        {
        cout << "----------------------------------------" << endl;
        // -- Player evaluation --
        cout << " -- PLAYER EVALUATION -- " << endl;
        cout << m_kart->getIdent() << " : player evaluation = ";
        switch(eval)
        {
            case (1): cout << "Good!" << endl;   break;
            case (2): cout << "Average" << endl; break;
            case (3): cout << "Bad!" << endl;    break;
            default : cout << "unexpected value : " << eval << endl;
        } // end switch
        
        // -- Items --
        cout << " -- ITEMS -- " << endl;
        cout << m_kart->getIdent() << " : items = " << closeItems.size() << endl;

        // -- Agent driving style --
        cout << " -- DRIVING STYLE --" << endl;
        cout << m_kart->getIdent() << " : agent competitiveness = ";
        switch(m_compet)
        {
            case (0): cout << "Not competitive" << endl;   break;
            case (1): cout << "Competitive" << endl; break;
            default : cout << "unexpected value : " << eval << endl;
        } // end switch
        
//        cout << m_kart->getIdent() << " : agent agressiveness = ";
//        switch(m_aggress)
//        {
//            case (1): cout << "Agressive" << endl;   break;
//            case (2): cout << "Neutral" << endl; break;
//            case (3): cout << "Careful" << endl; break;
//            default : cout << "unexpected value : " << eval << endl;
//        } // end switch
        }
#endif
    }
    // Update close items count
    m_item_count = closeItems.size();

    if(!commands_set)
    {
        /*Response handling functions*/
        handleAcceleration(dt);
        m_chosenDir = chooseDirection(m_attrPts);
//        if(m_debug)
//            cout << "    Chosen = " << m_chosenDir->attraction << endl;
        handleSteering(dt);
        handleItems(dt);
        handleRescue(dt);
        handleBraking();
        // If a bomb is attached, nitro might already be set.
        if(!m_controls->m_nitro)
            handleNitroAndZipper();
    }
    // If we are supposed to use nitro, but have a zipper, 
    // use the zipper instead
    if(m_controls->m_nitro && 
        m_kart->getPowerup()->getType()==PowerupManager::POWERUP_ZIPPER && 
        m_kart->getSpeed()>1.0f && 
        m_kart->getSpeedIncreaseTimeLeft(MaxSpeed::MS_INCREASE_ZIPPER)<=0)
    {
        // Make sure that not all AI karts use the zipper at the same
        // time in time trial at start up, so during the first 5 seconds
        // this is done at random only.
        if(race_manager->getMinorMode()!=RaceManager::MINOR_MODE_TIME_TRIAL ||
            (m_world->getTime()<3.0f && rand()%50==1) )
        {
            m_controls->m_nitro = false;
            m_controls->m_fire  = true;
        }
    }
}   // update

//------------------------------------------------------------------------------
/** Driving style computation methods for competitiveness and agressiveness.
 *  Simply call computeFuzzyModel with the right parameters.
 *  TODO : make this comment doxygen compliant
 */

int FuzzyAIController::computeCompetitiveness(int           playerEval,
                                              unsigned int  currentRanking)
{
//    const std::string& file_name = "driving_style_competitiveness.fcl";
    
	// Normalize rank (0 <= rank <= 10)
	int kartCount = World::getWorld()->getNumKarts();
	if(kartCount == 1) kartCount++; // to avoid dividing by 0
    float normRank = ((currentRanking-1)*10) / (kartCount - 1);

//    vector<float> evaluationParameters;
//    evaluationParameters.push_back((float)player_level);
//    evaluationParameters.push_back(normalized_current_ranking);
    int compet;
    
    // -- Compute competitiveness --
    if(playerEval == 3)         // Bad player
        compet = 0;             //     => not competitive
    else if(playerEval == 2)    // Average player...
    {                           // and
        if(normRank >= 7.5)     // bad ranking
            compet = 0;         //     => not competitive
        else                    // medium or good ranking
            compet = 1;         //     => competitive
    }
    else if(playerEval == 1)    // Good player
        compet = 1;             //     => competitive
    
    return compet; // (int) computeFuzzyModel(file_name, evaluationParameters);
}

/*
int FuzzyAIController::computeAggressiveness(unsigned int  kart_class,
                                             unsigned int  current_ranking)
{
    const std::string& file_name = "driving_style_agressiveness.fcl";
    
	//The rank needs to be normalized before computing
    float normalized_current_ranking;
    unsigned int kartCount = World::getWorld()->getNumKarts();
    normalized_current_ranking = (current_ranking*10.0f)/kartCount;

    vector<float> evaluationParameters;
    evaluationParameters.push_back(normalized_current_ranking);
    evaluationParameters.push_back((float)kart_class);

    return  (int) 0; // computeFuzzyModel(file_name, evaluationParameters);
}*/

//------------------------------------------------------------------------------
/** Path choice computation method, used when there are multiple paths.
 *  This method evaluates each one of the given paths, and returns the best
 *  path depending on the situation. The evaluation is based on data about the
 *  paths, computed by the FuzzyAIPathTree class.

 *  TODO : Make this comment Doxygen compliant
 *         Improvement for later : Use the number of turns and the kart class
 *         and the item count.
 */
int FuzzyAIController::choosePath(const vector<vector<PathData*>*>* pathData,
                                  float competitiveness)
{
//    const std::string& file_name = "path_chooser.fcl";  // Fcl file
//    vector<float> pathParameters;

    float         bonusCount, length, interest;
    float         bestInterest = 0;
    unsigned int  bestChoice;
    unsigned int  random1, random2;
    
    PathData*     bestPath;
    
    // Evaluate each path
    for (unsigned int i=0; i < pathData->size(); i++)
	{
        for (unsigned int j=0; j < pathData->at(i)->size(); j++)
        {
            // Get the length and bonus count of the path
            length = (float)pathData->at(i)->at(j)->pathLength;
            bonusCount = (float)(pathData->at(i)->at(j)->bonusCount);

//            pathParameters.push_back(length);
//            pathParameters.push_back(bonusCount);
//            pathParameters.push_back(competitiveness);

            // Compute the interest of the path
            interest = (competitiveness==1 ? -1 : 1)*length; // computeFuzzyModel(file_name, pathParameters);
//            pathParameters.clear();

            // Compare the computed interest with the best interest so far
            if(interest > bestInterest)
            {
                // If the path has a better interest than current best, store it
                bestInterest = interest;
                bestChoice = i;
                bestPath = pathData->at(i)->at(j);
            } // if current path has a better interest than current max interest
        } // for each path in the current choice
	} // for each possible choice
	
	// Randomization to avoid every agent from choosing the best path
	random1 = (m_instanceID * rand()) % pathData->size();
	random2 = (m_instanceID * rand() * 3) % pathData->size();
	
	unsigned int choice = (random2 < pathData->size()/2.1)? bestChoice : random1;
	
#ifdef AI_DEBUG
    if(m_debug)
    {
        cout << "**************************************************************" << endl;
        cout << "path fork detected ! choice = " << choice << ", best choice = " << bestChoice << endl;
        cout << "bonusCount = " << bestPath->bonusCount << ", length = " << bestPath->pathLength << endl;
    }
#endif
    return choice;
} // choosePath

//------------------------------------------------------------------------------
/** A generic module to compute the difficulty to reach a point (e.g. items...).
 *  Simply calls computeFuzzyModel with the right parameters.
 *  TODO : make this comment doxygen compliant
 */
float FuzzyAIController::computeDifficultyTag(float        angle,   // degrees
                                              int          direction,
                                              float        distance)
{
//    const std::string file_name = "difficulty_tagging1.fcl";
    
    // Normalize direction so that it's between -999(%) and 999(%)
    if(direction > 999)         direction = 999;
    else if(direction < -999)   direction = -999;

//    vector<float> objectParameters;
//    objectParameters.push_back(angle);
//    objectParameters.push_back((float)direction);
//    objectParameters.push_back(distance);

    // -- Compute difficulty --
    float diff;
    float distFactor = (distance / 12.5) - 1; // how distance will affect diff
    if(angle <= 20)                         // Small angle
        diff = 0.75 + distFactor/2;         //     => very easy
    else if(angle <= 62.5)                  // Medium angle...
    {                                       // and
        if(direction <= 0)                  // target (next waypoint) side
            diff = 9 - distFactor;          //     => very difficult
        else if(direction <= 35)            // towards target
            diff = 7.5 - distFactor;        //     => difficult
        else if(direction <= 65)            // in between
            diff = 5 - distFactor*2;        //     => avg. but depends on dist.
        else if(direction <= 100)           // towards item
            diff = 2 + distFactor*2;        //     => easy but depends on dist.
        else //if(direction > 100)          // item side
            diff = 3 + distFactor*2;        //     => easy but depends on dist.
    }
    else // if(angle > 62.5)                // Big angle
        diff = 10; //9.5 + distFactor/2;
    
    return diff; // computeFuzzyModel(file_name, objectParameters);
}

//-----------------------------------------------------------------------------
void FuzzyAIController::handleBraking()
{
    // In follow the leader mode, the kart should brake if they are ahead of
    // the leader (and not the leader, i.e. don't have initial position 1)
    if(race_manager->getMinorMode() == RaceManager::MINOR_MODE_FOLLOW_LEADER &&
        m_kart->getPosition() < m_world->getKart(0)->getPosition()           &&
        m_kart->getInitialPosition()>1                                         )
    {
        m_controls->m_brake = true;
        return;
    }
        
    const float MIN_SPEED = 5.0f;
    //We may brake if we are about to get out of the road, but only if the
    //kart is on top of the road, and if we won't slow down below a certain
    //limit.
    if (m_crashes.m_road == true && m_kart->getVelocityLC().getZ() > MIN_SPEED && 
        m_world->isOnRoad(m_kart->getWorldKartId()) )
    {
        float kart_ang_diff = 
            QuadGraph::get()->getAngleToNext(m_track_node,
                                         m_successor_index[m_track_node])
          - m_kart->getHeading();
        kart_ang_diff = normalizeAngle(kart_ang_diff);
        kart_ang_diff = fabsf(kart_ang_diff);

        // FIXME: The original min_track_angle value of 20 degrees
        // resulted in way too much braking. Is this test
        // actually necessary at all???
        const float MIN_TRACK_ANGLE = DEGREE_TO_RAD*60.0f;
        const float CURVE_INSIDE_PERC = 0.25f;

        //Brake only if the road does not goes somewhat straight.
        if(m_curve_angle > MIN_TRACK_ANGLE) //Next curve is left
        {
            //Avoid braking if the kart is in the inside of the curve, but
            //if the curve angle is bigger than what the kart can steer, brake
            //even if we are in the inside, because the kart would be 'thrown'
            //out of the curve.
            if(!(m_world->getDistanceToCenterForKart(m_kart->getWorldKartId()) 
                 > QuadGraph::get()->getNode(m_track_node).getPathWidth() *
                 -CURVE_INSIDE_PERC || 
                 m_curve_angle > RAD_TO_DEGREE*m_kart->getMaxSteerAngle()) )
            {
                m_controls->m_brake = false;
                return;
            }
        }
        else if( m_curve_angle < -MIN_TRACK_ANGLE ) //Next curve is right
        {
            if(!(m_world->getDistanceToCenterForKart( m_kart->getWorldKartId() ) 
                < QuadGraph::get()->getNode(m_track_node).getPathWidth() *
                 CURVE_INSIDE_PERC ||
                 m_curve_angle < -RAD_TO_DEGREE*m_kart->getMaxSteerAngle()))
            {
                m_controls->m_brake = false;
                return;
            }
        }

        //Brake if the kart's speed is bigger than the speed we need
        //to go through the curve at the widest angle, or if the kart
        //is not going straight in relation to the road.
        if(m_kart->getVelocityLC().getZ() > m_curve_target_speed ||
           kart_ang_diff          > MIN_TRACK_ANGLE         )
        {
#ifdef AI_DEBUG
        if(m_debug)
            std::cout << "BRAKING" << std::endl;
#endif
            m_controls->m_brake = true;
            return;
        }
    }
    m_controls->m_brake = false;
}   // handleBraking


void FuzzyAIController::handleSteering(float dt)
{
    const int next = m_next_node_index[m_track_node];
    const float x = m_kart->getXYZ().getX() - m_chosenDir->x;
    const float z = m_kart->getXYZ().getZ() - m_chosenDir->z;
    
    float steer_angle = 0.0f;

    /*The AI responds based on the information we just gathered, using a
     *finite state machine.
     */
    //Reaction to being outside of the road
    if( fabsf(m_world->getDistanceToCenterForKart( m_kart->getWorldKartId() ))  >
       0.5f* QuadGraph::get()->getNode(m_track_node).getPathWidth()+0.5f )
    {
        steer_angle = steerToPoint(QuadGraph::get()->getQuadOfNode(next)
                                                    .getCenter());

#ifdef AI_DEBUG
        if(m_debug)
        {
            m_debug_sphere->setPosition(QuadGraph::get()->getQuadOfNode(next)
                       .getCenter().toIrrVector());
            std::cout << "- Outside of road: steer to center point."<<std::endl;
        }
#endif
    }
    // Avoid bad items
    else if( m_crashes.m_item != NULL )
    {
        float dist = (m_crashes.m_item->getXYZ() - m_kart->getXYZ()).length();
        float steerOffset = M_PI*0.9f;
//        float steerOffset = (dist > 7.5f)? M_PI*0.97f : M_PI*0.85f;
        if(m_start_bad_item_crash_direction == 1)
        {
            steer_angle = steerToAngle(x, z, -steerOffset);
            m_start_bad_item_crash_direction = 0;
        }
        else if(m_start_bad_item_crash_direction == -1)
        {
            steer_angle = steerToAngle(x, z, steerOffset);
            m_start_bad_item_crash_direction = 0;
        }
        else
        {
            vector2d<float> kartToItem, kartToTarget;
            float           angle;
            kartToItem = vector2d<float>(
                   m_kart->getXYZ().getX() - m_crashes.m_item->getXYZ().getX(),
                   m_kart->getXYZ().getZ() - m_crashes.m_item->getXYZ().getZ());
            kartToTarget = vector2d<float>(m_kart->getXYZ().getX() - x,
                                           m_kart->getXYZ().getZ() - z);
            // (Target, Kart, Item) angle
            angle = (float)kartToTarget.getAngleTrig() - (float)kartToItem.getAngleTrig();
            angle = (angle > 180) ? 360 - angle : angle;

            if(angle > 0) // left
            {
                steer_angle = steerToAngle(x, z, -steerOffset );
                m_start_bad_item_crash_direction = 1;
            }
            else          // right
            {
                steer_angle = steerToAngle(x, z, steerOffset );
                m_start_bad_item_crash_direction = -1;
            }
        }
            
#ifdef AI_DEBUG
        if(m_debug)
        {
            m_debug_sphere->setPosition(m_crashes.m_item->getXYZ().toIrrVector());
            cout << "- Steering to avoid bad item." << endl;
        }
#endif
    }
    //If we are going to crash against a kart, avoid it if it doesn't
    //drives the kart out of the road or on a bad item
    else if( m_crashes.m_kart != -1 && !m_crashes.m_road &&
             m_crashes.m_item == NULL )
    {
        //-1 = left, 1 = right, 0 = no crash.
        if( m_start_kart_crash_direction == 1 )
        {
            steer_angle = steerToAngle(x, z, -M_PI*0.5f);
            m_start_kart_crash_direction = 0;
        }
        else if(m_start_kart_crash_direction == -1)
        {
            steer_angle = steerToAngle(x, z, M_PI*0.5f);
            m_start_kart_crash_direction = 0;
        }
        else
        {
            if(m_world->getDistanceToCenterForKart( m_kart->getWorldKartId() ) >
               m_world->getDistanceToCenterForKart( m_crashes.m_kart ))
            {
                steer_angle = steerToAngle(x, z, -M_PI*0.5f );
                m_start_kart_crash_direction = 1;
            }
            else
            {
                steer_angle = steerToAngle(x, z, M_PI*0.5f );
                m_start_kart_crash_direction = -1;
            }
        }

#ifdef AI_DEBUG
        if(m_debug)
            std::cout << "- Velocity vector crashes with kart and doesn't " <<
            "crashes with road : steer 90 degrees away from kart." << std::endl;
#endif
    }
    else
    {
        m_start_kart_crash_direction = 0;
        float y = QuadGraph::get()->getQuadOfNode(next).getCenter().getY()+0.1;
        Vec3 straight_point(m_chosenDir->x, y, m_chosenDir->z);
#ifdef AI_DEBUG
        if(m_debug)
            m_debug_sphere->setPosition(straight_point.toIrrVector());
#endif
        steer_angle = steerToPoint(straight_point);
    }

    setSteering(steer_angle, dt);
}   // handleSteering

//-----------------------------------------------------------------------------
/** Handle all items depending on the chosen strategy: Either (low level AI)
 *  just use an item after 10 seconds, or do a much better job on higher level
 *  AI - e.g. aiming at karts ahead/behind, wait an appropriate time before 
 *  using multiple items etc.
 */
void FuzzyAIController::handleItems(const float dt)
{
    bool playerBehind = (m_kart_behind && race_manager->isPlayerKart(
                                    m_kart_behind->getWorldKartId()));
    bool playerAhead  = (m_kart_ahead && race_manager->isPlayerKart(
                                    m_kart_ahead->getWorldKartId()));
    int eval = fuzzy_data_manager->getPlayerEvaluation();
    bool beNiceWithPlayers = (eval == 3); // 3 = bad
    
    m_controls->m_fire = false;
    if(m_kart->playingEmergencyAnimation() || 
        m_kart->getPowerup()->getType() == PowerupManager::POWERUP_NOTHING ) 
        return;

    m_time_since_last_shot += dt;

    // Tactic 1: wait ten seconds, then use item
    // -----------------------------------------
    if(m_item_tactic==IT_TEN_SECONDS)
    {
        if( m_time_since_last_shot > 10.0f )
        {
            m_controls->m_fire = true;
            m_time_since_last_shot = 0.0f;
        }
        return;
    }

    // Tactic 2: calculate
    // -------------------
    switch( m_kart->getPowerup()->getType() )
    {
    case PowerupManager::POWERUP_BUBBLEGUM:
        // Avoid dropping all bubble gums one after another
        if( m_time_since_last_shot <3.0f) break;

        // Either use the bubble gum after 10 seconds, or if the next kart 
        // behind is 'close' but not too close (too close likely means that the
        // kart is not behind but more to the side of this kart and so won't 
        // be hit by the bubble gum anyway). Should we check the speed of the
        // kart as well? I.e. only drop if the kart behind is faster? Otoh 
        // this approach helps preventing an overtaken kart to overtake us 
        // again.
        m_controls->m_fire = (m_distance_behind < 15.0f &&
                              m_distance_behind > 3.0f    );
        break;   // POWERUP_BUBBLEGUM

    // All the thrown/fired items might be improved by considering the angle
    // towards m_kart_ahead.
    case PowerupManager::POWERUP_CAKE:
        {

            // Leave some time between shots
            if(m_time_since_last_shot<3.0f) break;
            // Since cakes can be fired all around, just use a sane distance
            // with a bit of extra for backwards, as enemy will go towards cake
            bool fire_backwards = (m_kart_behind && m_kart_ahead &&
                                   m_distance_behind < m_distance_ahead) ||
                                  !m_kart_ahead;
            float distance = fire_backwards ? m_distance_behind
                                            : m_distance_ahead;
                                            
            m_controls->m_fire = (fire_backwards && distance < 25.0f)  ||
                                 (!fire_backwards && distance < 20.0f);

            // "Bad players" filter : 9.5 times out of 10 don't fire at them
            if(m_controls->m_fire)
            {
                if((fire_backwards && beNiceWithPlayers && playerBehind) ||
                   (!fire_backwards && beNiceWithPlayers && playerAhead))
                    m_controls->m_fire = (((rand()*instanceCount*33)%100) < 5);

            }
            
            if(m_controls->m_fire)
                m_controls->m_look_back = fire_backwards;
            break;
        }   // POWERUP_CAKE

    case PowerupManager::POWERUP_BOWLING:
        {
            // Leave more time between bowling balls, since they are 
            // slower, so it should take longer to hit something which
            // can result in changing our target.
            if(m_time_since_last_shot < 5.0f) break;
            // Bowling balls are slower, so only fire on closer karts - but when
            // firing backwards, the kart can be further away, since the ball
            // acts a bit like a mine (and the kart is racing towards it, too)
            bool fire_backwards = (m_kart_behind && m_kart_ahead && 
                                   m_distance_behind < m_distance_ahead) ||
                                  !m_kart_ahead;
            float distance = fire_backwards ? m_distance_behind 
                                            : m_distance_ahead;
            m_controls->m_fire = ( (fire_backwards && distance < 30.0f)  ||
                                   (!fire_backwards && distance <10.0f)    ) &&
                                m_time_since_last_shot > 3.0f;
                                
            // "Bad players" filter : 9.5 times out of 10 don't fire at them
            if(m_controls->m_fire)
            {
                if((fire_backwards && beNiceWithPlayers && playerBehind) ||
                   (!fire_backwards && beNiceWithPlayers && playerAhead))
                    m_controls->m_fire = (((rand()*instanceCount*33)%100) < 5);
            }
            
            if(m_controls->m_fire)
                m_controls->m_look_back = fire_backwards;
            break;
        }   // POWERUP_BOWLING

    case PowerupManager::POWERUP_ZIPPER:
        // Do nothing. Further up a zipper is used if nitro should be selected,
        // saving the (potential more valuable nitro) for later
        break;   // POWERUP_ZIPPER

    case PowerupManager::POWERUP_PLUNGER:
        {
            // Leave more time after a plunger, since it will take some
            // time before a plunger effect becomes obvious.
            if(m_time_since_last_shot < 5.0f) break;

            // Plungers can be fired backwards and are faster,
            // so allow more distance for shooting.
            bool fire_backwards = (m_kart_behind && m_kart_ahead && 
                                   m_distance_behind < m_distance_ahead) ||
                                  !m_kart_ahead;
            float distance      = fire_backwards ? m_distance_behind 
                                                 : m_distance_ahead;
            m_controls->m_fire  = distance < 30.0f                 || 
                                  m_time_since_last_shot > 10.0f;
            if(m_controls->m_fire)
                m_controls->m_look_back = false;//fire_backwards;            // Modified kinsu (DEMO)
            
            break;
        }   // POWERUP_PLUNGER

    case PowerupManager::POWERUP_SWITCH:
        // For now don't use a switch if this kart is first (since it's more 
        // likely that this kart then gets a good iteam), otherwise use it 
        // after a waiting an appropriate time
        if(m_kart->getPosition()>1 && 
            m_time_since_last_shot > stk_config->m_item_switch_time+2.0f)
            m_controls->m_fire = true;
        break;   // POWERUP_SWITCH

    case PowerupManager::POWERUP_PARACHUTE:
        // Wait one second more than a previous parachute
        if(m_time_since_last_shot > stk_config->m_parachute_time_other+1.0f)
            m_controls->m_fire = true;
        break;   // POWERUP_PARACHUTE

    case PowerupManager::POWERUP_ANVIL:
        // Wait one second more than a previous anvil
        if(m_time_since_last_shot < stk_config->m_anvil_time+1.0f) break;

        if(race_manager->getMinorMode()==RaceManager::MINOR_MODE_FOLLOW_LEADER)
        {
            m_controls->m_fire = m_world->getTime()<1.0f && 
                                 m_kart->getPosition()>2;
        }
        else
        {
            m_controls->m_fire = m_time_since_last_shot > 3.0f && 
                                 m_kart->getPosition()>1;
        }
        break;   // POWERUP_ANVIL

    case PowerupManager::POWERUP_SWATTER:
        {
            // Squared distance for which the swatter works
            float d2 = m_kart->getKartProperties()->getSwatterDistance2();
            // Fire if the closest kart ahead or to the back is not already 
            // squashed and close enough.
            // FIXME: this can be improved on, since more than one kart might 
            //        be hit, and a kart ahead might not be at an angle at 
            //        which the glove can be used.
            if(  ( m_kart_ahead && !m_kart_ahead->isSquashed()             &&
                    (m_kart_ahead->getXYZ()-m_kart->getXYZ()).length2()<d2 &&
                    m_kart_ahead->getSpeed() < m_kart->getSpeed()            ) ||
                 ( m_kart_behind && !m_kart_behind->isSquashed() &&
                    (m_kart_behind->getXYZ()-m_kart->getXYZ()).length2()<d2) )
                    m_controls->m_fire = true;
            break;
        }
    case PowerupManager::POWERUP_RUBBERBALL:
        // Perhaps some more sophisticated algorithm might be useful.
        // For now: fire if there is a kart ahead (which means that
        // this kart is certainly not the first kart)
        m_controls->m_fire = m_kart_ahead != NULL;
        break;
    default:
        printf("Invalid or unhandled powerup '%d' in fuzzyAIController.\n",
                m_kart->getPowerup()->getType());
        assert(false);
    }
    if(m_controls->m_fire)  m_time_since_last_shot = 0.0f;
}   // handleItems

//-----------------------------------------------------------------------------
/** Determines the closest karts just behind and in front of this kart. The
 *  'closeness' is for now simply based on the position, i.e. if a kart is
 *  more than one lap behind or ahead, it is not considered to be closest.
 */
void FuzzyAIController::computeNearestKarts()
{
    bool need_to_check = false;
    int my_position    = m_kart->getPosition();
    // See if the kart ahead has changed:
    if( ( m_kart_ahead && m_kart_ahead->getPosition()+1!=my_position ) ||
        (!m_kart_ahead && my_position>1                              )    )
       need_to_check = true;
    // See if the kart behind has changed:
    if( ( m_kart_behind && m_kart_behind->getPosition()-1!=my_position   ) ||
        (!m_kart_behind && my_position<(int)m_world->getCurrentNumKarts())    )
        need_to_check = true;
    if(!need_to_check) return;

    m_kart_behind    = m_kart_ahead      = NULL;
    m_distance_ahead = m_distance_behind = 9999999.9f;
    float my_dist = m_world->getDistanceDownTrackForKart(m_kart->getWorldKartId());
    for(unsigned int i=0; i<m_world->getNumKarts(); i++)
    {
        Kart *k = m_world->getKart(i);
        if(k->isEliminated() || k->hasFinishedRace() || k==m_kart) continue;
        if(k->getPosition()==my_position+1) 
        {
            m_kart_behind = k;
            m_distance_behind = my_dist - m_world->getDistanceDownTrackForKart(i);
            if(m_distance_behind<0.0f)
                m_distance_behind += m_track->getTrackLength();
        }
        else if(k->getPosition()==my_position-1)
        {
            m_kart_ahead = k;
            m_distance_ahead = m_world->getDistanceDownTrackForKart(i) - my_dist;
            if(m_distance_ahead<0.0f)
                m_distance_ahead += m_track->getTrackLength();
        }
    }   // for i<world->getNumKarts()
}   // computeNearestKarts

//-----------------------------------------------------------------------------
void FuzzyAIController::handleAcceleration( const float dt )
{
    //Do not accelerate until we have delayed the start enough
    if( m_start_delay > 0.0f )
    {
        m_start_delay -= dt;
        m_controls->m_accel = 0.0f;
        return;
    }

    if( m_controls->m_brake == true )
    {
        m_controls->m_accel = 0.0f;
        return;
    }

    if(m_kart->hasViewBlockedByPlunger())
    {
        if(!(m_kart->getSpeed() > m_kart->getCurrentMaxSpeed() / 2))
            m_controls->m_accel = 0.05f;
        else 
            m_controls->m_accel = 0.0f;
        return;
    }
    
    // FIXME: this needs to be rewritten, it doesn't make any sense:
    // wait for players triggers the opposite (if a player is ahead
    // of this AI, go full speed). Besides, it's going to use full
    // speed anyway.
    if( m_wait_for_players )
    {
        //Find if any player is ahead of this kart
        bool player_winning = false;
        for(unsigned int i = 0; i < race_manager->getNumPlayers(); ++i )
            if(m_kart->getPosition() > m_world->getPlayerKart(i)->getPosition())
            {
                player_winning = true;
                break;
            }

        if( player_winning )
        {
            m_controls->m_accel = m_max_handicap_speed;
            return;
        }
    }

    m_controls->m_accel = stk_config->m_ai_acceleration;
}   // handleAcceleration

//-----------------------------------------------------------------------------
void FuzzyAIController::handleRaceStart()
{
    if( m_start_delay <  0.0f )
    {
        // Each kart starts at a different, random time, and the time is
        // smaller depending on the difficulty.
        m_start_delay = m_min_start_delay 
            + (float) rand() / RAND_MAX * (m_max_start_delay-m_min_start_delay);

        // Now check for a false start. If so, add 1 second penalty time.
        if(rand() < RAND_MAX * m_false_start_probability)
        {
            m_start_delay+=stk_config->m_penalty_time;
            return;
        }
    }
}   // handleRaceStart

//-----------------------------------------------------------------------------
void FuzzyAIController::handleRescue(const float dt)
{
    // check if kart is stuck
    if(m_kart->getSpeed()<2.0f && !m_kart->playingEmergencyAnimation() && 
        !m_world->isStartPhase())
    {
        m_time_since_stuck += dt;
        if(m_time_since_stuck > 2.0f)
        {
            m_kart->forceRescue();
            m_time_since_stuck=0.0f;
        }   // m_time_since_stuck > 2.0f
    }
    else
    {
        m_time_since_stuck = 0.0f;
    }
}   // handleRescue

//-----------------------------------------------------------------------------
/** Decides wether to use nitro or not.
 */
void FuzzyAIController::handleNitroAndZipper()
{
    m_controls->m_nitro = false;
    // If we are already very fast, save nitro.
    if(m_kart->getSpeed() > 0.95f*m_kart->getCurrentMaxSpeed())
        return;
    // Don't use nitro when the AI has a plunger in the face!
    if(m_kart->hasViewBlockedByPlunger()) return;
    
    // Don't use nitro if the kart doesn't have any or is not on ground.
    if(!m_kart->isOnGround() || m_kart->hasFinishedRace()) return;
    
    // Don't compute nitro usage if we don't have nitro or are not supposed
    // to use it, and we don't have a zipper or are not supposed to use
    // it (calculated).
    if( (m_kart->getEnergy()==0 || m_nitro_level==NITRO_NONE)  &&
        (m_kart->getPowerup()->getType()!=PowerupManager::POWERUP_ZIPPER ||
          m_item_tactic==IT_TEN_SECONDS                                    ) )
        return;

    // If a parachute or anvil is attached, the nitro doesn't give much
    // benefit. Better wait till later.
    const bool has_slowdown_attachment = 
        m_kart->getAttachment()->getType()==Attachment::ATTACH_PARACHUTE ||
        m_kart->getAttachment()->getType()==Attachment::ATTACH_ANVIL;
    if(has_slowdown_attachment) return;

    // If the kart is very slow (e.g. after rescue), use nitro
    if(m_kart->getSpeed()<5)
    {
        m_controls->m_nitro = true;
        return;
    }

    // If this kart is the last kart, and we have enough 
    // (i.e. more than 2) nitro, use it.
    // -------------------------------------------------
    const unsigned int num_karts = m_world->getCurrentNumKarts();
    if(m_kart->getPosition()== (int)num_karts && m_kart->getEnergy()>2.0f)
    {
        m_controls->m_nitro = true;
        return;
    }

    // On the last track shortly before the finishing line, use nitro 
    // anyway. Since the kart is faster with nitro, estimate a 50% time
    // decrease (additionally some nitro will be saved when top speed
    // is reached).
    if(m_world->getLapForKart(m_kart->getWorldKartId())==race_manager->getNumLaps()-1 &&
        m_nitro_level == NITRO_ALL)
    {
        float finish = m_world->getEstimatedFinishTime(m_kart->getWorldKartId());
        if( 1.5f*m_kart->getEnergy() >= finish - m_world->getTime() )
        {
            m_controls->m_nitro = true;
            return;
        }
    }

    // A kart within this distance is considered to be overtaking (or to be
    // overtaken).
    const float overtake_distance = 10.0f;

    // Try to overtake a kart that is close ahead, except 
    // when we are already much faster than that kart
    // --------------------------------------------------
    if(m_kart_ahead                                       && 
        m_distance_ahead < overtake_distance              &&
        m_kart_ahead->getSpeed()+5.0f > m_kart->getSpeed()   )
    {
            m_controls->m_nitro = true;
            return;
    }

    if(m_kart_behind                                   &&
        m_distance_behind < overtake_distance          &&
        m_kart_behind->getSpeed() > m_kart->getSpeed()    )
    {
        // Only prevent overtaking on highest level
        m_controls->m_nitro = m_nitro_level==NITRO_ALL;
        return;
    }
    
}   // handleNitroAndZipper

//-----------------------------------------------------------------------------
void FuzzyAIController::checkCrashes(int steps, const Vec3& pos )
{
    //Right now there are 2 kind of 'crashes': with other karts and another
    //with the track. The sight line is used to find if the karts crash with
    //each other, but the first step is twice as big as other steps to avoid
    //having karts too close in any direction. The crash with the track can
    //tell when a kart is going to get out of the track so it steers.
    m_crashes.clear();

    // If slipstream should be handled actively, trigger overtaking the
    // kart which gives us slipstream if slipstream is ready
    const SlipStream *slip=m_kart->getSlipstream();
    if(m_make_use_of_slipstream && slip->isSlipstreamReady() &&
        slip->getSlipstreamTarget())
    {
        //printf("%s overtaking %s\n", m_kart->getIdent().c_str(),
        //    m_kart->getSlipstreamKart()->getIdent().c_str());
        // FIXME: we might define a minimum distance, and if the target kart
        // is too close break first - otherwise the AI hits the kart when
        // trying to overtake it, actually speeding the other kart up.
        m_crashes.m_kart = slip->getSlipstreamTarget()->getWorldKartId();
    }
    
    const size_t NUM_KARTS = m_world->getNumKarts();

    //Protection against having vel_normal with nan values
    const Vec3 &VEL = m_kart->getVelocity();
    Vec3 vel_normal(VEL.getX(), 0.0, VEL.getZ());
    float speed=vel_normal.length();
    // If the velocity is zero, no sense in checking for crashes in time
    if(speed==0) return;

    // -- Bad items crashes --
    Item*         crashItem;
    vector<Item*> closeItems;
    float         dist, diff;
    float         smallestDist = 99999.f;
    unsigned int  closestId    = 99999;
    
    // Get (very) close items
    item_manager->getCloseItems(closeItems, m_kart, 7.5f);
    // Get the closest bad item
    for(unsigned int i=0; i<closeItems.size(); i++)
    {
        if(closeItems[i]->getType() == Item::ITEM_BANANA ||
           closeItems[i]->getType() == Item::ITEM_BUBBLEGUM )
        {
            // -- Method 1 : look at the closest item --
            dist = (closeItems[i]->getXYZ() - m_kart->getXYZ()).length();
            if(dist < smallestDist)
            {
                smallestDist = dist;
                closestId = i;
            } // if it's closest than the currently known closest
            
            // -- Method 2 : look at the easiest to reach item --
//            diff = estimateDifficultyToReach(closeItems[i]->getXYZ());
//            if(diff < smallestDiff)
//            {
//                smallestDiff = diff;
//                easiestId = i;
//            }
            
        } // if this is a bad item
    } // for each close item
    if(closestId != 99999)
    {
        crashItem = closeItems[closestId];
        diff = estimateDifficultyToReach(crashItem->getXYZ());
        if(/*diff >= 0 && */diff < 1.5f)       // crash
            m_crashes.m_item = crashItem;
        else                            // no crash
            m_crashes.m_item = NULL;
    } // if there are bad items around
    else
        m_crashes.m_item = NULL;
    
    // -- Kart & road crashes --
    // Time it takes to drive for m_kart_length units.
    float dt = m_kart_length / speed; 
    vel_normal/=speed;

    int current_node = m_track_node;
    if(steps<1 || steps>1000)
    {
        printf("Warning, incorrect STEPS=%d. kart_length %f velocity %f\n",
            steps, m_kart_length, m_kart->getVelocityLC().getZ());
        steps=1000;
    }
    for(int i = 1; steps > i; ++i)
    {
        Vec3 step_coord = pos + vel_normal* m_kart_length * float(i);

        /* Find if we crash with any kart, as long as we haven't found one yet
         */
        if( m_crashes.m_kart == -1 )
        {
            for( unsigned int j = 0; j < NUM_KARTS; ++j )
            {
                const Kart* kart = m_world->getKart(j);
                if(kart==m_kart||kart->isEliminated()) continue;   // ignore eliminated karts
                const Kart *other_kart = m_world->getKart(j);
                // Ignore karts ahead that are faster than this kart.
                if(m_kart->getVelocityLC().getZ() < other_kart->getVelocityLC().getZ())
                    continue;
                Vec3 other_kart_xyz = other_kart->getXYZ() + other_kart->getVelocity()*(i*dt);
                float kart_distance = (step_coord - other_kart_xyz).length_2d();

                if( kart_distance < m_kart_length)
                    m_crashes.m_kart = j;
            }
        }

        /*Find if we crash with the drivelines*/
        if(current_node!=QuadGraph::UNKNOWN_SECTOR &&
            m_next_node_index[current_node]!=-1)
            QuadGraph::get()->findRoadSector(step_coord, &current_node,
                        /* sectors to test*/ &m_all_look_aheads[current_node]);

        if( current_node == QuadGraph::UNKNOWN_SECTOR)
        {
            m_crashes.m_road = true;
            return;
        }
    }
}   // checkCrashes

//-----------------------------------------------------------------------------
/** Find the sector that at the longest distance from the kart, that can be
 *  driven to without crashing with the track, then find towards which of
 *  the two edges of the track is closest to the next curve after wards,
 *  and return the position of that edge.
 */
void FuzzyAIController::findNonCrashingPoint(Vec3 *result)
{    
    unsigned int sector = m_next_node_index[m_track_node];
    int target_sector;

    Vec3 direction;
    Vec3 step_track_coord;

    // The original while(1) loop is replaced with a for loop to avoid
    // infinite loops (which we had once or twice). Usually the number
    // of iterations in the while loop is less than 7.
    for(unsigned int i=0; i<100; i++)
    {
        //target_sector is the sector at the longest distance that we can drive
        //to without crashing with the track.
        target_sector = m_next_node_index[sector];

        //direction is a vector from our kart to the sectors we are testing
        direction = QuadGraph::get()->getQuadOfNode(target_sector).getCenter()
                  - m_kart->getXYZ();

        float len=direction.length_2d();
        unsigned int steps = (unsigned int)( len / m_kart_length );
        if( steps < 3 ) steps = 3;

        // That shouldn't happen, but since we had one instance of
        // STK hanging, add an upper limit here (usually it's at most
        // 20 steps)
        if( steps>1000) steps = 1000;

        //Protection against having vel_normal with nan values
        if(len>0.0f) {
            direction*= 1.0f/len;
        }

        Vec3 step_coord;
        //Test if we crash if we drive towards the target sector
        for(unsigned int i = 2; i < steps; ++i )
        {
            step_coord = m_kart->getXYZ()+direction*m_kart_length * float(i);

            QuadGraph::get()->spatialToTrack(&step_track_coord, step_coord,
                                             sector );
 
            float distance = fabsf(step_track_coord[0]);

            //If we are outside, the previous sector is what we are looking for
            if ( distance + m_kart_width * 0.5f 
                 > QuadGraph::get()->getNode(sector).getPathWidth() )
            {
                *result = QuadGraph::get()->getQuadOfNode(sector).getCenter();
                return;
            }
        }
        sector = target_sector;
    }   // for i<100
    *result = QuadGraph::get()->getQuadOfNode(sector).getCenter();
}   // findNonCrashingPoint

//-----------------------------------------------------------------------------
/** calc_steps() divides the velocity vector by the lenght of the kart,
 *  and gets the number of steps to use for the sight line of the kart.
 *  The calling sequence guarantees that m_future_sector is not UNKNOWN.
 */
int FuzzyAIController::calcSteps()
{
    int steps = int( m_kart->getVelocityLC().getZ() / m_kart_length );
    if( steps < m_min_steps ) steps = m_min_steps;

    //Increase the steps depending on the width, if we steering hard,
    //mostly for curves.
#if 0
    // FIXME: I don't understand this: if we are steering hard, we check
    //        for more steps if we hit another kart?? If we steer hard,
    //        the approximation used (pos + velocity*dt) will be even
    //        worse, since it doesn't take steering into account.
    if( fabsf(m_controls->m_steer) > 0.95 )
    {
        const int WIDTH_STEPS = 
            (int)( QuadGraph::get()->getNode(m_future_sector).getPathWidth()
                   /( m_kart_length * 2.0 ) );

        steps += WIDTH_STEPS;
    }
#endif
    // The AI is driving significantly better with more steps, so for now
    // add 5 additional steps.
    return steps+5;
}   // calcSteps

//-----------------------------------------------------------------------------
/**FindCurve() gathers info about the closest sectors ahead: the curve
 * angle, the direction of the next turn, and the optimal speed at which the
 * curve can be travelled at it's widest angle.
 *
 * The number of sectors that form the curve is dependant on the kart's speed.
 */
void FuzzyAIController::findCurve()
{
    float total_dist = 0.0f;
    int i;
    for(i = m_track_node; total_dist < m_kart->getVelocityLC().getZ(); 
        i = m_next_node_index[i])
    {
        total_dist += QuadGraph::get()->getDistanceToNext(i, 
                                                         m_successor_index[i]);
    }

    m_curve_angle = 
        normalizeAngle(QuadGraph::get()->getAngleToNext(i, 
                                                        m_successor_index[i])
                      -QuadGraph::get()->getAngleToNext(m_track_node, 
                                            m_successor_index[m_track_node]) );
    
    m_curve_target_speed = m_kart->getCurrentMaxSpeed();
}   // findCurve

//------------------------------------------------------------------------------
/** Difficulty computation : computes the needed parameters and calls the fuzzy
 *  logic model (computeDifficultyTag() function) to estimate the difficulty to
 *  reach the given point.
 */
float FuzzyAIController::estimateDifficultyToReach(const Vec3& point)
{
    vector2d<float> kartToPoint, kartToNextNode, kartVel;
    int             direction, angleSign;
    float           dist, x, z, vel, angle, diff;
    float           kartX = m_kart->getXYZ().getX();
    float           kartZ = m_kart->getXYZ().getZ();
    
//    const int next = m_next_node_index[m_track_node]; TODODO

    // Get the distance between the given point and the kart
    dist = (m_kart->getXYZ() - point).length();
    
    // Kart to point vector
    kartToPoint = vector2d<float>(point.getX()-kartX, point.getZ()-kartZ);
    
    // Kart to next node vector --  method 1
//    kartToNextNode = vector2d<float>(m_mainAPt.x-kartX, m_mainAPt.z-kartZ);
    //                              method 2
    core::vector3df nextNode = QuadGraph::get()->getQuadOfNode(m_next_node_index[m_next_node_index[m_track_node]]).getCenter().toIrrVector();
    kartToNextNode = vector2d<float>(nextNode.X-kartX, nextNode.Z-kartZ);
    
    // (KartToNextNode, kartToPoint) angle
    angle = (float)kartToNextNode.getAngleTrig() - (float)kartToPoint.getAngleTrig();
    angle = (angle > 180) ? 360 - angle : angle;
    
    // Kart direction (in % of the above angle)        
    x = m_kart->getVelocity().getX();
    z = m_kart->getVelocity().getZ();
    kartVel = vector2d<float>(x, z);
    vel = (float)kartToNextNode.getAngleTrig() - (float)kartVel.getAngleTrig();
    vel = (vel > 180) ? 360 - vel : vel;
    direction = (int)(100.0f * vel / angle);
    
    // Once the relative direction is known, only keep the angle's magnitude
    if(angle < 0)
    {
        angle = -angle;
        angleSign = -1;
    }
    else
        angleSign = 1;

    diff = computeDifficultyTag(angle, direction, dist);
        
    // Finally, call the fuzzy model to estimate the difficulty
    return diff;
}

//------------------------------------------------------------------------------
/** Wrapper function for computing attraction for all items. In this function,
 *  the computation method for the different kinds of items can be modified.
 */
float FuzzyAIController::computeItemAttraction(const Item* item)
{
    float attraction;

    // Simplified version for every item : attraction just depends on the
    // estimated difficulty to reach the item.
    attraction = 10 - estimateDifficultyToReach(item->getXYZ());

//    float diffTag;
//    if(item->getType() == Item::ITEM_BONUS_BOX)
//    {
//
//        bool hasPowerup = (m_kart->getPowerup()->getType() !=
//                                               PowerupManager::POWERUP_NOTHING);
//        // Box attraction
//        attraction = computeBoxAttraction(diffTag, hasPowerup);    
//    }
//
//   if(item->getType() == Item::ITEM_NITRO_BIG   ||
//      item->getType() == Item::ITEM_NITRO_SMALL)
//   {
//       float energy = m_kart->getEnergy();
//
//       bool hasPowerup = (m_kart->getPowerup()->getType() !=
//                                               PowerupManager::POWERUP_NOTHING);
//       //Nitro attraction
//       attraction = computeBoxAttraction(diffTag, hasPowerup);
//       // TODO use computeNitroAttraction
//
//   }
    
    return attraction;
}

//------------------------------------------------------------------------------
/** Computes an attraction value for every item in the parameter vector. The
 *  estimation of the difficulty to reach the item is first computed, then this
 *  value is used to compute the output attraction value. Note that this is a
 *  simplified model of attraction value computation, which should also compute
 *  and take in account an interest value for the item (see Fuzzy AI specs).
 *  TODO make this comment doxygen compliant
 */
void FuzzyAIController::tagItems(const vector<Item*>& items,
                                       vector<AttrPoint*>& attrPts )
{
    vector<std::string*> texts;
    bool                 known;
    float                attraction;

    // Every existing attraction point must be updated
    for(unsigned int i=0 ; i < attrPts.size() ; i++)
        attrPts[i]->updated = false;
    
    for(unsigned int i=0 ; i < items.size() ; i++)
    {
        // Don't pay attention to bad items
        if(items[i]->getType() == Item::ITEM_BANANA ||
           items[i]->getType() == Item::ITEM_BUBBLEGUM)
            continue;
        
        known = false;
        // Check if the item is already known
        for(unsigned int j=0 ; j < attrPts.size() ; j++)
        {
            // If the item is already known, update the attraction point
            if(attrPts[j]->object != NULL && attrPts[j]->object == items[i])
            {   
                attraction = computeItemAttraction(items[i]);
                attrPts[j]->attraction = attraction;
                attrPts[j]->updated = true;
                known = true;
                break;
            } // if the item is already known
        } // for each known attraction point
        if(!known) // If the item is not known, create a new attraction point
        {
            // Compute item attraction
            attraction = computeItemAttraction(items[i]);
            
            float x = items[i]->getXYZ().getX();
            float z = items[i]->getXYZ().getZ();
            AttrPoint* attrPt = new AttrPoint(x, z);
            attrPt->attraction = attraction;
            attrPt->object = ((FuzzyAITaggable*) items[i]);
            attrPt->updated = true;
            attrPts.push_back(attrPt);
        } // if !known
        
#ifdef AI_DEBUG
        if(m_debug)
        {
            std::stringstream * t = new std::stringstream();
            attraction = (attraction*100 + (attraction<0? -0.5f : 0.5f))/100.0f; // round
            (*t) << "A = " << attraction << endl;
            ((FuzzyAITaggable*) items[i])->setDebugText(t->str());
            cout << t->str();
        } // if debug
#endif
    } // for every detected item
    
    for(int i=attrPts.size()-1 ; i>=0  ; i--)
    {
        if(attrPts[i]->object != NULL && attrPts[i]->updated == false)
        {
            AttrPoint* toDelete = attrPts[i];
            attrPts.erase(attrPts.begin() + i);
            delete toDelete;
        } // if the item from the attraction point is not detected anymore
    } // for each known attraction point
} // tagItems

//------------------------------------------------------------------------------
/** Detects path forks in the look_ahead nodes, and launches a path choice
 *  computation when needed.
 *  Important note : the m_fork_choices vector contains the choices for the
 *  encountered forks in the look_ahead nodes. Its first element MUST be removed
 *  each time the kart passes a fork node, so it only holds the upcoming forks.
 
 * TODO Doxygen compliant comment
 */
void FuzzyAIController::computeForkChoices(vector<unsigned int>& output)
{
    vector<unsigned int> nextNodes;
    unsigned int curNode = m_track_node;
    unsigned int forkId = 0;
    unsigned int nextId = 0;
    for(unsigned int i=0 ; i < m_look_ahead ; i++)
    {
        nextNodes.clear();
        QuadGraph::get()->getSuccessors(curNode, nextNodes, true);

        if(nextNodes.size() > 1)
        {
            if(forkId+1 > m_fork_choices.size())
            {
                const vector<vector<PathData*>*>* pathData = NULL;
                pathData = fuzzy_data_manager->getPathData(curNode);
#ifdef AI_DEBUG // If the FuzzyAIPathTree is OK, data should exist for this fork
                assert(pathData);
#endif
                unsigned int pathChoice = choosePath(pathData, (float)m_compet);
                output.push_back(pathChoice);
                nextId = pathChoice;
                forkId++;
            } // if this fork's choice is not already in the fork choices vector
        } // if there is a fork in the look_ahead nodes
        else
            nextId = 0;
        
        curNode = nextNodes[nextId];
    } // for the next look ahead nodes
} // computeForkChoices

//------------------------------------------------------------------------------
/** Direction chooser :
 *  Choose the attraction point with the highest attraction value.
 *  Also take in account attraction points with negative attraction value
 *  (ie. dangerous places).
 */
AttrPoint* FuzzyAIController::chooseDirection(vector<AttrPoint*> &attrPts)
{
    unsigned int bestId = 0;
    float bestVal = 0.f;
    for(unsigned int i=0 ; i<attrPts.size() ; i++)
    {
        if(attrPts[i]->attraction > bestVal)
        {
            bestVal = attrPts[i]->attraction;
            bestId = i;
        }
    }
    
    bool stillHere;
    for(unsigned int i=0 ; i<attrPts.size() ; i++)
    {
        if(attrPts[i] == m_chosenDir)
        {
            stillHere = true;
            break;
        }
    }
    
    // This test stop the AI to change its mind too often when there are
    // multiple objects
    if(stillHere && m_chosenDir && bestVal < m_chosenDir->attraction + 0.7f)
        return m_chosenDir;
    else
        return attrPts[bestId];
} // chooseDirection


//------------------------------------------------------------------------------
/** TODO Comment
 */
//float FuzzyAIController::computeBoxAttraction(float difficultyTag,
//                                              bool  hasWeapon)
//{
//    const std::string file_name = "box_attraction.fcl";
//    
//    vector<float> params;
//    params.push_back(difficultyTag);
//    params.push_back(hasWeapon);
//    
//    return computeFuzzyModel(file_name, params);
//}

//------------------------------------------------------------------------------
/** TODO Comment
 */
//float FuzzyAIController::computeNitroAttraction    (float        difficultyTag,
//                                   float         availableNitro)
//{
//    const std::string file_name = "nitro_attraction.fcl";
//    
//    vector<float> objectParameters;
//    objectParameters.push_back(difficultyTag);
//    objectParameters.push_back(availableNitro);
//    
//    return computeFuzzyModel(file_name, objectParameters);
//
//}

//------------------------------------------------------------------------------
/** TODO Comment
 */
//float FuzzyAIController::computeBananaAttraction(float difficultyTag,
//                                                 float speed)
//{
//    const std::string file_name = "banana_gum_attraction.fcl";
//    
//    // Normalize speed
//    speed = (speed > 100)? 99 : speed;
//    speed = (speed < 0  )?  1 : speed;
//    
//    vector<float> params;
//    params.push_back(difficultyTag);
//    params.push_back(speed);
//    
//    // return negative value as in this case this is a repulsion value
//    return (-1)*(computeFuzzyModel(file_name, params));
//}

//------------------------------------------------------------------------------
/** Module to compute a difficulty prediction to hit an opponent with a weapon.
 *  Simply call computeFuzzyModel with the right parameters.
 *  TODO : make this comment doxygen compliant
 *  Fuzzy model for each weapon difficulty prediction ?
 */
//float  FuzzyAIController::computeHitEstimation(int     possessed_item_type,
//                                               float   next_kart_distance)
//{
//    const std::string file_name = "weapon_hit_estimation.fcl";
//
//    float normalized_distance;
//
//    //Check if the distance is too important for fuzzyfication.
//    if (next_kart_distance > 30)
//        normalized_distance = 30;
//    else
//        normalized_distance = next_kart_distance;
//
//    int type;
//    switch(possessed_item_type)
//    {
//        // TODO : use constant names like "ITEM_BANANA" instead of hard-coded values. (see items.hpp)
//
//        // Weapons that do not a close opponent to fire.
//        case 1 : //POWERUP_BUBBLEGUM
//        case 4 : //POWERUP_ZIPPER
//        case 6 : //POWERUP_SWITCH
//        case 9 : //POWERUP_PARACHUTE
//        case 10 : //POWERUP_ANVIL (Rubber ball)
//        case 7 : //POWERUP_SWATTER
//            type = 1;
//            break;
//
//        // Weapon that need a short distance for better result.
//        case 2 : //POWERUP_CAKE
//        case 3 : //POWERUP_BOWLING
//        case 5 : //POWERUP_PLUNGER
//        case 8 : //POWERUP_RUBBERBALL
//            type = 2;
//            break;
//
//        default :
//            type = 0;
//    }
//
//    vector<float> HitEstimation;
//    HitEstimation.push_back(type);
//    HitEstimation.push_back(normalized_distance);
//
//    return computeFuzzyModel(file_name, HitEstimation);
//}

//------------------------------------------------------------------------------
/** Module to know if it is interesting to use the possessed weapon. Simply call computeFuzzyModel with the
 *  right parameters.
 *  TODO : make this comment doxygen compliant
 *  Fuzzy model for each weapon?
 */
//float FuzzyAIController::computeWeaponInterest(int   competitiveness,
//                                               float hit_estimation )
//{
//    const std::string& file_name = "weapon_interest.fcl";
//    
//    vector<float> interestParameters;
//    interestParameters.push_back(competitiveness);
//    interestParameters.push_back(hit_estimation);
//
//    return  computeFuzzyModel(file_name, interestParameters);
//} // computeWeaponInterest
