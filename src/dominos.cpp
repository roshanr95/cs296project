/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

//! These are user defined include files
//! Included in double quotes - the path to find these has to be given at compile time
#include <iostream>
#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"
#include "callbacks.hpp"

namespace cs296
{
  /**  The is the constructor \n
   * This is the documentation block for the constructor.
   */ 
  
  
	
  dominos_t::dominos_t()
  {
   
    	
	{	      /// \par GROUND
      /*! Variable - b1 \n
       *  is a pointer to the base ground
       *  \n Data type - b2Body*
       */
    b2Body* b1;
      
      /*! Variable - shape
       *  \n \brief defines a shape that can be assigned to a body
       *  \n Data type - b2EdgeShape
       */
      
    {
      /*! b2EdgeShape shape is assigned to the b1 pointer
       */
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
    
   }

   
   {
      /// \par SHELF
      /*! Creates a shelf \n
       *  b2EdgeShape shape is modified and assigned to ground
       */
      b2PolygonShape shape;
      shape.SetAsBox(0.5f, 15.f);
	
      b2BodyDef bd;
      bd.position.Set(7.5f, 15.0f);
        /*! Variable -  ground
         *  \n \brief A ground line , size=12.0*0.5
         *  \n Data type - b2EdgeShape
         */
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
   
      
    {
		
      /// \par SHELF
      /*! Creates a horizontal shelf
       */
      b2PolygonShape shape;
      shape.SetAsBox(0.5f, 15.f);//, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(-7.5f, 15.0f);
        /*! Variable -  ground
         *  \n \brief A ground line for the horizontal shelf , size=14.0*0.5
         *  \n Data type - b2EdgeShape
         */
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    
   // {
       
      //The light box on the right side of the see-saw
        /*! Variable -  shape2
         * \n \brief The shape of the tlight box on the right side of the see-saw
         *  \n Data type - b2PolygonShape
         */
      /*b2PolygonShape shape2;
      shape2.SetAsBox(7.0f, 7.0f);
      b2BodyDef bd3;
      bd3.position.Set(0.0f, 15.0f);
      bd3.type = b2_dynamicBody;
      bd3.linearVelocity.Set(0.0f,0.0f);*/
        /*! Variable -  body3
         *  \n \brief The The light box on the right side of the see-saw , size=4.0*4.0
         *  \n Data type -  b2Body*
      
         */
    
//      b2Body* body3 = m_world->CreateBody(&bd3);
    
      
  
        /*! Variable -  fd3;
         *  \n \brief The body properties of the light block
         *  \n Data type - b2FixtureDef
         *  \n Values - density = 0.01f
         */
   /*   b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.05f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);*/
   
    
  /* for(int i=0;i<20;i++)
    body3->ApplyForceToCenter( b2Vec2(0.0f,1000.0f),  true );*/


    //}
     
     
   {  
        /// \par 1. PULLEY
        /*! Creates the second, new pulley system \n
         * The box on the see-saw is caught by a bar on one end of the pulley \n
         * The bar on the other end catces a revolving platform
         */
          /*! Variable -  bd
           *  \n \brief A body definition of a pulley wheel
           *  \n Data type -  b2FixtureDef*
           *  \n Values - fixed rotation
           */
          b2BodyDef *bd = new b2BodyDef;
          bd->type = b2_dynamicBody;
          bd->position.Set(0,30);
          bd->fixedRotation = true;
          
          /*! Variable -  fd1
           *  \n \brief The bar that is attached to the pulleys , size=2*0.2
           *  \n Data type -  b2FixtureDef*
           *  \n Values - density = 20f, friction = 0.5f, restitution = 0f
           */
          b2FixtureDef *fd1 = new b2FixtureDef;
          fd1->density = 20.0;
          fd1->friction = 0.5;
          fd1->restitution = 0.f;
          fd1->shape = new b2PolygonShape;
       
       
          /*! Variable -  bs1
           *  \n \brief The shape and position of the bar 
           *  \n Data type -  b2PolygonShape
           */
          b2PolygonShape bs1;
          bs1.SetAsBox(7,7, b2Vec2(0.f,-1.9f), 0);
          fd1->shape = &bs1;
          
          //The bar1
          bd->position.Set(0,25);
          fd1->density = 10.0;
       
          /*! Variable -  box1
           * \n \brief The bar at one end of the system , size=4*0.4
           * \n Data type -  b2Body*
           * \n Values - side fd1
           */
          b2Body* box1 = m_world->CreateBody(bd);
          box1->CreateFixture(fd1);
       
       
          /*! Variable -  bs2
           *  \n \brief The shape and position of the bar
           *  \n Data type -  b2PolygonShape
           */
          b2PolygonShape bs2;
          bs2.SetAsBox(3.5,7, b2Vec2(0.f,-1.9f), 0);
          fd1->shape = &bs2;
          
          //The bar2
          bd->position.Set(-30,10);
          fd1->density = 20.0;
       
          /*! Variable -  box2
           * \n \brief The bar at the other end of the system , siz=4*0.4
           * \n Data type -  b2Body*
           * \n Values - side fd1
           */
          b2Body* box2 = m_world->CreateBody(bd);
          box2->CreateFixture(fd1);
       
       
          /*! Variable -  bs3
           *  \n \brief The shape and position of the bar
           *  \n Data type -  b2PolygonShape
           */
          b2PolygonShape bs3;
          bs3.SetAsBox(0.5,4, b2Vec2(0.f,-1.9f), 0);
          fd1->shape = &bs3;
          
          //The bar3
          bd->position.Set(5,34);
          bd->fixedRotation = false;
          fd1->density = 20.0;
          /*! Variable -  box3
           * \n \brief The bar at the other end of the system , siz=4*0.4
           * \n Data type -  b2Body*
           * \n Values - side fd1
           */
          b2Body* box3 = m_world->CreateBody(bd);
          box3->CreateFixture(fd1);
      

          /*! Variable -  bs4
           *  \n \brief The shape and position of the bar
           *  \n Data type -  b2PolygonShape
           */
          b2PolygonShape bs3;
          bs3.SetAsBox(0.5,4, b2Vec2(0.f,-1.9f), 0);
          fd1->shape = &bs3;
          
          //The bar4
          bd->position.Set(-5,34);
          bd->fixedRotation = false;
          fd1->density = 20.0;
          /*! Variable -  box4
           * \n \brief The bar at the other end of the system , siz=4*0.4
           * \n Data type -  b2Body*
           * \n Values - side fd1
           */
          b2Body* box4 = m_world->CreateBody(bd);
          box3->CreateFixture(fd1);
       
       
          b2RevoluteJointDef *revoluteJointDef = new b2RevoluteJointDef;
          revoluteJointDef->bodyA = box1;
          revoluteJointDef->bodyB = box3;
          revoluteJointDef->collideConnected = false;
          b2Vec2 worldAnchorOnBody3(5, 7);
          revoluteJointDef->localAnchorA.Set(5,7);
          revoluteJointDef->localAnchorB.Set(0,-4);
          revoluteJointDef->referenceAngle = 0;
          //revoluteJointDef->enableLimit = true;
          //revoluteJointDef->lowerAngle = -0.8 ;
   		  //revoluteJointDef->upperAngle =  0.8 ;
          revoluteJointDef->Initialize(box1, box3, box1->GetWorldCenter()+worldAnchorOnBody3);    
          m_world->CreateJoint(revoluteJointDef);

          b2RevoluteJointDef *revoluteJointDef2 = new b2RevoluteJointDef;
          revoluteJointDef2->bodyA = box1;
          revoluteJointDef2->bodyB = box4;
          revoluteJointDef2->collideConnected = false;
          b2Vec2 worldAnchorOnBody4(-5, 7);
          revoluteJointDef2->localAnchorA.Set(-5,7);
          revoluteJointDef2->localAnchorB.Set(0,-4);
          revoluteJointDef2->referenceAngle = 0;
          //revoluteJointDef->enableLimit = true;
          //revoluteJointDef->lowerAngle = -0.8 ;
   		  //revoluteJointDef->upperAngle =  0.8 ;
          revoluteJointDef2->Initialize(box1, box, box1->GetWorldCenter()+worldAnchorOnBody4);    
          m_world->CreateJoint(revoluteJointDef2);

          // The pulley joint
          /*! Variable -  myjoint
           * \n \brief The pulley joint with two anchors
           * \n Data type -   b2PulleyJointDef*
           * \n Values - anchors = twoanchors on bodies worldAnchorOnBody1 and worldAnchorOnBody2, \n
           *                      two anchors on ground worldAnchorGround1 and worldAnchorGround2, ratio
           */
          b2PulleyJointDef* myjoint = new b2PulleyJointDef();
          b2Vec2 worldAnchorOnBody1(0, 4); // Anchor point on body 1 in world axis
          b2Vec2 worldAnchorOnBody2(-30, 40); // Anchor point on body 2 in world axis
          b2Vec2 worldAnchorGround1(0, 40); // Anchor point for ground 1 in world axis
          b2Vec2 worldAnchorGround2(-30, 40); // Anchor point for ground 2 in world axis
          float32 ratio = 1.0f; // Define ratio
          /*! The pulley joint myjoint is initialised with all the input values - box1, box2, ratio, anchors
           */
          myjoint->Initialize(box3, box2, worldAnchorGround1, worldAnchorGround2, box3->GetWorldCenter()+worldAnchorOnBody1, box2->GetWorldCenter(), ratio);
          m_world->CreateJoint(myjoint);

          b2PulleyJointDef* myjoint2 = new b2PulleyJointDef();
          /*! The pulley joint myjoint is initialised with all the input values - box1, box2, ratio, anchors
           */
          myjoint2->Initialize(box4, box2, worldAnchorGround1, worldAnchorGround2, box4->GetWorldCenter()+worldAnchorOnBody1, box2->GetWorldCenter(), ratio);
          m_world->CreateJoint(myjoint2);
          
         for(int i=0;i<100;i++)
         box2->ApplyForceToCenter( b2Vec2(0.0f,400.0f),  true );
   
   }
      
   
   }     
   
   
  ///   The dominos _t class is attached the name dominos by passing the base_sim_t* class dominos_t using the dominos::create function to the constructor of sim_t
  sim_t *sim = new sim_t("Dominos", dominos_t::create);

}
