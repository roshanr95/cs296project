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

    b2Body* box1;
    b2Body* box2;
    b2Body* doorRect1;
    b2WeldJointDef* temp = new b2WeldJointDef;
    b2PrismaticJoint* prismaticJoint;
    b2RevoluteJoint* circleToWorldJoint;
     /**  The is the constructor \n
     * This is the documentation block for the constructor.
     */ 
     dominos_t::dominos_t()
     {


         {	  

        /// \par GROUND
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
             fd1->friction = 0.0f;
             fd1->restitution = 0.f;
             fd1->shape = new b2PolygonShape;

            /*! Variable -  bs1
             *  \n \brief The shape and position of the bar 
             *  \n Data type -  b2PolygonShape
             */
             b2PolygonShape bs1;
             bs1.SetAsBox(7,5, b2Vec2(0.f,0.f), 0);
             fd1->shape = &bs1;

            //The bar1
             bd->position.Set(0,25);
             fd1->density =  0.1;
             fd1->filter.groupIndex = -1;

            /*! Variable -  box1
             * \n \brief The bar at one end of the system , size=4*0.4
             * \n Data type -  b2Body*
             * \n Values - side fd1
             */
             box1 = m_world->CreateBody(bd);
             box1->CreateFixture(fd1);
             fd1->filter.groupIndex = 0;


             {
               b2BodyDef centerCircleDef;
               centerCircleDef.type = b2_dynamicBody;
               centerCircleDef.position.Set(0,32);
               b2Body* centreCircle = m_world->CreateBody(&centerCircleDef);

               b2CircleShape circleShape;
               circleShape.m_p.Set(0, 0); 
               circleShape.m_radius = 1.5f; 

               b2FixtureDef centerCircleFixtureDef;
               centerCircleFixtureDef.shape = &circleShape;
               centerCircleFixtureDef.density = 1.0f;
               centerCircleFixtureDef.filter.groupIndex = -1;
               centreCircle->CreateFixture(&centerCircleFixtureDef);

               b2BodyDef rectBodyDef1;
               rectBodyDef1.type = b2_dynamicBody;
               rectBodyDef1.position.Set(1,31);
               doorRect1 = m_world->CreateBody(&rectBodyDef1);

               b2PolygonShape rectShape1;
               rectShape1.SetAsBox(0.2,1,b2Vec2(0,0), 0);

               b2FixtureDef rectFixtureDef;
               rectFixtureDef.shape = &rectShape1;
               rectFixtureDef.density = 0.1f;
               rectFixtureDef.filter.groupIndex = -1;
               doorRect1->CreateFixture(&rectFixtureDef);

               b2RevoluteJointDef rodJointDef1;
               rodJointDef1.bodyA = centreCircle;
               rodJointDef1.bodyB = doorRect1;
               rodJointDef1.collideConnected = false;

               rodJointDef1.localAnchorA.Set(1,0);
               rodJointDef1.localAnchorB.Set(0,1);
               m_world->CreateJoint(&rodJointDef1);

               b2RevoluteJointDef circleToWorldJointDef;
               circleToWorldJointDef.bodyA = centreCircle;
               circleToWorldJointDef.bodyB = box1;
               circleToWorldJointDef.collideConnected = false;
               circleToWorldJointDef.enableMotor = true;
               circleToWorldJointDef.maxMotorTorque = 1000;
               circleToWorldJointDef.motorSpeed = 0;

               circleToWorldJointDef.localAnchorA.Set(0,0);
               circleToWorldJointDef.localAnchorB.Set(0,7);
               circleToWorldJoint = (b2RevoluteJoint *)m_world->CreateJoint(&circleToWorldJointDef);

               {
        // b2BodyDef rectBodyDef3;
        // rectBodyDef3.type = b2_dynamicBody;
        // rectBodyDef3.position.Set(-2.5,25.5);
        // b2Body* doorRect3 = m_world->CreateBody(&rectBodyDef3);

                b2PolygonShape rectShape3;
                rectShape3.SetAsBox(2.5,0.2,b2Vec2(-2.5,-1), 0);

                b2FixtureDef rectFixtureDef3;
                rectFixtureDef3.shape = &rectShape3;
                rectFixtureDef3.filter.groupIndex = -1;
                doorRect1->CreateFixture(&rectFixtureDef3);

        // b2WeldJointDef rodJointDef3;
        // rodJointDef3.bodyA = doorRect1;
        // rodJointDef3.bodyB = doorRect3;
        // rodJointDef3.collideConnected = false;

        // rodJointDef3.localAnchorA.Set(0,-2.5);
        // rodJointDef3.localAnchorB.Set(5,0);
        // m_world->CreateJoint(&rodJointDef3);
            }

            {
                b2BodyDef doorMainRodDef;
                doorMainRodDef.type = b2_dynamicBody;
                doorMainRodDef.position.Set(-4,29.5);
                b2Body *doorMainRod = m_world->CreateBody(&doorMainRodDef);

                b2PolygonShape doorMainRodShape;
                doorMainRodShape.SetAsBox(0.3,2.5,b2Vec2(0,0),0);

                b2FixtureDef doorMainRodFixtureDef;
                doorMainRodFixtureDef.shape = &doorMainRodShape;
                doorMainRodFixtureDef.density = 0.1f;
                doorMainRodFixtureDef.filter.groupIndex = -1;
                doorMainRod->CreateFixture(&doorMainRodFixtureDef);

                b2RevoluteJointDef rodToWorldJointDef;
                rodToWorldJointDef.bodyA = box1;
                rodToWorldJointDef.bodyB = doorMainRod;
                rodToWorldJointDef.collideConnected = false;

                rodToWorldJointDef.localAnchorA.Set(-4,7);
                rodToWorldJointDef.localAnchorB.Set(0,2.5);
                m_world->CreateJoint(&rodToWorldJointDef);

                b2RevoluteJointDef rodToRodJointDef;
                rodToRodJointDef.bodyA = doorRect1;
                rodToRodJointDef.bodyB = doorMainRod;
                rodToRodJointDef.collideConnected = false;

                rodToRodJointDef.localAnchorA.Set(-3.5,-1);
                rodToRodJointDef.localAnchorB.Set(0,0);
                m_world->CreateJoint(&rodToRodJointDef);


                b2BodyDef doorBodyDef;
                doorBodyDef.type = b2_dynamicBody;
                doorBodyDef.position.Set(2,24);
                b2Body *doorBody = m_world->CreateBody(&doorBodyDef);

                b2PolygonShape doorBodyShape;
                doorBodyShape.SetAsBox(2,4,b2Vec2(0,0),0);

                b2FixtureDef doorBodyFixtureDef;
                doorBodyFixtureDef.shape = &doorBodyShape;
                doorBodyFixtureDef.density = 0.1f;
                doorBodyFixtureDef.filter.groupIndex = -1;
                doorBody->CreateFixture(&doorBodyFixtureDef);

                b2PrismaticJointDef doorToBoxJoint;
                doorToBoxJoint.bodyA = box1;
                doorToBoxJoint.bodyB = doorBody;
                doorToBoxJoint.localAxisA = b2Vec2(1,0);
                doorToBoxJoint.collideConnected = false;

                doorToBoxJoint.localAnchorA.Set(0,-5);
                doorToBoxJoint.localAnchorB.Set(0,-4);
                m_world->CreateJoint(&doorToBoxJoint);

                b2WheelJointDef mainRodToDoorJoint;
                mainRodToDoorJoint.Initialize(doorBody, doorMainRod, doorMainRod->GetPosition()+b2Vec2(0,-2), b2Vec2(0,1));
                mainRodToDoorJoint.localAnchorA.Set(-2,0);
                m_world->CreateJoint(&mainRodToDoorJoint);

                doorMainRod->SetTransform(b2Vec2(-4+3.5/2,32-3.5/2), 3.14/4);


            }


        }

        {
            /// \par 1.SHELF
            /*! Creates a shelf \n
             *  b2EdgeShape shape is modified and assigned to ground
             */
             b2PolygonShape shape;
             shape.SetAsBox(0.5f, 10.f);

             b2BodyDef bd1;
             bd1.position.Set(7.5f, 10.0f);

             b2FixtureDef *fd2 = new b2FixtureDef;
             fd2->density = 5.0;
             fd2->friction = 0.0f;
             fd2->restitution = 0.f;
             fd2->shape = &shape;

            /*! Variable -  ground
             *  \n \brief A ground line , size=12.0*0.5
             *  \n Data type - b2EdgeShape
             */
             b2Body* ground1 = m_world->CreateBody(&bd1);
             ground1->CreateFixture(fd2);

             b2BodyDef bd2;
             bd2.position.Set(7.5f, 38.0f);

            /*! Variable -  ground
             *  \n \brief A ground line , size=12.0*0.5
             *  \n Data type - b2EdgeShape
             */
             b2Body* ground2 = m_world->CreateBody(&bd2);
             ground2->CreateFixture(fd2);

             b2BodyDef bd3;
             bd3.position.Set(-7.5f, 10.0f);
            /*! Variable -  ground
             *  \n \brief A ground line for the horizontal shelf , size=14.0*0.5
             *  \n Data type - b2EdgeShape
             */
             b2Body* ground3 = m_world->CreateBody(&bd3);
             ground3->CreateFixture(fd2);

             b2BodyDef bd4;
             bd4.position.Set(-7.5f, 38.0f);
            /*! Variable -  ground
             *  \n \brief A ground line , size=12.0*0.5
             *  \n Data type - b2EdgeShape
             */
             b2Body* ground4 = m_world->CreateBody(&bd4);
             ground4->CreateFixture(fd2);

             b2PrismaticJointDef* prismaticJointDef = new b2PrismaticJointDef;
             prismaticJointDef->bodyB = ground3;
             prismaticJointDef->bodyA = box1;
             prismaticJointDef->collideConnected = false;
             prismaticJointDef->localAxisA.Set(0,1);
            prismaticJointDef->localAnchorB.Set( 0.5,6);//a little outside the bottom right corner
            prismaticJointDef->localAnchorA.Set(-7,-5);//bottom left corner
  		    prismaticJointDef->enableMotor = false;//5 units per second in positive axis direction
            prismaticJointDef->maxMotorForce = 100000.;
            prismaticJointDef->motorSpeed = 5.;
            prismaticJointDef->enableLimit = true;
            prismaticJointDef->lowerTranslation = -20;
            prismaticJointDef->upperTranslation = 30;
            prismaticJoint =  (b2PrismaticJoint*)m_world->CreateJoint(prismaticJointDef);
            prismaticJointDef->bodyA = ground4;
            prismaticJointDef->bodyB = box1;
            prismaticJointDef->collideConnected = false;
            prismaticJointDef->localAxisA.Set(0,1);
            prismaticJointDef->localAnchorA.Set( 0.5,-6);//a little outside the bottom right corner
            prismaticJointDef->localAnchorB.Set(-7,5);//bottom left corner
            prismaticJointDef->enableMotor = false;//5 units per second in positive axis direction       
            m_world->CreateJoint(prismaticJointDef);

            b2PrismaticJointDef* prismaticJointDef2 = new b2PrismaticJointDef;
            prismaticJointDef2->bodyA = ground1;
            prismaticJointDef2->bodyB = box1;
            prismaticJointDef2->collideConnected = false;
            prismaticJointDef2->localAxisA.Set(0,1);
            prismaticJointDef2->localAnchorA.Set(-0.5,6);//a little outside the bottom right corner
            prismaticJointDef2->localAnchorB.Set(7,-5);//bottom left corner
  	        //prismaticJointDef2->enableMotor = true;//5 units per second in positive axis direction
            //prismaticJointDef2->maxMotorForce = 1000;
            //prismaticJointDef2->motorSpeed = -5;
            m_world->CreateJoint(prismaticJointDef2);
            prismaticJointDef2->bodyA = ground2;
            prismaticJointDef2->bodyB = box1;
            prismaticJointDef2->collideConnected = false;
            prismaticJointDef2->localAxisA.Set(0,1);
            prismaticJointDef2->localAnchorA.Set( -0.5,-6);//a little outside the bottom right corner
            prismaticJointDef2->localAnchorB.Set(7,5);//bottom left corner
            prismaticJointDef2->enableMotor = false;//5 units per second in positive axis direction       
            m_world->CreateJoint(prismaticJointDef2);

        }


        {
          /// \par 2. PULLEY
          /*! Creates the second, new pulley system \n
           * The box on the see-saw is caught by a bar on one end of the pulley \n
           * The bar on the other end catces a revolving platform
           */


            /*! Variable -  bs2
             *  \n \brief The shape and position of the bar
             *  \n Data type -  b2PolygonShape
             */
             b2PolygonShape bs2;
             bs2.SetAsBox(3.5,7, b2Vec2(0.f,-1.9f), 0);
             fd1->shape = &bs2;

            //The bar2
             bd->position.Set(-30,10);
             fd1->density = 25.0f;

            /*! Variable -  box2
             * \n \brief The bar at the other end of the system , siz=4*0.4
             * \n Data type -  b2Body*
             * \n Values - side fd1
             */
             box2 = m_world->CreateBody(bd);
             b2Fixture* weights = box2->CreateFixture(fd1);


            /*! Variable -  bs3
             *  \n \brief The shape and position of the bar
             *  \n Data type -  b2PolygonShape
             */
             b2PolygonShape bs3;
             bs3.SetAsBox(0.5,3, b2Vec2(0.f,0.f), 0);
             fd1->shape = &bs3;

            //The bar3
             bd->position.Set(6,32);
             bd->fixedRotation = false;
             fd1->density = 1.0;
            /*! Variable -  box3
             * \n \brief The bar at the other end of the system , siz=4*0.4
             * \n Data type -  b2Body*
             * \n Values - side fd1
             */
             b2Body* box3 = m_world->CreateBody(bd);
             box3->CreateFixture(fd1);
	          //  m_world->registerPhysicsConnector(new PhysicsConnector(box1, box3, true, true));        


            /*! Variable -  bs4
             *  \n \brief The shape and position of the bar
             *  \n Data type -  b2PolygonShape
             */

            //The bar4
             bd->position.Set(-6,32);
             bd->fixedRotation = false;
             fd1->density = 1.0;
            /*! Variable -  box4
             * \n \brief The bar at the other end of the system , siz=4*0.4
             * \n Data type -  b2Body*
             * \n Values - side fd1
             */
             b2Body* box4 = m_world->CreateBody(bd);
             box4->CreateFixture(fd1);
            //  m_world->registerPhysicsConnector(new PhysicsConnector(box1, box3, true, true));  


             b2RevoluteJointDef *revoluteJointDef = new b2RevoluteJointDef;
             revoluteJointDef->bodyA = box1;
             revoluteJointDef->bodyB = box3;
             revoluteJointDef->collideConnected = false;
             b2Vec2 worldAnchorOnBody3(6, 5);
             revoluteJointDef->localAnchorA.Set(6,5);
             revoluteJointDef->localAnchorB.Set(0,-2 );
             revoluteJointDef->referenceAngle = 0;
             revoluteJointDef->enableMotor = true;
             revoluteJointDef->maxMotorTorque = 800;
             revoluteJointDef->motorSpeed = -5*3.14;
             revoluteJointDef->enableLimit = true;
             revoluteJointDef->lowerAngle = -3.14/4 ;
     	      //revoluteJointDef->upperAngle =  3.14/2 ;
            //revoluteJointDef->Initialize(box1, box3, box1->GetWorldCenter()+worldAnchorOnBody3);    
             m_world->CreateJoint(revoluteJointDef);

             b2RevoluteJointDef *revoluteJointDef2 = new b2RevoluteJointDef;
             revoluteJointDef2->bodyA = box1;
             revoluteJointDef2->bodyB = box4;
             revoluteJointDef2->collideConnected = false;
             b2Vec2 worldAnchorOnBody4(-6, 5);
             revoluteJointDef2->localAnchorA.Set(-6,5);
             revoluteJointDef2->localAnchorB.Set(0,-2);
             revoluteJointDef2->referenceAngle = 0;
             revoluteJointDef2->enableMotor = true;
             revoluteJointDef2->maxMotorTorque = 800;
             revoluteJointDef2->motorSpeed = 5*3.14;
             revoluteJointDef2->enableLimit = true;
            //revoluteJointDef2->lowerAngle = -3.14/4 ;
             revoluteJointDef2->upperAngle =  3.14/4 ;
            //revoluteJointDef2->Initialize(box1, box4, box1->GetWorldCenter()+worldAnchorOnBody4);    
             m_world->CreateJoint(revoluteJointDef2);

            // The pulley joint
            /*! Variable -  myjoint
             * \n \brief The pulley joint with two anchors
             * \n Data type -   b2PulleyJointDef*
             * \n Values - anchors = twoanchors on bodies worldAnchorOnBody1 and worldAnchorOnBody2, \n
             *                      two anchors on ground worldAnchorGround1 and worldAnchorGround2, ratio
             */
             b2PulleyJointDef* myjoint = new b2PulleyJointDef();
            b2Vec2 worldAnchorOnBody1(0, 3); // Anchor point on body 1 in world axis
            b2Vec2 worldAnchorOnBody2(-30, 40); // Anchor point on body 2 in world axis
            b2Vec2 worldAnchorGround1(0, 42); // Anchor point for ground 1 in world axis
            b2Vec2 worldAnchorGround2(-30, 42); // Anchor point for ground 2 in world axis
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

        }

    }

}  

float32 min = 100.;
b2WeldJoint *tempJoint;
void dominos_t::keyboard(unsigned char key) 
{   
  if(key == 'd'){    
  circleToWorldJoint->SetMotorSpeed(0);
  if((box1->GetPosition()).y > 10) box1->SetType(b2_dynamicBody);   
  prismaticJoint->SetMotorSpeed(5);
  prismaticJoint->EnableMotor(true);

}


else if(key == 'a'){    

  circleToWorldJoint->SetMotorSpeed(0);
  if((box1->GetWorldCenter()).y < 25) box1->SetType(b2_dynamicBody);
  prismaticJoint->SetMotorSpeed(-5);
  prismaticJoint->EnableMotor(false);
}

else if(key == 'l'){    
  circleToWorldJoint->SetMotorSpeed(1);
    if((box1->GetWorldCenter()).y >= 25){
      box1->SetType(b2_staticBody);  
  }

  if((box1->GetWorldCenter()).y <= 10){
      box1->SetType(b2_staticBody);  
  }

  prismaticJoint->EnableMotor(false);

  if(tempJoint!=0) {
    m_world->DestroyJoint(tempJoint);
  }
}

else if(key == 'c'){
  circleToWorldJoint->SetMotorSpeed(0);
    box1->SetType(b2_dynamicBody);  
    box2->GetWorld()->DestroyBody(box2);  
}

}       


    ///   The dominos _t class is attached the name dominos by passing the base_sim_t* class dominos_t using the dominos::create function to the constructor of sim_t
sim_t *sim = new sim_t("Dominos", dominos_t::create);

}
