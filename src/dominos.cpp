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


   //   {
   //          /*! Variable -  bd
   //           *  \n \brief A body definition of a pulley wheel
   //           *  \n Data type -  b2FixtureDef*
   //           *  \n Values - fixed rotation
   //           */
   //          b2BodyDef *bd = new b2BodyDef;
   //          bd->type = b2_dynamicBody;
   //          bd->position.Set(0,30);
   //          bd->fixedRotation = true;

   //          /*! Variable -  fd1
   //           *  \n \brief The bar that is attached to the pulleys , size=2*0.2
   //           *  \n Data type -  b2FixtureDef*
   //           *  \n Values - density = 20f, friction = 0.5f, restitution = 0f
   //           */
   //          b2FixtureDef *fd1 = new b2FixtureDef;
   //          fd1->friction = 0.0f;
   //          fd1->restitution = 0.f;
   //          fd1->shape = new b2PolygonShape;
         
   //          /*! Variable -  bs1
   //           *  \n \brief The shape and position of the bar 
   //           *  \n Data type -  b2PolygonShape
   //           */
   //          b2PolygonShape bs1;
   //          bs1.SetAsBox(7,5, b2Vec2(0.f,0.f), 0);
   //          fd1->shape = &bs1;
            
   //          //The bar1
   //          bd->position.Set(0,25);
   //          fd1->density =  0.1;
   //          fd1->filter.groupIndex = 0;
         
   //          /*! Variable -  box1
   //           * \n \brief The bar at one end of the system , size=4*0.4
   //           * \n Data type -  b2Body*
   //           * \n Values - side fd1
   //           */
   //          b2Body* box1 = m_world->CreateBody(bd);
   //          box1->CreateFixture(fd1);
   //          fd1->filter.groupIndex = 0;


   //        {
   //          b2PolygonShape shape;
   //          shape.SetAsBox(2.f, 4.f);
    
   //          b2BodyDef *bd1 = new b2BodyDef;
   //          bd1->position.Set(-2.f, 24.f);
   //          bd1->type = b2_dynamicBody;

   //          b2FixtureDef *fd2 = new b2FixtureDef;
   //          fd2->density = 00.0;
   //          fd2->friction = 0.0f;
   //          fd2->restitution = 0.f;
   //          fd2->shape = &shape;
   //          fd2->filter.groupIndex = -1;


   //          b2Body* door1 = m_world->CreateBody(bd1);
   //          door1->CreateFixture(fd2);

   //          b2BodyDef *bd2 = new b2BodyDef;
   //          bd2->position.Set(2.f, 24.f);
   //          bd2->type = b2_dynamicBody;

   //          b2Body* door2 = m_world->CreateBody(bd2);
   //          door2->CreateFixture(fd2);

   //        b2PrismaticJointDef* prismaticJointDef = new b2PrismaticJointDef;
   //        prismaticJointDef->bodyA = door1;
   //        prismaticJointDef->bodyB = box1;
   //        prismaticJointDef->collideConnected = false;
   //        prismaticJointDef->localAxisA.Set(1,0);
   //        prismaticJointDef->localAnchorA.Set( 0,0);//a little outside the bottom right corner
   //        prismaticJointDef->localAnchorB.Set(2,-1.5);//bottom left corner
   //        m_world->CreateJoint(prismaticJointDef);

   //        b2PrismaticJointDef* prismaticJointDef2 = new b2PrismaticJointDef;
   //        prismaticJointDef2->bodyA = door2;
   //        prismaticJointDef2->bodyB = box1;
   //        prismaticJointDef2->collideConnected = false;
   //        prismaticJointDef2->localAxisA.Set(1,0);
   //        prismaticJointDef2->localAnchorA.Set(0,0);//a little outside the bottom right corner
   //        prismaticJointDef2->localAnchorB.Set(-2,-1.5);//bottom left corner
   //        m_world->CreateJoint(prismaticJointDef2);


   //        b2PolygonShape shape2;
   //        shape2.SetAsBox(0.2f, 4.f,b2Vec2(0.,4.),0);
  
   //        fd2->shape = &shape2;

   //        door1->CreateFixture(fd2);

   //        door2->CreateFixture(fd2);

   //        b2PolygonShape shape3;
   //        shape3.SetAsBox(0.2f, 2.5f); 
    
   //        b2BodyDef *bd4 = new b2BodyDef;
   //        bd4->position.Set(0.f, 31.f);
   //        bd4->type = b2_dynamicBody;

   //        fd2->shape = &shape3;
   //        fd2->density=1.f;
   //        fd2->filter.groupIndex = 0;
   //        b2Body* circle1 = m_world->CreateBody(bd4);
   //        circle1->CreateFixture(fd2);
   //        circle1->SetTransform( circle1->GetPosition(), 3.14/3 );
   //        circle1->SetAngularVelocity(0);

   //        b2PolygonShape shape4;
   //        shape4.SetAsBox(0.1,0.1);
   //        fd2->shape = &shape4;

   //        b2BodyDef *bd5 = new b2BodyDef;
   //        bd5->type = b2_dynamicBody;
   //        bd5->position.Set(0.f, 31.f);

   //        b2Body* nail1 = m_world->CreateBody(bd5);
   //        nail1->CreateFixture(fd2);

   //        b2WeldJointDef wj;
   //        wj.Initialize(box1, nail1, box1->GetWorldCenter());//,circle1->GetWorldCenter());
   //        m_world->CreateJoint(&wj);


   //        b2DistanceJointDef dj;
   //        dj.Initialize(nail1, circle1, nail1->GetWorldCenter(),circle1->GetWorldCenter());
   //        m_world->CreateJoint(&dj);


   //        b2DistanceJointDef dj2;
   //        dj2.Initialize(door1, circle1, door1->GetWorldCenter() + b2Vec2(0,7),circle1->GetWorldCenter() + b2Vec2(-1,1));
   //        m_world->CreateJoint(&dj2);

   //        b2DistanceJointDef dj3;
   //        dj3.Initialize(door2, circle1, door2->GetWorldCenter() + b2Vec2(0,5),circle1->GetWorldCenter() + b2Vec2(1,-1));
   //        m_world->CreateJoint(&dj3);
   //        //circle1->ApplyAngularImpulse(3.0,true);

        
          
   //        }

   //        {
   //        /// \par 1.SHELF
   //        /*! Creates a shelf \n
   //         *  b2EdgeShape shape is modified and assigned to ground
   //         */
   //        b2PolygonShape shape;
   //        shape.SetAsBox(0.5f, 10.f);
    
   //        b2BodyDef bd1;
   //        bd1.position.Set(7.5f, 10.0f);

   //        b2FixtureDef *fd2 = new b2FixtureDef;
   //        fd2->density = 5.0;
   //        fd2->friction = 0.0f;
   //        fd2->restitution = 0.f;
   //        fd2->shape = &shape;

   //        /*! Variable -  ground
   //         *  \n \brief A ground line , size=12.0*0.5
   //         *  \n Data type - b2EdgeShape
   //         */
   //        b2Body* ground1 = m_world->CreateBody(&bd1);
   //        ground1->CreateFixture(fd2);

   //        b2BodyDef bd2;
   //        bd2.position.Set(7.5f, 38.0f);

   //        /*! Variable -  ground
   //         *  \n \brief A ground line , size=12.0*0.5
   //         *  \n Data type - b2EdgeShape
   //         */
   //        b2Body* ground2 = m_world->CreateBody(&bd2);
   //        ground2->CreateFixture(fd2);
    
   //        b2BodyDef bd3;
   //        bd3.position.Set(-7.5f, 10.0f);
   //        /*! Variable -  ground
   //         *  \n \brief A ground line for the horizontal shelf , size=14.0*0.5
   //         *  \n Data type - b2EdgeShape
   //         */
   //        b2Body* ground3 = m_world->CreateBody(&bd3);
   //        ground3->CreateFixture(fd2);

   //        b2BodyDef bd4;
   //        bd4.position.Set(-7.5f, 38.0f);
   //        /*! Variable -  ground
   //         *  \n \brief A ground line , size=12.0*0.5
   //         *  \n Data type - b2EdgeShape
   //         */
   //        b2Body* ground4 = m_world->CreateBody(&bd4);
   //        ground4->CreateFixture(fd2);
         
   //        b2PrismaticJointDef* prismaticJointDef = new b2PrismaticJointDef;
   //        prismaticJointDef->bodyA = ground3;
   //        prismaticJointDef->bodyB = box1;
   //        prismaticJointDef->collideConnected = false;
   //        prismaticJointDef->localAxisA.Set(0,1);
   //        prismaticJointDef->localAnchorA.Set( 0.5,6);//a little outside the bottom right corner
   //        prismaticJointDef->localAnchorB.Set(-7,-5);//bottom left corner
   //        m_world->CreateJoint(prismaticJointDef);

   //        b2PrismaticJointDef* prismaticJointDef2 = new b2PrismaticJointDef;
   //        prismaticJointDef2->bodyA = ground1;
   //        prismaticJointDef2->bodyB = box1;
   //        prismaticJointDef2->collideConnected = false;
   //        prismaticJointDef2->localAxisA.Set(0,1);
   //        prismaticJointDef2->localAnchorA.Set(-0.5,6);//a little outside the bottom right corner
   //        prismaticJointDef2->localAnchorB.Set(7,-5);//bottom left corner
   //        m_world->CreateJoint(prismaticJointDef2);

   //        }


   //        {
   //        /// \par 2. PULLEY
   //        /*! Creates the second, new pulley system \n
   //         * The box on the see-saw is caught by a bar on one end of the pulley \n
   //         * The bar on the other end catces a revolving platform
   //         */
            

   //          /*! Variable -  bs2
   //           *  \n \brief The shape and position of the bar
   //           *  \n Data type -  b2PolygonShape
   //           */
   //          b2PolygonShape bs2;
   //          bs2.SetAsBox(3.5,7, b2Vec2(0.f,-1.9f), 0);
   //          fd1->shape = &bs2;
            
   //          //The bar2
   //          bd->position.Set(-30,10);
   //          fd1->density = 20.0f;
         
   //          /*! Variable -  box2
   //           * \n \brief The bar at the other end of the system , siz=4*0.4
   //           * \n Data type -  b2Body*
   //           * \n Values - side fd1
   //           */
   //          b2Body* box2 = m_world->CreateBody(bd);
   //          b2Fixture* weights = box2->CreateFixture(fd1);
         
         


   //          /*! Variable -  bs3
   //           *  \n \brief The shape and position of the bar
   //           *  \n Data type -  b2PolygonShape
   //           */
   //          b2PolygonShape bs3;
   //          bs3.SetAsBox(0.5,3, b2Vec2(0.f,0.f), 0);
   //          fd1->shape = &bs3;
            
   //          //The bar3
   //          bd->position.Set(6,32);
   //          bd->fixedRotation = false;
   //          fd1->density = 1.0;
   //          /*! Variable -  box3
   //           * \n \brief The bar at the other end of the system , siz=4*0.4
   //           * \n Data type -  b2Body*
   //           * \n Values - side fd1
   //           */
   //          b2Body* box3 = m_world->CreateBody(bd);
   //          box3->CreateFixture(fd1);
      // //  m_world->registerPhysicsConnector(new PhysicsConnector(box1, box3, true, true));        


   //          /*! Variable -  bs4
   //           *  \n \brief The shape and position of the bar
   //           *  \n Data type -  b2PolygonShape
   //           */
         
   //          //The bar4
   //          bd->position.Set(-6,32);
   //          bd->fixedRotation = false;
   //          fd1->density = 1.0;
   //          /*! Variable -  box4
   //           * \n \brief The bar at the other end of the system , siz=4*0.4
   //           * \n Data type -  b2Body*
   //           * \n Values - side fd1
   //           */
   //          b2Body* box4 = m_world->CreateBody(bd);
   //          box4->CreateFixture(fd1);
   //        //  m_world->registerPhysicsConnector(new PhysicsConnector(box1, box3, true, true));  

         
   //          b2RevoluteJointDef *revoluteJointDef = new b2RevoluteJointDef;
   //          revoluteJointDef->bodyA = box1;
   //          revoluteJointDef->bodyB = box3;
   //          revoluteJointDef->collideConnected = false;
   //          b2Vec2 worldAnchorOnBody3(6, 5);
   //          revoluteJointDef->localAnchorA.Set(6,5);
   //          revoluteJointDef->localAnchorB.Set(0,-2 );
   //          revoluteJointDef->referenceAngle = 0;
   //          revoluteJointDef->enableMotor = true;
   //          revoluteJointDef->maxMotorTorque = 1000;
   //          revoluteJointDef->motorSpeed = -10*3.14;
   //          revoluteJointDef->enableLimit = true;
   //          revoluteJointDef->lowerAngle = -3.14/4 ;
   //               //revoluteJointDef->upperAngle =  3.14/2 ;
   //          //revoluteJointDef->Initialize(box1, box3, box1->GetWorldCenter()+worldAnchorOnBody3);    
   //          m_world->CreateJoint(revoluteJointDef);

   //          b2RevoluteJointDef *revoluteJointDef2 = new b2RevoluteJointDef;
   //          revoluteJointDef2->bodyA = box1;
   //          revoluteJointDef2->bodyB = box4;
   //          revoluteJointDef2->collideConnected = false;
   //          b2Vec2 worldAnchorOnBody4(-6, 5);
   //          revoluteJointDef2->localAnchorA.Set(-6,5);
   //          revoluteJointDef2->localAnchorB.Set(0,-2);
   //          revoluteJointDef2->referenceAngle = 0;
   //          revoluteJointDef2->enableMotor = true;
   //          revoluteJointDef2->maxMotorTorque = 1000;
   //          revoluteJointDef2->motorSpeed = 10*3.14;
   //          revoluteJointDef2->enableLimit = true;
   //          //revoluteJointDef2->lowerAngle = -3.14/4 ;
   //           revoluteJointDef2->upperAngle =  3.14/4 ;
   //          //revoluteJointDef2->Initialize(box1, box4, box1->GetWorldCenter()+worldAnchorOnBody4);    
   //          m_world->CreateJoint(revoluteJointDef2);

   //          // The pulley joint
   //          /*! Variable -  myjoint
   //           * \n \brief The pulley joint with two anchors
   //           * \n Data type -   b2PulleyJointDef*
   //           * \n Values - anchors = twoanchors on bodies worldAnchorOnBody1 and worldAnchorOnBody2, \n
   //           *                      two anchors on ground worldAnchorGround1 and worldAnchorGround2, ratio
   //           */
   //          b2PulleyJointDef* myjoint = new b2PulleyJointDef();
   //          b2Vec2 worldAnchorOnBody1(0, 3); // Anchor point on body 1 in world axis
   //          b2Vec2 worldAnchorOnBody2(-30, 40); // Anchor point on body 2 in world axis
   //          b2Vec2 worldAnchorGround1(0, 42); // Anchor point for ground 1 in world axis
   //          b2Vec2 worldAnchorGround2(-30, 42); // Anchor point for ground 2 in world axis
   //          float32 ratio = 1.0f; // Define ratio
   //          /*! The pulley joint myjoint is initialised with all the input values - box1, box2, ratio, anchors
   //           */
   //          myjoint->Initialize(box3, box2, worldAnchorGround1, worldAnchorGround2, box3->GetWorldCenter()+worldAnchorOnBody1, box2->GetWorldCenter(), ratio);
   //          m_world->CreateJoint(myjoint);

   //          b2PulleyJointDef* myjoint2 = new b2PulleyJointDef();
   //          /*! The pulley joint myjoint is initialised with all the input values - box1, box2, ratio, anchors
   //           */
   //          myjoint2->Initialize(box4, box2, worldAnchorGround1, worldAnchorGround2, box4->GetWorldCenter()+worldAnchorOnBody1, box2->GetWorldCenter(), ratio);
   //          m_world->CreateJoint(myjoint2);
            
   //        for(int i=0;i<10000;i++)
   //        box1->ApplyForceToCenter( b2Vec2(0.0f,-200.0f),  true );
            
   //        }


   //   }
    
    b2BodyDef centerCircleDef;
    centerCircleDef.type = b2_dynamicBody;
    centerCircleDef.position.Set(0,30);
    b2Body* centreCircle = m_world->CreateBody(&centerCircleDef);
    
    b2CircleShape circleShape;
    circleShape.m_p.Set(0, 0); 
    circleShape.m_radius = 4; 

    b2FixtureDef centerCircleFixtureDef;
    centerCircleFixtureDef.shape = &circleShape;
    centerCircleFixtureDef.density = 1.0f;
    centerCircleFixtureDef.filter.groupIndex = -1;
    centreCircle->CreateFixture(&centerCircleFixtureDef);

    b2BodyDef rectBodyDef1;
    rectBodyDef1.type = b2_dynamicBody;
    rectBodyDef1.position.Set(2.5,28);
    b2Body* doorRect1 = m_world->CreateBody(&rectBodyDef1);

    b2PolygonShape rectShape1;
    rectShape1.SetAsBox(0.2,2.5,b2Vec2(0,0), 0);

    b2FixtureDef rectFixtureDef;
    rectFixtureDef.shape = &rectShape1;
    rectFixtureDef.density = 0.1f;
    rectFixtureDef.filter.groupIndex = -1;
    doorRect1->CreateFixture(&rectFixtureDef);

    b2RevoluteJointDef rodJointDef1;
    rodJointDef1.bodyA = centreCircle;
    rodJointDef1.bodyB = doorRect1;
    rodJointDef1.collideConnected = false;

    rodJointDef1.localAnchorA.Set(2.5,0);
    rodJointDef1.localAnchorB.Set(0,2);
    m_world->CreateJoint(&rodJointDef1);

    b2BodyDef rectBodyDef2;
    rectBodyDef2.type = b2_staticBody;
    rectBodyDef2.position.Set(0,15);
    b2Body* mount = m_world->CreateBody(&rectBodyDef2);

    b2PolygonShape rectShape2;
    rectShape2.SetAsBox(0.5,15,b2Vec2(0,0), 0);

    b2FixtureDef rectFixtureDef2;
    rectFixtureDef2.shape = &rectShape2;
    rectFixtureDef2.filter.groupIndex = -1;
    mount->CreateFixture(&rectFixtureDef2);

    b2RevoluteJointDef circleToWorldJointDef;
    circleToWorldJointDef.bodyA = centreCircle;
    circleToWorldJointDef.bodyB = mount;
    circleToWorldJointDef.collideConnected = false;

    circleToWorldJointDef.localAnchorA.Set(0,0);
    circleToWorldJointDef.localAnchorB.Set(0,15);
    m_world->CreateJoint(&circleToWorldJointDef);

    {
        // b2BodyDef rectBodyDef3;
        // rectBodyDef3.type = b2_dynamicBody;
        // rectBodyDef3.position.Set(-2.5,25.5);
        // b2Body* doorRect3 = m_world->CreateBody(&rectBodyDef3);

        b2PolygonShape rectShape3;
        rectShape3.SetAsBox(10,0.2,b2Vec2(-10,-2.5), 0);

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
        b2BodyDef doorMechNail1Def;
        doorMechNail1Def.type = b2_staticBody;
        doorMechNail1Def.position.Set(-10,30);
        b2Body *doorMechNail1 = m_world->CreateBody(&doorMechNail1Def);

        b2BodyDef doorMainRodDef;
        doorMainRodDef.type = b2_dynamicBody;
        doorMainRodDef.position.Set(-10,20);
        b2Body *doorMainRod = m_world->CreateBody(&doorMainRodDef);

        b2PolygonShape doorMainRodShape;
        doorMainRodShape.SetAsBox(0.4,10,b2Vec2(0,0),0);

        b2FixtureDef doorMainRodFixtureDef;
        doorMainRodFixtureDef.shape = &doorMainRodShape;
        doorMainRodFixtureDef.density = 0.1f;
        doorMainRodFixtureDef.filter.groupIndex = -1;
        doorMainRod->CreateFixture(&doorMainRodFixtureDef);

        b2RevoluteJointDef rodToWorldJointDef;
        rodToWorldJointDef.bodyA = doorMechNail1;
        rodToWorldJointDef.bodyB = doorMainRod;
        rodToWorldJointDef.collideConnected = false;

        rodToWorldJointDef.localAnchorA.Set(0,0);
        rodToWorldJointDef.localAnchorB.Set(0,10);
        m_world->CreateJoint(&rodToWorldJointDef);

        b2RevoluteJointDef rodToRodJointDef;
        rodToRodJointDef.bodyA = doorRect1;
        rodToRodJointDef.bodyB = doorMainRod;
        rodToRodJointDef.collideConnected = false;

        rodToRodJointDef.localAnchorA.Set(-10,-2.5);
        rodToRodJointDef.localAnchorB.Set(0,0);
        m_world->CreateJoint(&rodToRodJointDef);

        doorMainRod->SetTransform(b2Vec2(-5,21.5), 0.5);
    }

    // doorRect1->SetTransform(centreCircle->GetPosition(), 1);
    // centreCircle->SetTransform(centreCircle->GetPosition(), 1);

    centreCircle->ApplyAngularImpulse(-1000.0f,true);


}     
     
     
    ///   The dominos _t class is attached the name dominos by passing the base_sim_t* class dominos_t using the dominos::create function to the constructor of sim_t
    sim_t *sim = new sim_t("Dominos", dominos_t::create);

  }
