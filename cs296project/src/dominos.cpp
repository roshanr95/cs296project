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

       /* The is the constructor \n
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
              shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(-32.0f, 0.0f));
              b2BodyDef bd; 
              b1 = m_world->CreateBody(&bd); 
              b1->CreateFixture(&shape, 0.0f);
              shape.Set(b2Vec2(-28.0f, 0.0f), b2Vec2(-5.0f, 0.0f));
              b1->CreateFixture(&shape, 0.0f);
              shape.Set(b2Vec2(5.0f, 0.0f), b2Vec2(90.0f, 0.0f));
              b1->CreateFixture(&shape, 0.0f);

          }

      }

      {   /// \par ELEVATOR BOX

              /*! Variable -  bd
               *  \n \brief A body definition of elevator box
               *  \n Data type -  b2FixtureDef*
               *  \n Values - fixed rotation,position= (0,30)
               */
               b2BodyDef *bd = new b2BodyDef;
               bd->type = b2_dynamicBody;
               bd->position.Set(0,30);
               bd->fixedRotation = true;

              /*! Variable -  fd1
               *  \n \brief fixture for elevator box
               *  \n Data type -  b2FixtureDef*
               *  \n Values - friction = 0.0f, restitution = 0f
               */
               b2FixtureDef *fd1 = new b2FixtureDef;
               fd1->friction = 0.0f;
               fd1->restitution = 0.f;
               fd1->shape = new b2PolygonShape;

              /*! Variable -  bs1
               *  \n \brief The shape and position of the elevator box
               *  \n Data type -  b2PolygonShape
               */
               b2PolygonShape bs1;
               bs1.SetAsBox(7,5, b2Vec2(0.f,0.f), 0);
               fd1->shape = &bs1;

              
               bd->position.Set(0,25);
               fd1->density =  5.;
               fd1->filter.groupIndex = -1;

              /*! Variable -  box1
               * \n \brief The Elevator Box
               * \n Data type -  b2Body*
               * \n Values - fixture fd1
               */
               box1 = m_world->CreateBody(bd);
               box1->CreateFixture(fd1);
               fd1->filter.groupIndex = 0;


               {
                /*! Variable - centerCircleDef
                 * \n \brief body definition of center circle
                 * \n Values - position=(0,32)
                 * Data type - b2BodyDef
                 */ 
                 b2BodyDef centerCircleDef;
                 centerCircleDef.type = b2_dynamicBody;
                 centerCircleDef.position.Set(0,32);
                 b2Body* centreCircle = m_world->CreateBody(&centerCircleDef);
          
                /*! Variable - circleShape
                 * \n \brief shape for center circle
                 * \n Values - position=(0,0)
                 * \n Data Type - b2CircleShape
                 */
                 b2CircleShape circleShape;
                 circleShape.m_p.Set(0, 0); 
                 circleShape.m_radius = 1.5f; 
                 
                /*! Variable - centerCircleFixtureDef
                 * \n \brief fixture for center circle
                 * \n Values - density=1.0f 
                 * \n Data Type - b2FixtureDef
                 */
                 b2FixtureDef centerCircleFixtureDef;
                 centerCircleFixtureDef.shape = &circleShape;
                 centerCircleFixtureDef.density = 1.0f;
                 centerCircleFixtureDef.filter.groupIndex = -1;
                 centreCircle->CreateFixture(&centerCircleFixtureDef);
                 
                /*! Variable - rectBodyDef1
                 * \n \brief Body definition for door
                 * \n Values - position=(1,31)
                 * \n Data Type - b2BodyDef
                 */
                 b2BodyDef rectBodyDef1;
                 rectBodyDef1.type = b2_dynamicBody;
                 rectBodyDef1.position.Set(1,31);
                 b2Body* doorRect1 = m_world->CreateBody(&rectBodyDef1);
                 
                 /*! Variable - rectShape1
                  * \n \brief shape for door1
                  * \n Values - length=0.2 , height =1 ,  position= (0,0) , angle = 0
                  * \n Data Type - b2PolgonShape
                  */
                 b2PolygonShape rectShape1;
                 rectShape1.SetAsBox(0.2,1,b2Vec2(0,0), 0);
                 
                 /*! Variable - rectFixtureDef
                  * \n \brief fixture for door1
                  * \n Values - density=0.1f , 
                  * \n Data Type - b2CircleShape
                  */
                 b2FixtureDef rectFixtureDef;
                 rectFixtureDef.shape = &rectShape1;
                 rectFixtureDef.density = 0.1f;
                 rectFixtureDef.filter.groupIndex = -1;
                 doorRect1->CreateFixture(&rectFixtureDef);
                 
                 /*! Variable - rodJointDef1
                  * \n \brief revolute joint between door and center circle
                  * \n Values - collide connect = false
                  * \n Data Type - b2RevoluteJointDef
                  */
                 b2RevoluteJointDef rodJointDef1;
                 rodJointDef1.bodyA = centreCircle;
                 rodJointDef1.bodyB = doorRect1;
                 rodJointDef1.collideConnected = false;
                 
                 

                 rodJointDef1.localAnchorA.Set(1,0);
                 rodJointDef1.localAnchorB.Set(0,1);
                 m_world->CreateJoint(&rodJointDef1);
                 
                 /*! Variable - circleToWorldJointDef
                  * \n \brief revolute joint between center circle and elevator box
                  * \n Values - enable motor , collide connect , max motor torque , motor speed
                  * \n Data Type - b2CircleShape
                  */
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
                   
                  /*! Variable - rectShape3
                   * \n \brief shape for door1
                   * \n Values - position=(-2,5,-1) length=2.5 height=0.2
                   * \n Data Type - b2CircleShape
                   */ 
                  b2PolygonShape rectShape3;
                  rectShape3.SetAsBox(2.5,0.2,b2Vec2(-2.5,-1), 0);
                  
                  /*! Variable - rectFixtureDef3
                   * \n \brief fixture for door1
                   * \n Values - 
                   * \n Data Type - b2CircleShape
                   */
                  b2FixtureDef rectFixtureDef3;
                  rectFixtureDef3.shape = &rectShape3;
                  rectFixtureDef3.filter.groupIndex = -1;
                  doorRect1->CreateFixture(&rectFixtureDef3);

              }

              {
          
                  /*! Variable - doorMainRodDef
                   * \n \brief body definition for door main rod
                   * \n Values - position=(-4,29.5)
                   * \n Data Type - b2BodyDef
                   */
                  b2BodyDef doorMainRodDef;
                  doorMainRodDef.type = b2_dynamicBody;
                  doorMainRodDef.position.Set(-4,29.5);
                  b2Body *doorMainRod = m_world->CreateBody(&doorMainRodDef);
                  
                  /*! Variable - doorMainRodShape
                   * \n \brief shape for door Main Rod
                   * \n Values - length=0.3 height=2.5 position=(0,0)
                   * \n Data Type - b2PolygonShape
                   */
                  b2PolygonShape doorMainRodShape;
                  doorMainRodShape.SetAsBox(0.3,2.5,b2Vec2(0,0),0);
                  
                  /*! Variable - doorMainRodFixtureDef
                   * \n \brief fixture for door main rod
                   * \n Values - density = 0.1f 
                   * \n Data Type - b2FixtureDef
                   */
                  b2FixtureDef doorMainRodFixtureDef;
                  doorMainRodFixtureDef.shape = &doorMainRodShape;
                  doorMainRodFixtureDef.density = 0.1f;
                  doorMainRodFixtureDef.filter.groupIndex = -1;
                  doorMainRod->CreateFixture(&doorMainRodFixtureDef);
                  
                  /*! Variable - rodToWorldDef
                   * \n \brief Revolute joint between elevator box and door main rod
                   * \n Values - collide connect = false
                   * \n Data Type - b2RevoluteJointDef
                   */
                  b2RevoluteJointDef rodToWorldJointDef;
                  rodToWorldJointDef.bodyA = box1;
                  rodToWorldJointDef.bodyB = doorMainRod;
                  rodToWorldJointDef.collideConnected = false;

                  rodToWorldJointDef.localAnchorA.Set(-4,7);
                  rodToWorldJointDef.localAnchorB.Set(0,2.5);
                  m_world->CreateJoint(&rodToWorldJointDef);
                  
                  /*! Variable - rodToRodJointDef
                   * \n \brief Revolute joint between door1 and door main rod
                   * \n Values - collide connect = false
                   * \n Data Type - b2RevoluteJointDef
                   */
                  b2RevoluteJointDef rodToRodJointDef;
                  rodToRodJointDef.bodyA = doorRect1;
                  rodToRodJointDef.bodyB = doorMainRod;
                  rodToRodJointDef.collideConnected = false;

                  rodToRodJointDef.localAnchorA.Set(-3.5,-1);
                  rodToRodJointDef.localAnchorB.Set(0,0);
                  m_world->CreateJoint(&rodToRodJointDef);
                  
                  /*! Variable - doorBodyDef
                   * \n \brief body definition for door
                   * \n Values - position=(2,24)
                   * \n Data Type - b2BodyDef
                   */
                  b2BodyDef doorBodyDef;
                  doorBodyDef.type = b2_dynamicBody;
                  doorBodyDef.position.Set(2,24);
                  doorBody = m_world->CreateBody(&doorBodyDef);
                  doorBodyDef.position.Set(1,24);
                  b2Body* doorEntrance = m_world->CreateBody(&doorBodyDef);
                  
                  /*! Variable - doorBodyShape
                   * \n \brief shape for door
                   * \n Values - length = 2 , height = 4 , position = (0,0) 
                   * \n Data Type - b2PolygonShape
                   */
                  b2PolygonShape doorBodyShape;
                  doorBodyShape.SetAsBox(2,4,b2Vec2(0,0),0);
                  
                  /*! Variable - doorBodyFixtureDef
                   * \n \brief fixture for door
                   * \n Values - density = 0.1f 
                   * \n Data Type - b2FixtureDef
                   */
                  b2FixtureDef doorBodyFixtureDef;
                  doorBodyFixtureDef.shape = &doorBodyShape;
                  doorBodyFixtureDef.density = 0.1f;
                  doorBodyFixtureDef.filter.groupIndex = -1;
                  doorBody->CreateFixture(&doorBodyFixtureDef);

                  doorBodyShape.SetAsBox(1.5,4,b2Vec2(0,0),0);
                  doorBodyFixtureDef.shape = &doorBodyShape;
                  doorEntrance->CreateFixture(&doorBodyFixtureDef);

                  b2WeldJointDef* wj = new b2WeldJointDef;
                  wj->Initialize(box1,doorEntrance,box1->GetWorldCenter());
                  m_world->CreateJoint(wj);
                  
                  /*! Variable - doorTBoxJoint
                   * \n \brief prismatic joint between elevator box and door
                   * \n Values - collide connect = false , local axis = (1,0)
                   * \n Data Type - b2PrismaticJointDef
                   */
                  b2PrismaticJointDef doorToBoxJoint;
                  doorToBoxJoint.bodyA = box1;
                  doorToBoxJoint.bodyB = doorBody;
                  doorToBoxJoint.localAxisA = b2Vec2(1,0);
                  doorToBoxJoint.collideConnected = false;

                  doorToBoxJoint.localAnchorA.Set(0,-5);
                  doorToBoxJoint.localAnchorB.Set(0,-4);
                  m_world->CreateJoint(&doorToBoxJoint);

                 /*! Variable - mainRodToDoorJoint
                  * \n \brief wheel joint between door and door main rod
                  * \n Data Type - b2WheelJointDef
                  */
                  b2WheelJointDef mainRodToDoorJoint;
                  mainRodToDoorJoint.Initialize(doorBody, doorMainRod, doorMainRod->GetPosition()+b2Vec2(0,-2), b2Vec2(0,1));
                  mainRodToDoorJoint.localAnchorA.Set(-2,0);
                  m_world->CreateJoint(&mainRodToDoorJoint);

                  doorMainRod->SetTransform(b2Vec2(-4+3.5/2,32-3.5/2), 3.14/4);


              }


          }

          { /// \par FRAMES AND EQUILISING BOX

              /*! Variable - shape
               * \n \brief shape for frames
               * \n Values - position=(0.5f , 10.0f)
               * Data Type = b2PolygonShape
               */ 
               b2PolygonShape shape;
               shape.SetAsBox(0.5f, 10.f);
               
               /*! Variable - bd1
                * \n \brief bode definition for frame1( at bottom right position )
                * \n Values - position=(7.5f , 10.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd1;
               bd1.position.Set(7.5f, 10.0f);
               
               /*! Variable - fd2
                * \n \brief fixture for ground1
                * \n Values - density = 5.0 friction = 0.0f , restitution = 0.0f
                * Data Type = b2FixtureDef
                */ 
               b2FixtureDef *fd2 = new b2FixtureDef;
               fd2->density = 5.0;
               fd2->friction = 0.0f;
               fd2->restitution = 0.f;
               fd2->shape = &shape;
               fd2->filter.groupIndex = -1;

              /*! Variable -  ground1
               *  \n \brief body for frame1
               *  \n Data type - b2Body
               */
               b2Body* ground1 = m_world->CreateBody(&bd1);
               ground1->CreateFixture(fd2);

               /*! Variable - bd2
                * \n \brief body defintion for frame2 (at top right position)
                * \n Values - position=(7.5f , 38.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd2;
               bd2.position.Set(7.5f, 30.0f);

              /*! Variable -  ground2
               * \n \brief body for frame2
               *  \n Data type - b2Body
               */
               b2Body* ground2 = m_world->CreateBody(&bd2);
               ground2->CreateFixture(fd2);

               /*! Variable - bd3
                * \n \brief body definition for frame3(at bottom left position)
                * \n Values - position=(-7.5f , 10.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd3;
               bd3.position.Set(-7.5f, 10.0f);

              /*! Variable -  ground3
               * \n \brief body for frame3
               *  \n Data type - b2Body
               */
               b2Body* ground3 = m_world->CreateBody(&bd3);
               ground3->CreateFixture(fd2);
               
               /*! Variable - bd4
                * \n \brief body definition for frame4(at top left position)
                * \n Values - position=(-7.5f , 38.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd4;
               bd4.position.Set(-7.5f, 30.0f);

              /*! Variable -  ground4
               * \n \brief body for frame4
               *  \n Data type - b2Body
               */
               b2Body* ground4 = m_world->CreateBody(&bd4);
               ground4->CreateFixture(fd2);
               
               /*! Variable - prismaticJointDef
                * \n \brief prismatic joint between frame3 and elevator box
                * Data Type = b2PrismaticJointDef
                */ 
               b2PrismaticJointDef* prismaticJointDef = new b2PrismaticJointDef;
               prismaticJointDef->bodyB = ground3;
               prismaticJointDef->bodyA = box1;
               prismaticJointDef->collideConnected = false;
               prismaticJointDef->localAxisA.Set(0,1);
               prismaticJointDef->localAnchorB.Set( 0.5,0);//a little outside the bottom right corner
               prismaticJointDef->localAnchorA.Set(-7,-5);//bottom left corner
               prismaticJointDef->enableMotor = false;//5 units per second in positive axis direction
               prismaticJointDef->maxMotorForce = 100000.;
               prismaticJointDef->motorSpeed = 5.;
               prismaticJointDef->enableLimit = true;
               prismaticJointDef->lowerTranslation = -10;
               prismaticJointDef->upperTranslation = 10;
               prismaticJoint =  (b2PrismaticJoint*)m_world->CreateJoint(prismaticJointDef);

               prismaticJointDef->bodyA = ground4;
               prismaticJointDef->bodyB = box1;
               prismaticJointDef->collideConnected = false;
               prismaticJointDef->localAxisA.Set(0,1);
               prismaticJointDef->localAnchorA.Set( 0.5,0);//a little outside the bottom right corner
               prismaticJointDef->localAnchorB.Set(-7,5);//bottom left corner
               prismaticJointDef->enableMotor = false;//5 units per second in positive axis direction   
               prismaticJointDef->enableLimit = false;    
               m_world->CreateJoint(prismaticJointDef);

             /*! Variable - prismaticJointDef2
              * \n \brief prismatic joint between elevator box and frame 
              * Data Type = b2PrismaticJointDef
              */ 
              b2PrismaticJointDef* prismaticJointDef2 = new b2PrismaticJointDef;
              prismaticJointDef2->bodyA = ground1;
              prismaticJointDef2->bodyB = box1;
              prismaticJointDef2->collideConnected = false;
              prismaticJointDef2->localAxisA.Set(0,1);
              prismaticJointDef2->localAnchorA.Set(-0.5,0);//a little outside the bottom right corner
              prismaticJointDef2->localAnchorB.Set(7,-5);//bottom left corner
              m_world->CreateJoint(prismaticJointDef2);

              prismaticJointDef2->bodyA = ground2;
              prismaticJointDef2->bodyB = box1;
              prismaticJointDef2->collideConnected = false;
              prismaticJointDef2->localAxisA.Set(0,1);
              prismaticJointDef2->localAnchorA.Set( -0.5,0);//a little outside the bottom right corner
              prismaticJointDef2->localAnchorB.Set(7,5);//bottom left corner
              prismaticJointDef2->enableMotor = false;//5 units per second in positive axis direction       
              m_world->CreateJoint(prismaticJointDef2);

          }




          {
            
              /*! Variable -  bs2
               *  \n \brief The shape and position of the equilising box
               *  \n Data type -  b2PolygonShape
               */
               b2PolygonShape bs2;
               bs2.SetAsBox(3.5,7, b2Vec2(0.f,-1.9f), 0);
               fd1->shape = &bs2;

              //The bar2
               bd->position.Set(-30,10);
               fd1->density = 50.0f;

              /*! Variable -  box2
               * \n \brief the equilising box
               * \n Data type -  b2Body*
               */
               box2 = m_world->CreateBody(bd);
               box2->CreateFixture(fd1);

              /*! Variable -  bs3
               *  \n \brief The shape and position of box3
               *  \n Data type -  b2PolygonShape
               */
               b2PolygonShape bs3;
               bs3.SetAsBox(0.5,3, b2Vec2(0.f,0.f), 0);
               fd1->shape = &bs3;

              //The bar3
               bd->position.Set(6,32);
               bd->fixedRotation = false;
               fd1->density = 5.0;
               fd1->filter.groupIndex = -1;

               /// \par FRAMES AND EQUILISING BOX

              /*! Variable - shape
               * \n \brief shape for frames
               * \n Values - position=(0.5f , 10.0f)
               * Data Type = b2PolygonShape
               */ 
               b2PolygonShape shape;
               shape.SetAsBox(0.5f, 10.f);
               
               /*! Variable - bd1
                * \n \brief bode definition for frame1( at bottom right position )
                * \n Values - position=(7.5f , 10.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd1;
               bd1.position.Set(-26.0f, 10.0f);
               
               /*! Variable - fd2
                * \n \brief fixture for ground1
                * \n Values - density = 5.0 friction = 0.0f , restitution = 0.0f
                * Data Type = b2FixtureDef
                */ 
               b2FixtureDef *fd2 = new b2FixtureDef;
               fd2->density = 5.0;
               fd2->friction = 0.0f;
               fd2->restitution = 0.f;
               fd2->shape = &shape;
               fd2->filter.groupIndex = -1;

              /*! Variable -  ground1
               *  \n \brief body for frame1
               *  \n Data type - b2Body
               */
               b2Body* ground1 = m_world->CreateBody(&bd1);
               ground1->CreateFixture(fd2);

               /*! Variable - bd2
                * \n \brief body defintion for frame2 (at top right position)
                * \n Values - position=(7.5f , 38.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd2;
               bd2.position.Set(-26.0f, 30.0f);

              /*! Variable -  ground2
               * \n \brief body for frame2
               *  \n Data type - b2Body
               */
               b2Body* ground2 = m_world->CreateBody(&bd2);
               ground2->CreateFixture(fd2);

               /*! Variable - bd3
                * \n \brief body definition for frame3(at bottom left position)
                * \n Values - position=(-7.5f , 10.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd3;
               bd3.position.Set(-34.0f, 10.0f);

              /*! Variable -  ground3
               * \n \brief body for frame3
               *  \n Data type - b2Body
               */
               b2Body* ground3 = m_world->CreateBody(&bd3);
               ground3->CreateFixture(fd2);
               
               /*! Variable - bd4
                * \n \brief body definition for frame4(at top left position)
                * \n Values - position=(-7.5f , 38.0f)
                * Data Type = b2BodyDef
                */ 
               b2BodyDef bd4;
               bd4.position.Set(-34.0f, 30.0f);

              /*! Variable -  ground4
               * \n \brief body for frame4
               *  \n Data type - b2Body
               */
               b2Body* ground4 = m_world->CreateBody(&bd4);
               ground4->CreateFixture(fd2);
               
               /*! Variable - prismaticJointDef
                * \n \brief prismatic joint between frame3 and elevator box
                * Data Type = b2PrismaticJointDef
                */ 
               b2PrismaticJointDef* prismaticJointDef = new b2PrismaticJointDef;
               prismaticJointDef->bodyB = ground3;
               prismaticJointDef->bodyA = box2;
               prismaticJointDef->collideConnected = false;
               prismaticJointDef->localAxisA.Set(0,1);
               prismaticJointDef->localAnchorB.Set( 0.5,0);//a little outside the bottom right corner
               prismaticJointDef->localAnchorA.Set(-3.5,-5);//bottom left corner
               prismaticJointDef->enableMotor = false;//5 units per second in positive axis direction
               prismaticJointDef->maxMotorForce = 100000.;
               prismaticJointDef->motorSpeed = 5.;
               prismaticJointDef->enableLimit = true;
               prismaticJointDef->lowerTranslation = -10;
               prismaticJointDef->upperTranslation = 10;
               m_world->CreateJoint(prismaticJointDef);

               prismaticJointDef->bodyA = ground4;
               prismaticJointDef->bodyB = box2;
               prismaticJointDef->collideConnected = false;
               prismaticJointDef->localAxisA.Set(0,1);
               prismaticJointDef->localAnchorA.Set( 0.5,0);//a little outside the bottom right corner
               prismaticJointDef->localAnchorB.Set(-3.5,5);//bottom left corner
               prismaticJointDef->enableMotor = false;//5 units per second in positive axis direction   
               prismaticJointDef->enableLimit = false;    
               m_world->CreateJoint(prismaticJointDef);

             /*! Variable - prismaticJointDef2
              * \n \brief prismatic joint between elevator box and frame 
              * Data Type = b2PrismaticJointDef
              */ 
              b2PrismaticJointDef* prismaticJointDef2 = new b2PrismaticJointDef;
              prismaticJointDef2->bodyA = ground1;
              prismaticJointDef2->bodyB = box2;
              prismaticJointDef2->collideConnected = false;
              prismaticJointDef2->localAxisA.Set(0,1);
              prismaticJointDef2->localAnchorA.Set(-0.5,0);//a little outside the bottom right corner
              prismaticJointDef2->localAnchorB.Set(3.5,-5);//bottom left corner
              m_world->CreateJoint(prismaticJointDef2);

              prismaticJointDef2->bodyA = ground2;
              prismaticJointDef2->bodyB = box2;
              prismaticJointDef2->collideConnected = false;
              prismaticJointDef2->localAxisA.Set(0,1);
              prismaticJointDef2->localAnchorA.Set( -0.5,0);//a little outside the bottom right corner
              prismaticJointDef2->localAnchorB.Set(3.5,5);//bottom left corner
              prismaticJointDef2->enableMotor = false;//5 units per second in positive axis direction       
              m_world->CreateJoint(prismaticJointDef2);

              /*! Variable -  box3
               \n \brief safety latch1
               * \n Data type -  b2Body*
               * \n Values - side fd1
               */
               b2Body* box3 = m_world->CreateBody(bd);
               box3->CreateFixture(fd1);      

               b2Vec2 vertices[3];
               vertices[0].Set(0.5f, 3.0f);
               vertices[1].Set(1.f, 2.8f);
               vertices[2].Set(0.5f, 2.8f);
               int32 count = 3;
               b2PolygonShape safetyLatchShape2;
               safetyLatchShape2.Set(vertices, count);

               b2FixtureDef safetyLatchFixture2;
               safetyLatchFixture2.shape = &safetyLatchShape2;
               safetyLatchFixture2.density = 10.f;
               safetyLatchFixture2.filter.groupIndex = -1;
               safetyLatchFixture2.restitution = 0.f;
               box3->CreateFixture(&safetyLatchFixture2); 

              //The bar4
               bd->position.Set(-6,32);
               bd->fixedRotation = false;
               fd1->density = 5.0;

              /*! Variable -  box4
               \n \brief safety latch2
               * \n Data type -  b2Body*
               * \n Values - side fd1
               */
               b2Body* box4 = m_world->CreateBody(bd);
               box4->CreateFixture(fd1);
 
               vertices[0].Set(-0.5f, 3.0f);
               vertices[1].Set(-1.f, 2.8f);
               vertices[2].Set(-0.5f, 2.8f);
               safetyLatchShape2.Set(vertices, count);

               safetyLatchFixture2.shape = &safetyLatchShape2;
               box4->CreateFixture(&safetyLatchFixture2);

              /*! Variable -  revoluteJointDef
               * \n \brief revolute joint between elevator and safety latch 1
               * \n Data type -  b2RevoluteJoint
               * \n Values - side fd1
               */
               b2RevoluteJointDef *revoluteJointDef = new b2RevoluteJointDef;
               revoluteJointDef->bodyA = box1;
               revoluteJointDef->bodyB = box3;
               revoluteJointDef->collideConnected = false;
               b2Vec2 worldAnchorOnBody3(6, 5);
               revoluteJointDef->localAnchorA.Set(6,5);
               revoluteJointDef->localAnchorB.Set(0,-2 );
               revoluteJointDef->referenceAngle = 0;
               revoluteJointDef->enableMotor = true;
               revoluteJointDef->maxMotorTorque = 1500;
               revoluteJointDef->motorSpeed = -1;
               revoluteJointDef->enableLimit = true;
               revoluteJointDef->lowerAngle = -3.14/4 ; 
               m_world->CreateJoint(revoluteJointDef);
               
              /*! Variable -  revoluteJointDef2
               * \n \brief revolute joint between elevator box and safety latch2
               * \n Data type -  b2RevoluteJoint
               * \n Values - side fd1
               */
               b2RevoluteJointDef *revoluteJointDef2 = new b2RevoluteJointDef;
               revoluteJointDef2->bodyA = box1;
               revoluteJointDef2->bodyB = box4;
               revoluteJointDef2->collideConnected = false;
               b2Vec2 worldAnchorOnBody4(-6, 5);
               revoluteJointDef2->localAnchorA.Set(-6,5);
               revoluteJointDef2->localAnchorB.Set(0,-2);
               revoluteJointDef2->referenceAngle = 0;
               revoluteJointDef2->enableMotor = true;
               revoluteJointDef2->maxMotorTorque = 1500;
               revoluteJointDef2->motorSpeed = 1;
               revoluteJointDef2->enableLimit = true;
               revoluteJointDef2->upperAngle =  3.14/4 ;
               m_world->CreateJoint(revoluteJointDef2);

              // The pulley joint
              /*! Variable -  myjoint
               * \n \brief The pulley joint between safety latch1 and equilising box
               * \n Data type -   b2PulleyJointDef*
               * \n Values - anchors = twoanchors on bodies worldAnchorOnBody1 and worldAnchorOnBody2, \n
               *  two anchors on ground worldAnchorGround1 and worldAnchorGround2, ratio
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
              /*! The pulley joint myjoint is re-initialised with all the input values - equilising box and safety latch 2, ratio, anchors
               */
              myjoint2->Initialize(box4, box2, worldAnchorGround1, worldAnchorGround2, box4->GetWorldCenter()+worldAnchorOnBody1, box2->GetWorldCenter(), ratio);
              m_world->CreateJoint(myjoint2);

               
          }

          {
            b2BodyDef spikesRightDef;
            spikesRightDef.type = b2_staticBody;
            spikesRightDef.position.Set(9,0);
            b2Body *spikesRight = m_world->CreateBody(&spikesRightDef);

            b2Vec2 vertices[3];
            vertices[0].Set(1.0f, 0.0f);
            vertices[1].Set(0.0f, 2.5f);
            vertices[2].Set(1.0f, 1.0f);
            int32 count = 3;
            b2PolygonShape spikeRightShape;
            spikeRightShape.Set(vertices, count);

            for(int i=0; i<40; i++) {
                b2FixtureDef spikeRightFixture;
                spikeRightFixture.shape = &spikeRightShape;
                spikeRightFixture.density = 0.1f;
                spikeRightFixture.filter.groupIndex = 0;
                spikeRightFixture.restitution = 0.f;
                spikesRight->CreateFixture(&spikeRightFixture);
                vertices[0] += b2Vec2(0,1);
                vertices[1] += b2Vec2(0,1);
                vertices[2] += b2Vec2(0,1);
                spikeRightShape.Set(vertices, count);
            }
        }

        {
            b2BodyDef spikesLeftDef;
            spikesLeftDef.type = b2_staticBody;
            spikesLeftDef.position.Set(-9,0);
            b2Body *spikesLeft = m_world->CreateBody(&spikesLeftDef);

            b2Vec2 vertices[3];
            vertices[0].Set(-1.0f, 0.0f);
            vertices[1].Set(0.0f, 2.5f);
            vertices[2].Set(-1.0f, 1.0f);
            int32 count = 3;
            b2PolygonShape spikeLeftShape;
            spikeLeftShape.Set(vertices, count);

            for(int i=0; i<40; i++) {
                b2FixtureDef spikeLeftFixture;
                spikeLeftFixture.shape = &spikeLeftShape;
                spikeLeftFixture.density = 0.1f;
                spikeLeftFixture.filter.groupIndex = 0;
                spikeLeftFixture.restitution = 0.f;
                spikesLeft->CreateFixture(&spikeLeftFixture);
                vertices[0] += b2Vec2(0,1);
                vertices[1] += b2Vec2(0,1);
                vertices[2] += b2Vec2(0,1);
                spikeLeftShape.Set(vertices, count);
            }
        }

        {

            b2PolygonShape bar;
            bar.SetAsBox(0.2,0.5);
            b2BodyDef* bodyDef = new b2BodyDef();
            bodyDef->type = b2_dynamicBody;
            // initial body
            bodyDef->position.x=-30;
            bodyDef->position.y=1;
            b2FixtureDef* boxDef = new b2FixtureDef();
            boxDef->shape=&bar;
            boxDef->density=1;
            boxDef->friction=0.5;
            boxDef->restitution=0.2;
            b2Body* body=m_world->CreateBody(bodyDef);
            body->CreateFixture(boxDef);
            b2Body * link = body;

            b2WeldJointDef* weld_joint = new b2WeldJointDef;
            weld_joint->Initialize(box2, body, b2Vec2(-30, 1.5));
            m_world->CreateJoint(weld_joint); 

           for (int i = 1; i <= 3; i++) {
                // rope segment
                b2BodyDef* bodyDef = new b2BodyDef();
                bodyDef->type = b2_dynamicBody;
                bodyDef->position.x=-30;
                bodyDef->position.y=1-i;
                b2FixtureDef* boxDef = new b2FixtureDef();
                boxDef->shape=&bar;
                boxDef->density=1;
                boxDef->friction=0.5;
                boxDef->restitution=0.2;
                b2Body* body=m_world->CreateBody(bodyDef);
                body->CreateFixture(boxDef);
                // joint
                b2RevoluteJointDef* revolute_joint = new b2RevoluteJointDef;
                revolute_joint->Initialize(link, body, b2Vec2(-30, 1-i+0.5));
                m_world->CreateJoint(revolute_joint);
                // saving the reference of the last placed link
                link=body;
            }

            // final body

            bodyDef->position.x=-30;
            bodyDef->position.y=-3;
            b2Body* body2 = m_world->CreateBody(bodyDef);
            body2->CreateFixture(boxDef);
            b2RevoluteJointDef* revolute_joint = new b2RevoluteJointDef;
            revolute_joint->Initialize(link, body2, b2Vec2(-30, -2.5));
            m_world->CreateJoint(revolute_joint);

            link = body2;
            bar.SetAsBox(0.5,0.2);  

            for (int i = 1; i <= 29; i++) {
                // rope segment
                b2BodyDef* bodyDef = new b2BodyDef();
                bodyDef->type = b2_dynamicBody;
                bodyDef->position.x=-30+i-0.5;
                bodyDef->position.y=-3;
                b2FixtureDef* boxDef = new b2FixtureDef();
                boxDef->shape=&bar;
                boxDef->density=1;
                boxDef->friction=0.5;
                boxDef->restitution=0.2;
                b2Body* body=m_world->CreateBody(bodyDef);
                body->CreateFixture(boxDef);
                // joint
                b2RevoluteJointDef* revolute_joint = new b2RevoluteJointDef;
                revolute_joint->Initialize(link, body, b2Vec2(-30-1+i, -3));
                m_world->CreateJoint(revolute_joint);
                // saving the reference of the last placed link
                link=body;
            }

             // final body
            bodyDef->position.x=-0.5;
            bodyDef->position.y=-3;
            b2Body* body3 = m_world->CreateBody(bodyDef);
            boxDef->shape=&bar;
            body3->CreateFixture(boxDef);
            revolute_joint->Initialize(link, body3, b2Vec2(-1, -3));
            m_world->CreateJoint(revolute_joint);

            link = body3;
            bar.SetAsBox(0.2,0.5);  

            for (int i = 1; i <= 23; i++) {
                // rope segment
                b2BodyDef* bodyDef = new b2BodyDef();
                bodyDef->type = b2_dynamicBody;
                bodyDef->position.x=0;
                bodyDef->position.y=-3+0.5+i-1;
                b2FixtureDef* boxDef = new b2FixtureDef();
                boxDef->shape=&bar;
                boxDef->density=1;
                boxDef->friction=0.5;
                boxDef->restitution=0.2;
                b2Body* body=m_world->CreateBody(bodyDef);
                body->CreateFixture(boxDef);
                // joint
                b2RevoluteJointDef* revolute_joint = new b2RevoluteJointDef;
                revolute_joint->Initialize(link, body, b2Vec2(0, -3 +i -1));
                m_world->CreateJoint(revolute_joint);
                // saving the reference of the last placed link
                link=body;
            }

             // final body
            weld_joint->Initialize(box1, link, b2Vec2(0, 20));
            m_world->CreateJoint(weld_joint); 

        }

        {      
              /*! Variable - centerCircleDef
               * \n \brief body definition of center circle
               * \n Values - position=(0,32)
               * Data type - b2BodyDef
               */ 
               b2BodyDef centerCircleDef;
               centerCircleDef.type = b2_dynamicBody;
               centerCircleDef.position.Set(-2,-2);
               b2Body* centreCircle = m_world->CreateBody(&centerCircleDef);
               centerCircleDef.type = b2_staticBody;       
               b2Body* nail = m_world->CreateBody(&centerCircleDef);

              /*! Variable - circleShape
               * \n \brief shape for center circle
               * \n Values - position=(0,0)
               * \n Data Type - b2CircleShape
               */
               b2CircleShape circleShape;
               circleShape.m_p.Set(0, 0); 
               circleShape.m_radius = 1.5f; 
               
              /*! Variable - centerCircleFixtureDef
               * \n \brief fixture for center circle
               * \n Values - density=1.0f 
               * \n Data Type - b2FixtureDef
               */
               b2FixtureDef centerCircleFixtureDef;
               centerCircleFixtureDef.shape = &circleShape;
               centerCircleFixtureDef.density = 1.0f;
               centreCircle->CreateFixture(&centerCircleFixtureDef);

               circleShape.m_radius = 1.f;
               centerCircleFixtureDef.shape = &circleShape;
               nail->CreateFixture(&centerCircleFixtureDef);

               /*! Variable - circleToWorldJointDef
                * \n \brief revolute joint between center circle and elevator box
                * \n Values - enable motor , collide connect , max motor torque , motor speed
                * \n Data Type - b2CircleShape
                */
               b2RevoluteJointDef circleToWorldJointDef;
               circleToWorldJointDef.bodyA = centreCircle;
               circleToWorldJointDef.bodyB = nail;
               circleToWorldJointDef.collideConnected = false;
               circleToWorldJointDef.enableMotor = true;
               circleToWorldJointDef.maxMotorTorque = 1000;
               circleToWorldJointDef.motorSpeed = 0;

               circleToWorldJointDef.localAnchorA.Set(0,0);
               circleToWorldJointDef.localAnchorB.Set(0,0);
               b2RevoluteJoint* circleToWorldJoint2 = (b2RevoluteJoint *)m_world->CreateJoint(&circleToWorldJointDef);
          }       

          {      
              /*! Variable - centerCircleDef
               * \n \brief body definition of center circle
               * \n Values - position=(0,32)
               * Data type - b2BodyDef
               */ 
               b2BodyDef centerCircleDef;
               centerCircleDef.type = b2_dynamicBody;
               centerCircleDef.position.Set(-28,-2);
               b2Body* centreCircle = m_world->CreateBody(&centerCircleDef);
               centerCircleDef.type = b2_staticBody;       
               b2Body* nail = m_world->CreateBody(&centerCircleDef);

              /*! Variable - circleShape
               * \n \brief shape for center circle
               * \n Values - position=(0,0)
               * \n Data Type - b2CircleShape
               */
               b2CircleShape circleShape;
               circleShape.m_p.Set(0, 0); 
               circleShape.m_radius = 1.5f; 
               
              /*! Variable - centerCircleFixtureDef
               * \n \brief fixture for center circle
               * \n Values - density=1.0f 
               * \n Data Type - b2FixtureDef
               */
               b2FixtureDef centerCircleFixtureDef;
               centerCircleFixtureDef.shape = &circleShape;
               centerCircleFixtureDef.density = 1.0f;
               centreCircle->CreateFixture(&centerCircleFixtureDef);

               circleShape.m_radius = 1.f;
               centerCircleFixtureDef.shape = &circleShape;
               nail->CreateFixture(&centerCircleFixtureDef);

               /*! Variable - circleToWorldJointDef
                * \n \brief revolute joint between center circle and elevator box
                * \n Values - enable motor , collide connect , max motor torque , motor speed
                * \n Data Type - b2CircleShape
                */
               b2RevoluteJointDef circleToWorldJointDef;
               circleToWorldJointDef.bodyA = centreCircle;
               circleToWorldJointDef.bodyB = nail;
               circleToWorldJointDef.collideConnected = false;
               circleToWorldJointDef.enableMotor = true;
               circleToWorldJointDef.maxMotorTorque = 1000;
               circleToWorldJointDef.motorSpeed = 0;

               circleToWorldJointDef.localAnchorA.Set(0,0);
               circleToWorldJointDef.localAnchorB.Set(0,0);
               b2RevoluteJoint* circleToWorldJoint2 = (b2RevoluteJoint *)m_world->CreateJoint(&circleToWorldJointDef);
          }       
      }

  }  

float32 min = 100.;
b2WeldJoint *tempJoint;

/*! Function - Keyboard
 * \n \brief to control keyboard events
 */
void dominos_t::keyboard(unsigned char key) 
{   

if(key == 'd'){    
  if(doorBody->GetWorldCenter().x > 0.85){
    circleToWorldJoint->SetMotorSpeed(0);
    if((box1->GetPosition()).y > 10) box1->SetType(b2_dynamicBody);   
    prismaticJoint->SetMotorSpeed(5);
    prismaticJoint->EnableMotor(true);
  }
}

else if(key == 'a'){    
  if(doorBody->GetWorldCenter().x > 0.85){
    if((box1->GetPosition()).y < 25) box1->SetType(b2_dynamicBody);   
    circleToWorldJoint->SetMotorSpeed(0);
    prismaticJoint->SetMotorSpeed(-5);
    prismaticJoint->EnableMotor(true);
  }
}

else if(key == 'c'){
    prismaticJoint->EnableMotor(false);
    circleToWorldJoint->SetMotorSpeed(0);
    box1->SetType(b2_dynamicBody);  
    box2->GetWorld()->DestroyBody(box2); 
}

else if(key == 'l'){    
  if(((box1->GetWorldCenter()).y >= 25) || ((box1->GetWorldCenter()).y <= 10)){
    start = true;
    circleToWorldJoint->SetMotorSpeed(0.5);
    box1->SetType(b2_staticBody);  
    prismaticJoint->EnableMotor(false);
    if(tempJoint!=0) {
      m_world->DestroyJoint(tempJoint);
    }
  }

}   

}    

///   The dominos _t class is attached the name dominos by passing the base_sim_t* class dominos_t using the dominos::create function to the constructor of sim_t
sim_t *sim = new sim_t("Dominos", dominos_t::create);

}
