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

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_

namespace cs296
{
  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  class dominos_t : public base_sim_t
  {
  public:
    
    dominos_t();
    void keyboard(unsigned char key); 
    static base_sim_t* create()
    {
      return new dominos_t;
    }
  };

     /*! Variables - box1 , box2 , doorBody1
      * \n \brief bodies for elevator box ,  equalising box and door1 respectively
      * \n Data Type - b2Body
      */
      b2Body* box1;
      b2Body* box2;
      
      /*! Variables - temp
       * \n Data Type - b2WeldJointDef
       */
      b2WeldJointDef* temp = new b2WeldJointDef;
      /*! Variables - b2PrismaticJoint
       * \n Data Type - b2PrismaticJoint
       */
      b2PrismaticJoint* prismaticJoint;

      b2RevoluteJoint* safetyLatchJoint1;
      b2RevoluteJoint* safetyLatchJoint2;
   
}

      b2Body *doorBody; 
      /*! Variables - circleToWorldJoint
       * \n Data Type - b2RevoluteJoint
       */
      b2RevoluteJoint* circleToWorldJoint;
      bool start;

  
#endif
