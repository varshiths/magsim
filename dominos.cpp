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
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "cs251_base.hpp"
#include "render.hpp"
#include <iostream>

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

      // void base_sim_t::keyboard(unsigned char key){
        // switch(key){
          // case 'a':
          // std::cout << key << endl;
          // bodysp->SetLinearVelocity(b2Vec2(0,-1));
        // }
      // }

namespace cs251
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t()
  {
    //Ground
    /*! \var b1 
     * \brief pointer to the body ground 
     */ 

    m_world->SetGravity(b2Vec2(0,-4));



    b2Body* b1;  
    {
      
      b2BodyDef boundaryDef;
      boundaryDef.type = b2_staticBody;
      boundaryDef.position = b2Vec2(0,0);
      b2Body* boundary = m_world->CreateBody(&boundaryDef);
      
      m_ground_body = m_world->CreateBody(&boundaryDef);

      b2EdgeShape line;
      b2FixtureDef lineDefFix;
      lineDefFix.restitution = 0.7;
      lineDefFix.shape = &line;

      line.Set(b2Vec2(-44.0f, -4.0f), b2Vec2(44.0f, -4.0f));
      m_ground_body->CreateFixture(&lineDefFix);

      // lineDefFix.shape = &line;
      line.Set(b2Vec2(-44.0f, 44.0f), b2Vec2(44.0f, 44.0f));
      boundary->CreateFixture(&lineDefFix);
      
      // lineDefFix.shape = &line;
      line.Set(b2Vec2(-44.0f, -4.0f), b2Vec2(-44.0f, 44.0f));
      boundary->CreateFixture(&lineDefFix);

      // lineDefFix.shape = &line;
      line.Set(b2Vec2(44.0f, -4.0f), b2Vec2(44.0f, 44.0f));
      boundary->CreateFixture(&lineDefFix);
     
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-5.0f, 0.0f), b2Vec2(5.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }

    {
      // shape definition
      b2CircleShape circleShape;
      circleShape.m_p.Set(0,0);
      circleShape.m_radius = 1.5f;

      // b2FixtureDef circleFix;

      // metallic->SetUserData(metallic);

      b2BodyDef magnet1Def;
      magnet1Def.type = b2_staticBody;
      magnet1Def.position.Set(-15,20);

      b2BodyDef magnet2Def;
      magnet2Def.type = b2_staticBody;
      magnet2Def.position.Set(15,20);      

      // namer = "agn";
      circleFix.shape = &circleShape;
      circleFix.restitution = 0.8;

      circleFix.density = 1.1f;
      b2Body* magnet1 = m_world->CreateBody(&magnet1Def);
      magnet1->CreateFixture(&circleFix);

      circleFix.density = 2.1f;
      b2Body* magnet2 = m_world->CreateBody(&magnet2Def);
      magnet2->CreateFixture(&circleFix);      


      circleShape.m_radius = 1.0f;
      circleFix.shape = &circleShape;
      circleFix.density = 2;

      b2BodyDef metalDef;
      metalDef.type = b2_dynamicBody;
      metalDef.position.Set(5,30);
      b2Body* metallic = m_world->CreateBody(&metalDef);
      metallic->CreateFixture(&circleFix);

      // metallic->SetUserData(&circleFix);


    }
          
  }

  sim_t *sim = new sim_t("Magnets", dominos_t::create);
}
