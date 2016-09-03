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
#include "callbacks.hpp"

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

namespace cs251
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t():
  f1toggler(-1), f2toggler(-1)
  {
    //Ground
    /*! \var b1 
     * \brief pointer to the body ground 
     */ 
    m_world->SetGravity(b2Vec2(0,-4));
    // m_world->SetContactListener(this);
      
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
    m_ground_body->CreateFixture(&lineDefFix); // ground

    line.Set(b2Vec2(-44.0f, 44.0f), b2Vec2(44.0f, 44.0f));
    boundary->CreateFixture(&lineDefFix);
    
    line.Set(b2Vec2(-44.0f, -4.0f), b2Vec2(-44.0f, 44.0f));
    boundary->CreateFixture(&lineDefFix);

    line.Set(b2Vec2(44.0f, -4.0f), b2Vec2(44.0f, 44.0f));
    boundary->CreateFixture(&lineDefFix);
   
    b2EdgeShape shape; 
    shape.Set(b2Vec2(-5.0f, 0.0f), b2Vec2(5.0f, 0.0f));
    b2BodyDef bd; 
    paddle = m_world->CreateBody(&bd); 
    paddle->CreateFixture(&shape, 0.0f);

    // shape definition
    b2CircleShape circleShape;
    circleShape.m_p.Set(0,0);
    circleShape.m_radius = 1.5f;

    b2BodyDef magnet1Def;
    magnet1Def.type = b2_staticBody;
    magnet1Def.position.Set(-15,20);

    b2BodyDef magnet2Def;
    magnet2Def.type = b2_staticBody;
    magnet2Def.position.Set(15,20);      

    circleFix.shape = &circleShape;
    circleFix.restitution = 0.8;

    circleFix.density = 1.1f;
    mag1 = m_world->CreateBody(&magnet1Def);
    mag1->CreateFixture(&circleFix);

    circleFix.density = 2.1f;
    mag2 = m_world->CreateBody(&magnet2Def);
    mag2->CreateFixture(&circleFix);      

    circleShape.m_radius = 1.0f;
    circleFix.shape = &circleShape;
    circleFix.density = 2;

    b2BodyDef metalDef;
    metalDef.type = b2_dynamicBody;
    metalDef.position.Set(5,30);
    
    metalBall = m_world->CreateBody(&metalDef);
    metalBall->CreateFixture(&circleFix);
    
          
  }

  void dominos_t::step(settings_t* settings)
  {

      base_sim_t::step(settings);

      b2Vec2 mag1Cen = mag1->GetWorldCenter();
      b2Vec2 mag2Cen = mag2->GetWorldCenter();

      b2Body* metal;
      metal = m_world->GetBodyList();

      int children = m_world->GetBodyCount()-6;
      
      for(int i = 0; i < children and metal != NULL; i++){ 
        b2Vec2 metCen = metal->GetWorldCenter();
  
        b2Vec2 force1 = f1toggler*(75.0f/(metCen-mag1Cen).LengthSquared()*(metCen-mag1Cen).Length())*(metCen-mag1Cen);
        b2Vec2 force2 = f2toggler*(75.0f/(metCen-mag2Cen).LengthSquared()*(metCen-mag2Cen).Length())*(metCen-mag2Cen);
        metal->ApplyForceToCenter(force1+force2,true);
        metal = metal->GetNext();
      }

      b2ContactEdge* paddleHit = paddle->GetContactList();

      if(paddleHit != NULL){

        b2Body* hit = paddleHit->other;

        srand(time(NULL));
        hit->SetTransform(b2Vec2(rand()%80-40,rand()%10+30),0);
        hit->SetLinearVelocity(b2Vec2(0,0));

        srand(time(NULL));
        b2BodyDef temp;
        temp.position.Set(rand()%88-44,rand()%10+33);
        temp.type = b2_dynamicBody;

        b2CircleShape circleShape;
        circleShape.m_p.Set(0,0);
        circleShape.m_radius = 1.0f;
        circleFix.shape = &circleShape;
        circleFix.density = 2;

        b2Body* child = m_world->CreateBody(&temp);
        child->CreateFixture(&circleFix);
        
      }

      b2ContactEdge* abyssHit = m_ground_body->GetContactList();

      if(abyssHit != NULL){
            cout << "Score: " << m_world->GetBodyCount()-6 << endl;
            cs251::callbacks_t::restart_cb(0);
      }      

  }

  void dominos_t::keyboard(unsigned char key) {
    if(key == 'q'){
      f1toggler = -1; f2toggler = -1;
    }

    if(key == 't'){
      f1toggler = -1*f1toggler; f2toggler = -1*f2toggler;
    }

    if(key == 'f' and paddle->GetWorldCenter().x < +39){
      paddle->SetTransform(paddle->GetWorldCenter() + b2Vec2(1.5f,0),paddle->GetAngle());
    }
    
    if(key == 'd' and paddle->GetWorldCenter().x > -39){
      paddle->SetTransform(paddle->GetWorldCenter() - b2Vec2(1.5f,0),paddle->GetAngle());
    }
  }

  void dominos_t::mouse_down(const b2Vec2& p) {
    if( mag1->GetFixtureList()->TestPoint(p) ) f1toggler = -1*f1toggler;
    if( mag2->GetFixtureList()->TestPoint(p) ) f2toggler = -1*f2toggler;
  }

  // void dominos_t::BeginContact(b2Contact* contact) {
  //   if(contact->GetFixtureA()->GetBody() == m_ground_body or contact->GetFixtureB()->GetBody() == m_ground_body){

  //   }
  //   else{

  //   }
  // }

  sim_t *sim = new sim_t("", dominos_t::create);
}
