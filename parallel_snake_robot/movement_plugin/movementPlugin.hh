#ifndef _GAZEBO_MOVEMENT_PLUGIN_HH_
#define _GAZEBO_MOVEMENT_PLUGIN_HH_


#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <stdio.h>
#include <math.h>
#include <string>

#define NUM_JOINTS_PER_LINK 4
#define NUM_JOINTS 20 

namespace gazebo
{
	class movementPlugin :public ModelPlugin
	{
		///////////////////////////------------function prototypes-----------------//////////////////////////////////////////////////////////
		public: movementPlugin();
		public:	void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf);
		public: void OnUpdate();
		public: void retractScrewJoint(int _jointPos, int _jointIndex);// _jointPos->[1..20] _jointIndex->[0..3]
		public: void expandScrewJoint(int _jointPos, int _jointIndex);
		public: void left(int k,int limit);
		public: void right(int k,int limit);
		public: void shrinkLeft(int k,int limit);
		public: void shrinkRight(int k,int limit);
		
		///////////////////////////------------variable declarations--------------///////////////////////////////////////////////////////////
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: physics::JointPtr joints[NUM_JOINTS][NUM_JOINTS_PER_LINK];// 19*4 array
		private: math::Angle maxLimit;//the max length of the prismatic joint
		private: math::Angle minLimit;//minimum limit->same for all joints
		private: int lamdaLimit[2][4]; // [starting point of lamda/4 (back to forward w.r.t joints)][length of lamda/4(w.r.t joints)]->will be static for now
		private: int actuatorSeq[NUM_JOINTS][NUM_JOINTS_PER_LINK];//retract=0 expand =1
		private: int currentOrientation[NUM_JOINTS];//0-shrinked,1-expaned,2-turned left,3-turned right
		private: int commandSeq[4];//sequence of movement for lamda/4s -> 0,1,2,3->
		private: int isCommComplt;
		private:int count;
		private:int initialConfigCount;
		private:int sequenceCount;
		private:int k0,k1,k2,k3;

	};
}

#endif
