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

#define NUM_JOINTS 20 

namespace gazebo
{
	class movementPlugin :public ModelPlugin
	{
		///////////////////////////------------function prototypes-----------------//////////////////////////////////////////////////////////

		public: 
			void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf);
			void OnUpdate();
			void moveJointLeft(int _jointPos);
			void moveJointRight(int _jointPos);
			void shrinkJoint(int _jointPos);
			void left(int k,int limit);
			void right(int k,int limit);
			void shrink(int k,int limit);
			movementPlugin();

		
		///////////////////////////------------variable declarations--------------///////////////////////////////////////////////////////////

		private: 
			physics::ModelPtr model;
			event::ConnectionPtr updateConnection;
			physics::JointPtr joints[NUM_JOINTS];// 20*1 array
			math::Angle maxLimit;
			math::Angle minLimit;
			int lamdaLimit[2][4];
			int count;
			int initialConfigCount;
			int sequenceCount;
			int k0,k1,k2,k3;

	};
}

#endif
