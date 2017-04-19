
#include "movementPlugin.hh"


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(movementPlugin)

//---------------------------------Constructor----------------------------------
movementPlugin::movementPlugin()
{
	printf("------------------------------------ \n");
	printf("Initialized movementPlugin(): S1 \n");
}

//---------------------------------Loading Model at start-----------------------
void movementPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{

	this->model =_model;
	printf("loaded model \n");

	for(int i=0;i<NUM_JOINTS;i++){ 
			std::string name = "plink_"+std::to_string(i+2)+"_MOVE";
			joints[i]=_model->GetJoint(name);
	}

	printf("Got all joints \n");

	for(int i=0; i<4;i++){
		//int val= 5*(i+1)-1;// gives the final index of each block    B->F
		int val= 5*i;//gives the first index of each block           F->B
		lamdaLimit[0][i]= val;
		lamdaLimit[1][i]= 5; 
	}

	maxLimit=this->joints[0]->GetUpperLimit(0); //25 deg->static for now
	minLimit=this->joints[0]->GetLowerLimit(0); // -25 deg

	k0=0,k1=1,k2=2,k3=3;
	initialConfigCount=0;
	count=0;
	sequenceCount=0;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&movementPlugin::OnUpdate, this));

}

//---------------------------------Update Function------------------------------

void movementPlugin::OnUpdate()
{
	//------[ B1-> B2 -> B3 -> B4 ]---------------------
	//-------initial config-----------------------------
	if(initialConfigCount<150){
		left(k0,5);
		initialConfigCount++;
	}else if(initialConfigCount<300){
		shrink(k0,5);
		left(k1,5);
		initialConfigCount++;
	}else if(initialConfigCount<450){
		right(k0,5);
		shrink(k1,5);
		left(k2,5);
		initialConfigCount++;
	}else if(initialConfigCount<600){
		shrink(k0,5);
		right(k1,5);
		shrink(k2,5);
		left(k3,5);
		initialConfigCount++;
	}else{
	//-------continues congfig-------------------------
		if(sequenceCount==0){
			left(k0,5);
			shrink(k1,5);
			right(k2,5);
			shrink(k3,5);
			count++;
		}else if(sequenceCount==1){
			shrink(k0,5);
			left(k1,5);
			shrink(k2,5);
			right(k3,5);
			count++;
		}else if(sequenceCount==2){
			right(k0,5);
			shrink(k1,5);
			left(k2,5);
			shrink(k3,5);
			count++;
		}else if(sequenceCount==3){
			shrink(k0,5);
			right(k1,5);
			shrink(k2,5);
			left(k3,5);
			count++;
		}
	}
	//----------seconds counter-----------------------
	if(count>150){
		count=0;
		sequenceCount=(sequenceCount+1)%4;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                movement functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void movementPlugin::moveJointLeft(int _jointPos)
{
	math::Angle currentAngle = this->joints[_jointPos]->GetAngle(0);
	if(currentAngle <maxLimit ){
		joints[_jointPos]->SetForce(0,10);//left
	}
}

void movementPlugin::moveJointRight(int _jointPos)
{
	math::Angle currentAngle = this->joints[_jointPos]->GetAngle(0);
	if(currentAngle >minLimit ){
		joints[_jointPos]->SetForce(0,-10);//right
	}
}

void movementPlugin::shrinkJoint(int _jointPos)
{
	math::Angle currentAngle = this->joints[_jointPos]->GetAngle(0);
	math::Angle upper = maxLimit/2;
	math::Angle lower = minLimit/2;	
	if(currentAngle < lower ){
		joints[_jointPos]->SetForce(0,-10);
	}else if(currentAngle >upper){
		joints[_jointPos]->SetForce(0,10);
	}	
}
//---------------------------------------------------------------------------

void movementPlugin::left(int k,int limit)
{
	for(int i=0;i<limit;i++){
		//int loc=lamdaLimit[0][k]-i; //  B->F
		int loc=lamdaLimit[0][k]+i; //    F->B
		moveJointLeft(loc);		
	}
}

void movementPlugin::right(int k,int limit)
{
	for(int i=0;i<limit;i++){
		//int loc=lamdaLimit[0][k]-i; //  B->F
		int loc=lamdaLimit[0][k]+i; //    F->B
		moveJointRight(loc);		
	}
}

void movementPlugin::shrink(int k,int limit)
{
	for(int i=0;i<limit;i++){
		//int loc=lamdaLimit[0][k]-i; //  B->F
		int loc=lamdaLimit[0][k]+i; //    F->B
		shrinkJoint(loc);		
	}	
}