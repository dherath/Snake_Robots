#include "movementPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(movementPlugin)

//---------------------------------Constructor----------------------------------
movementPlugin::movementPlugin()
{
	printf("------------------------------------ \n");
	printf("Initialized movementPlugin() : P4 \n");
}

//---------------------------------Loading Model at start-----------------------
void movementPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{

	this->model =_model;
	printf("loaded model \n");

	for(int i=0;i<NUM_JOINTS;i++){ 
		for(int j=0;j<NUM_JOINTS_PER_LINK;j++){ 
			std::string name = "plink_"+std::to_string(i+2)+"_MOVE_"+std::to_string(j+1);
			joints[i][j]=_model->GetJoint(name);
			actuatorSeq[i][j] =0; //all retracted in the begining 
		}
	}

	printf("Got all joints \n");

	for(int i=0; i<4;i++){
		int val= 5*(i+1)-1;// gives the final index of each block    B->F
		//int val= 5*i;//gives the first index of each block           F->B
		lamdaLimit[0][i]= val;
		lamdaLimit[1][i]= 5; // no use ryt now since its static->otherwise break each section to blocks
	}

	maxLimit=this->joints[0][0]->GetUpperLimit(0); //0 ->static for now
	minLimit=this->joints[0][0]->GetLowerLimit(0); //-0.03

	k0=0,k1=1,k2=2,k3=3;
	initialConfigCount=0;
	count=0;
	sequenceCount=0;

	printf("initial config set \n");

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&movementPlugin::OnUpdate, this));

}

//---------------------------------Update Function------------------------------

void movementPlugin::OnUpdate()
{
	//------[ B4-> B3 -> B2 -> B1 ]---------------------
	//-------initial config-----------------------------
	if(initialConfigCount<150){
		left(k3,5);
		initialConfigCount++;
	}else if(initialConfigCount<300){
		shrinkLeft(k3,5);
		left(k2,5);
		initialConfigCount++;
	}else if(initialConfigCount<450){
		right(k3,5);
		shrinkLeft(k2,5);
		left(k1,5);
		initialConfigCount++;
	}else if(initialConfigCount<600){
		shrinkRight(k3,5);
		right(k2,5);
		shrinkLeft(k1,5);
		left(k0,5); 
		initialConfigCount++;
	}else{
	//-------continues congfig-------------------------
		if(sequenceCount==0){
			left(k3,5);
			shrinkRight(k2,5);
			right(k1,5);
			shrinkLeft(k0,5);
			count++;
		}else if(sequenceCount==1){
			shrinkLeft(k3,5);
			left(k2,5);
			shrinkRight(k1,5);
			right(k0,5);
			count++;
		}else if(sequenceCount==2){
			right(k3,5);
			shrinkLeft(k2,5);
			left(k1,5);
			shrinkRight(k0,5);
			count++;
		}else if(sequenceCount==3){
			shrinkRight(k3,5);
			right(k2,5);
			shrinkLeft(k1,5);
			left(k0,5);
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

//-------------------------------------------------------------------------------
void movementPlugin::retractScrewJoint(int _jointPos, int _jointIndex)
{
	math::Angle currentAngle = this->joints[_jointPos][_jointIndex]->GetAngle(0);
	if (currentAngle < maxLimit){
		joints[_jointPos][_jointIndex]->SetForce(0,-30);
		actuatorSeq[_jointPos][_jointIndex] =1;				
	}else if(currentAngle == maxLimit){
		actuatorSeq[_jointPos][_jointIndex]=0;
	}
}

void movementPlugin::expandScrewJoint(int _jointPos, int _jointIndex)
{
	math::Angle currentAngle = this->joints[_jointPos][_jointIndex]->GetAngle(0);
	if (currentAngle > minLimit){
		joints[_jointPos][_jointIndex]->SetForce(0,30);
		actuatorSeq[_jointPos][_jointIndex]=0;		
	}else if (currentAngle == minLimit){
		actuatorSeq[_jointPos][_jointIndex] =1;
	}	
}
//-------------------------------------------------------------------------------
void movementPlugin::left(int k,int limit)
{
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		expandScrewJoint(loc,3);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		expandScrewJoint(loc,1);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,2);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,0);
	}	
}

void movementPlugin::right(int k,int limit)
{
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		expandScrewJoint(loc,2);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		expandScrewJoint(loc,0);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,3);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,1);
	}	
}

void movementPlugin::shrinkLeft(int k,int limit)
{
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,3);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,1);
	}
	/*for(int i=0;i<limit;i++){
		//int loc=lamdaLimit[0][k]-i; //  B->F
		int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,2);
	}
	for(int i=0;i<limit;i++){
		//int loc=lamdaLimit[0][k]-i; //  B->F
		int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,0);
	}*/
}

void movementPlugin::shrinkRight(int k,int limit)
{
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,2);
	}
	for(int i=0;i<limit;i++){
		int loc=lamdaLimit[0][k]-i; //  B->F
		//int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,0);
	}
	/*for(int i=0;i<limit;i++){
		//int loc=lamdaLimit[0][k]-i; //  B->F
		int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,3);
	}
	for(int i=0;i<limit;i++){
		//int loc=lamdaLimit[0][k]-i; //  B->F
		int loc=lamdaLimit[0][k]+i; //    F->B
		retractScrewJoint(loc,1);
	}	*/
}