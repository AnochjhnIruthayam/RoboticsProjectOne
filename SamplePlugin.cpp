#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <opencv2/opencv.hpp>

#include <QPushButton>

using namespace rws;

using namespace cv;

//global settings
WorkCell::Ptr currentWorkCell;
Device::Ptr RobotDevice;
State currentState;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn2    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn3    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn4    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
}

SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize() {
	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
}

void SamplePlugin::open(WorkCell* workcell)
{
	//get pointer and save to global pointer
	currentWorkCell = workcell;
	currentState = getRobWorkStudio()->getState();
	RobotDevice = currentWorkCell->findDevice("KukaKr16");
}

void SamplePlugin::transfromBaseToTCP(Q config)
{
	//Check if workcell is loaded
	if(currentWorkCell->getDevices().empty())
	{
		RW_THROW("Load failed");
	}
	// update the Robot Device KukaKr16
	Device::Ptr RobotDevice = currentWorkCell->findDevice("KukaKr16");

	//update state by config
	RobotDevice->setQ(config, currentState);

	//Calculates the homogeneous transform from base to the end frame
	Transform3D<double> transform = RobotDevice->baseTend(currentState);
	//output the information
	log().info()
			<< "Calculate transformation from base to TCP\nSelected Base Frame: " << RobotDevice->getBase()->getName()
			<< "\nPosition: " << transform.P()
			<< "\nRotation: " << transform.R() << "\n\n";
}



void SamplePlugin::transformFrameToFrame(Frame* from, Frame* to)
{
	if(currentWorkCell->getDevices().empty())
	{
		RW_THROW("Load failed");
	}
	//get state
	currentState = getRobWorkStudio()->getState();
	// get the Robot Device KukaKr16
	RobotDevice = currentWorkCell->findDevice("KukaKr16");
	//init Kinematics
	rw::kinematics::Kinematics tf;
	Transform3D<double>  transform = tf.frameTframe(from, to, currentState);
	log().info()
			<< "Calculate transformation from "<< from->getName() << " to " << to->getName()
			<< "\nPosition: " << transform.P()
			<< "\nRotation: " << transform.R() << "\n\n";
}



void SamplePlugin::interpolation(Q Q1, Q Q2, int sample)
{
	if(currentWorkCell->getDevices().empty())
	{
		RW_THROW("Load failed");
	}

	// calculate difference
	Q dQ = Q2-Q1;
	// calucalte stepsize
	Q step = dQ/sample;
	//run simulation
	for(int i = 0; i < sample; i++)
	{
		//calc next step
		Q QStep = Q1 + (i*step);
		//update state by Q
		RobotDevice->setQ(QStep, currentState);
		//update RobWorkStudio
		getRobWorkStudio()->setState(currentState);
		//print Qstep info
		log().info() << QStep << "\n";
	}
}

void SamplePlugin::RRT(Q Q1, Q Q2)
{
	if(currentWorkCell->getDevices().empty())
	{
		RW_THROW("Load failed");
	}
	//update state
	currentState = getRobWorkStudio()->getState();
	//update the Robot Device KukaKr16
	RobotDevice = currentWorkCell->findDevice("KukaKr16");

	// The path planning constraint is to avoid collisions.
	//tutorial from http://www.robwork.dk/apidoc/nightly/rw/page_rw_manual.html#sec_rw_manual_pathplanning
	const PlannerConstraint contraint = PlannerConstraint::make(ProximityStrategyYaobi::make(), currentWorkCell,RobotDevice, currentState);
	 // A sampler of collision free configurations for the device.
	QSampler::Ptr cfreeQ = QSampler::makeConstrained(QSampler::makeUniform(RobotDevice),contraint.getQConstraintPtr());
	Metric<Q>::Ptr currentMetric = MetricFactory::makeEuclidean<Q>();
	QToQPlanner::Ptr planner = RRTQToQPlanner::makeConnect(contraint, cfreeQ, currentMetric, 0.1);

	//initilize path. Path will save the the path from Q1 to Q2
	Path<Q> path;

	//query gets the path from Q1 to Q2 and saves to path. return true, path exists, if false, path not found
	bool ok = planner->query(Q1,Q2,path);
	if(!ok)
	{
		//if no path is found, then exit
		log().info() << "Path not found \n";
		return;
	}
	//if path found then run simulation
	for (int i= 0; i < (int)path.size()-1;i++)
	{
		//print path info
		log().info() << path.data()[i] << "\n";
		//if path is only 2 steps, then make interpolation for smooth simulation
		if((int)path.size() <= 2)
			interpolation(path[i], path[i+1], 200);
		else
		{
			//else run simulation here
			RobotDevice->setQ(path[i],currentState);
			getRobWorkStudio()->setState(currentState);
		}


	}
}

void SamplePlugin::Task1()
{
	currentState = getRobWorkStudio()->getState();
	transfromBaseToTCP(RobotDevice->getQ(currentState));
}

void SamplePlugin::Task2(Frame* from, Frame* to)
{
	transformFrameToFrame(from,to);
}

void SamplePlugin::Task3(int sample)
{
	//get state
	currentState = getRobWorkStudio()->getState();
	// get the Robot Device KukaKr16
	RobotDevice = currentWorkCell->findDevice("KukaKr16");
	// Hardcoded default state, Data taken from RobWorkStudio, when robot is in the initial state
	Q Q1(6, 0, -2.094, 1.920, 0, 1.745, 0);

	// Get Q for current state
	Q Q2 = RobotDevice->getQ(currentState);
	//set up config
	RobotDevice->setQ(Q2, currentState);
	interpolation(Q1, Q2, sample);
}

void SamplePlugin::Task4()
{
	currentState = getRobWorkStudio()->getState();
	// get the Robot Device KukaKr16
	RobotDevice = currentWorkCell->findDevice("KukaKr16");
	// Hardcoded default state, Data taken from RobWorkStudio, when robot is in the initial state
	Q Q1(6, 0, -2.094, 1.920, 0, 1.745, 0);
	// Get Q for current state
	Q Q2 = RobotDevice->getQ(currentState);
	//set up config
	RobotDevice->setQ(Q2, currentState);
	RRT(Q1,Q2);
}



void SamplePlugin::close() {
}

void SamplePlugin::btnPressed() {
	QObject *obj = sender();
	if(obj==_btn0){
		//TASK1 Base to TCP
		Task1();
	} else if(obj==_btn1){
		//TASK 2a Base to Bottle & Bottle to Base
		Task2(currentWorkCell->findFrame("KukaKr16.Base"), currentWorkCell->findFrame("Bottle"));
		Task2(currentWorkCell->findFrame("Bottle"), currentWorkCell->findFrame("KukaKr16.Base"));
	} else if(obj==_btn2){
		//TASK 2b Bottle to TCP
		Task2(currentWorkCell->findFrame("Bottle"), currentWorkCell->findFrame("ToolMount"));
		Task2(currentWorkCell->findFrame("ToolMount"), currentWorkCell->findFrame("Bottle"));
	} else if(obj == _btn3){
		Task3(200);
	} else if(obj == _btn4){
		Task4();
	}
}

void SamplePlugin::stateChangedListener(const State& state) {
}

Q_EXPORT_PLUGIN(SamplePlugin);
