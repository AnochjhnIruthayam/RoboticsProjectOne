#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudioPlugin.hpp>
#include "rw/kinematics/Kinematics.hpp"
#include <rw/kinematics/State.hpp>
#include <rw/rw.hpp>

//includes to pathplaning
#include <rw/models/WorkCell.hpp>
#include <rw/models/Models.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;

//namespace to pathplaning
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::pathplanners;
using namespace rw::trajectory;
using namespace rw::pathplanning;



class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
	SamplePlugin();
	virtual ~SamplePlugin();

	virtual void open(rw::models::WorkCell* workcell);

	virtual void close();

	virtual void initialize();

	virtual void transfromBaseToTCP(Q config);

	virtual void transformFrameToFrame(Frame* from, Frame* to);

	virtual void interpolation(Q Q1, Q Q2, int sample);

	virtual void RRT(Q Q1, Q Q2);

	virtual void Task1();

	virtual void Task2(Frame* from, Frame* to);

	virtual void Task3(int sample);

	virtual void Task4();

private slots:
	void btnPressed();

	void stateChangedListener(const rw::kinematics::State& state);
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
