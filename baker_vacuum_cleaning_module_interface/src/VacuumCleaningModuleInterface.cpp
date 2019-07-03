/*
 * Copyright (C) 2015 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file VacuumCleaningModuleInterface.C
 *    Interface for BakeR vacuum cleaning module
 *
 * @author Christof Schroeter
 * @date   2019/07/03
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <fw/Unit.h>
#include <fw/ServiceProperty.h>

#include <actors/BakeRVacuumCleaningModule.h>


namespace mira { namespace actors {

///////////////////////////////////////////////////////////////////////////////

/**
 * Interface for BakeR vacuum cleaning module
 */
class VacuumCleaningModuleInterface : public Unit
{
MIRA_OBJECT(VacuumCleaningModuleInterface)


public:

	VacuumCleaningModuleInterface();
	VacuumCleaningModuleInterface(ros::NodeHandle nh);

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		Unit::reflect(r);

		r.property("ModuleService", mModule,
		           "Used module service", "BakeRVacuumCleaningModule");

		// TODO: reflect all parameters (members and properties) that specify the persistent state of the unit
		//r.property("Param1", mParam1, "First parameter of this unit with default value", 123.4f);
		//r.member("Param2", mParam2, setter(&UnitName::setParam2,this), "Second parameter with setter");
	}

protected:

	virtual void initialize();

	virtual void process(const Timer& timer);

	//bool startVacuumCleanerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	//bool stopVacuumCleanerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool cleanPatternCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	ros::NodeHandle nh_;
	ros::ServiceServer clean_pattern_srv_;		/// server for activating celan pattern

private:

	void onStatusChanged(ChannelRead<BakeRVacuumCleaningModule::ModuleStatus> status);
	void onProgressChanged(ChannelRead<float> progress);

private:

	void startCleaning();

	ServiceProperty mModule;
};

///////////////////////////////////////////////////////////////////////////////

VacuumCleaningModuleInterface::VacuumCleaningModuleInterface() : Unit(Duration::milliseconds(100))
{
	// TODO: further initialization of members, etc.
}

VacuumCleaningModuleInterface::VacuumCleaningModuleInterface(ros::NodeHandle nh) : Unit(Duration::milliseconds(100)), nh_(nh)
{
	// TODO: further initialization of members, etc.
}

void VacuumCleaningModuleInterface::initialize()
{
	mModule = "BakeRVacuumCleaningModule";
	subscribe<BakeRVacuumCleaningModule::ModuleStatus>("Status", &VacuumCleaningModuleInterface::onStatusChanged);
	subscribe<float>("Progress", &VacuumCleaningModuleInterface::onProgressChanged);

	std::cout << "mModule='" << (std::string)mModule << "'      resolveName(mModule)=" << resolveName(mModule) << std::endl;
	bootup(MakeString() << "Waiting for service " << resolveName(mModule));
	bool service_available = waitForService(mModule, mira::Duration::seconds(10));
	if (service_available == false)
		std::cout << "mModule='" << (std::string)mModule << "'  resolveName(mModule)=" << resolveName(mModule) << " is not available." << std::endl;

	clean_pattern_srv_ = nh_.advertiseService("clean_pattern", &VacuumCleaningModuleInterface::cleanPatternCallback, this);

	std::cout << "VacuumCleaningModuleInterface initialized." << std::endl;
}

void VacuumCleaningModuleInterface::process(const Timer& timer)
{
	// TODO: this method is called periodically with the specified cycle time, so you can perform your computation here.
}

void VacuumCleaningModuleInterface::onStatusChanged(mira::ChannelRead<BakeRVacuumCleaningModule::ModuleStatus> status)
{
	// std::cout << "received module status: connected=" << status->connected << std::endl;
	// TODO: encode to ROS message and publish
}

void VacuumCleaningModuleInterface::onProgressChanged(mira::ChannelRead<float> progress)
{
	// TODO: encode to ROS message and publish
}


bool VacuumCleaningModuleInterface::cleanPatternCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	std::cout << "VacuumCleaningModuleInterface::cleanPatternCallback: Starting cleaner." << std::endl;
	try
	{
		std::list<mira::SignedDegree<int>> directions { mira::SignedDegree<int>(-45), mira::SignedDegree<int>(0), mira::SignedDegree<int>(45) };
		callService<void>(mModule, "cleanPattern", directions).get();
	}
	catch (XRuntime &e)
	{
		res.message = e.what();
		res.success = false;
		return false;
	}

	res.success = true;
	return true;
}

///////////////////////////////////////////////////////////////////////////////


}}

MIRA_CLASS_SERIALIZATION(mira::actors::VacuumCleaningModuleInterface, mira::Unit);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "baker_vacuum_cleaning_module_interface");
	ros::NodeHandle nh("~");

	std::string  config_file, port_number, scitos_modules;
	std::vector<std::string> args;

	if (argc < 2)
	{
		// no arguments, so use ROS parameters.
		if (ros::param::get("~config_file", config_file))
		{
			args.push_back(std::string("-c"));
			args.push_back(config_file);
		}
		else
		{
			ROS_ERROR("Can't read parameter 'config_file'");
			return 1;
		}
		if (ros::param::get("~server_port", port_number))
		{
			args.push_back(std::string("-p"));
			args.push_back(port_number);
			ROS_INFO_STREAM("Loading with MIRA multiprocess communication support on port " << port_number);
		}
		else
		{
			ROS_INFO("Not loading with MIRA multiprocess support.");
		}
	}
	else
	{
		for (int i = 1; i < argc; i++)
			args.push_back(std::string(argv[i]));
	}

	mira::Framework framework(args, true);
	ros::Duration(2).sleep();

	mira::actors::VacuumCleaningModuleInterface ifc(nh);
	ifc.checkin("/modules/vacuumcleaning", "ROSInterface");
	//ifc.addImmediateHandlerFunction(boost::bind(&VacuumCleaningModuleInterface::initialize, &ifc));
	ifc.start();

	std::cout << "VacuumCleaningModuleInterface started." << std::endl;

	while (ros::ok())
		ros::spinOnce();

	ifc.stop();

	return 0;
}
