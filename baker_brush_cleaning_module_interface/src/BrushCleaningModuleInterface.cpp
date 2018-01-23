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
 * @file BrushCleaningModuleInterface.C
 *    Interface for BakeR brush cleaning module
 *
 * @author Christof Schröter
 * @date   2018/01/23
 */

#include <fw/Unit.h>
#include <fw/ServiceProperty.h>

#include <actors/BakeRCleaningModule.h>

using namespace mira;

namespace mira { namespace actors {

///////////////////////////////////////////////////////////////////////////////

/**
 * Interface for BakeR brush cleaning module
 */
class BrushCleaningModuleInterface : public Unit
{
MIRA_OBJECT(BrushCleaningModuleInterface)


public:

	BrushCleaningModuleInterface();

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		Unit::reflect(r);

		r.property("ModuleService", mModule,
		           "Used module service", "BakerCleaningModule");

		// TODO: reflect all parameters (members and properties) that specify the persistent state of the unit
		//r.property("Param1", mParam1, "First parameter of this unit with default value", 123.4f);
		//r.member("Param2", mParam2, setter(&UnitName::setParam2,this), "Second parameter with setter");
	}

protected:

	virtual void initialize();

	virtual void process(const Timer& timer);

private:

	void onStatusChanged(ChannelRead<BakeRCleaningModule::ModuleStatus> status);

private:

	void startCleaning();

	ServiceProperty mModule;
};

///////////////////////////////////////////////////////////////////////////////

BrushCleaningModuleInterface::BrushCleaningModuleInterface() : Unit(Duration::milliseconds(100))
{
	// TODO: further initialization of members, etc.
}

void BrushCleaningModuleInterface::initialize()
{
	// TODO: call after instantiation
	subscribe<BakeRCleaningModule::ModuleStatus>("Status", &BrushCleaningModuleInterface::onStatusChanged);

	bootup(MakeString() << "Waiting for service " << resolveName(mModule));
	waitForService(mModule);
}

void BrushCleaningModuleInterface::process(const Timer& timer)
{
	// TODO: call start() after initialize()

	// TODO: this method is called periodically with the specified cycle time, so you can perform your computation here.
}

void BrushCleaningModuleInterface::onStatusChanged(ChannelRead<BakeRCleaningModule::ModuleStatus> status)
{
	std::cout << "received module status" << std::endl;
	// TODO: encode to ROS message and publish
}

void BrushCleaningModuleInterface::startCleaning()
{
	// TODO (vor initialize()): call setProperty("ModuleService", "BakeRCleaningModule")
	callService<void>(mModule, "startCleaning").get();
}

/*
 * BrushCleaningModuleInterface ifc;
 * ifc.checkin("/modules/brushcleaning"", "ROSInterface");
 * ifc.setProperty("ModuleService", "BakerCleaningModule");
 * ifc.addImmediateHandlerFunction(boost::bind(&BrushCleaningModuleInterface::initialize, this));
 * ifc.start();
 *
 */
///////////////////////////////////////////////////////////////////////////////

}}

MIRA_CLASS_SERIALIZATION(mira::actors::BrushCleaningModuleInterface, mira::Unit );
