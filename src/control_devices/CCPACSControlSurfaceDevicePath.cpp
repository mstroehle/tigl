/*
 * Copyright (C) 2007-2013 German Aerospace Center (DLR/SC)
 *
 * Created: 2014-01-28 Mark Geiger <Mark.Geiger@dlr.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <sstream>
#include <exception>

#include "CCPACSControlSurfaceDevicePath.h"

namespace tigl
{

CCPACSControlSurfaceDevicePath::CCPACSControlSurfaceDevicePath()
{
}

// Read CPACS TrailingEdgeDevicePath element
void CCPACSControlSurfaceDevicePath::ReadCPACS(
        TixiDocumentHandle tixiHandle,
        const std::string& controlSurfaceDevicePathXPath)
{

    char*       elementPath;
    std::string tempString;

    tempString = controlSurfaceDevicePathXPath + "/steps";
    elementPath = const_cast<char*>(tempString.c_str());
    if (tixiCheckElement(tixiHandle, elementPath) == SUCCESS) {
        steps.ReadCPACS(tixiHandle, elementPath);
    }

    tempString = controlSurfaceDevicePathXPath + "/innerHingePoint";
    elementPath = const_cast<char*>(tempString.c_str());
    if (tixiCheckElement(tixiHandle, elementPath) == SUCCESS) {
        innerPoint.ReadCPACS(tixiHandle, elementPath);
    }

    tempString = controlSurfaceDevicePathXPath + "/outerHingePoint";
    elementPath = const_cast<char*>(tempString.c_str());
    if (tixiCheckElement(tixiHandle, elementPath) == SUCCESS) {
        outerPoint.ReadCPACS(tixiHandle, elementPath);
    }

}

CCPACSControlSurfaceDeviceSteps CCPACSControlSurfaceDevicePath::getSteps() const
{
    return steps;
}

CCPACSControlSurfaceDevicePathHingePoint CCPACSControlSurfaceDevicePath::getInnerHingePoint() const
{
    return innerPoint;
}

CCPACSControlSurfaceDevicePathHingePoint CCPACSControlSurfaceDevicePath::getOuterHingePoint() const
{
    return outerPoint;
}

std::vector<double> CCPACSControlSurfaceDevicePath::getInnerHingeTranslationsX() const
{
    std::vector<double> tmp;
    for ( int i = 1; i <= steps.getControlSurfaceDeviceStepCount(); i++ ) {
        tmp.push_back( steps.getControlSurfaceDeviceStepByID(i).getInnerHingeTranslation().getX() );
    }
    return tmp;
}

std::vector<double> CCPACSControlSurfaceDevicePath::getInnerHingeTranslationsY() const
{
    std::vector<double> tmp;
    for ( int i = 1; i <= steps.getControlSurfaceDeviceStepCount(); i++ ) {
        tmp.push_back( steps.getControlSurfaceDeviceStepByID(i).getInnerHingeTranslation().getY() );
    }
    return tmp;
}

std::vector<double> CCPACSControlSurfaceDevicePath::getInnerHingeTranslationsZ() const
{
    std::vector<double> tmp;
    for ( int i = 1; i <= steps.getControlSurfaceDeviceStepCount(); i++ ) {
        tmp.push_back( steps.getControlSurfaceDeviceStepByID(i).getInnerHingeTranslation().getZ() );
    }
    return tmp;
}

std::vector<double> CCPACSControlSurfaceDevicePath::getOuterHingeTranslationsX() const
{
    std::vector<double> tmp;
    for ( int i = 1; i <= steps.getControlSurfaceDeviceStepCount(); i++ ) {
        tmp.push_back( steps.getControlSurfaceDeviceStepByID(i).getOuterHingeTranslation().getX() );
    }
    return tmp;
}

std::vector<double> CCPACSControlSurfaceDevicePath::getOuterHingeTranslationsZ() const
{
    std::vector<double> tmp;
    for ( int i = 1; i <= steps.getControlSurfaceDeviceStepCount(); i++ ) {
        tmp.push_back( steps.getControlSurfaceDeviceStepByID(i).getOuterHingeTranslation().getZ() );
    }
    return tmp;
}

std::vector<double> CCPACSControlSurfaceDevicePath::getRelDeflections() const
{
    std::vector<double> tmp;
    for ( int i = 1; i <= steps.getControlSurfaceDeviceStepCount(); i++ ) {
        tmp.push_back( steps.getControlSurfaceDeviceStepByID(i).getRelDeflection() );
    }
    return tmp;
}

std::vector<double> CCPACSControlSurfaceDevicePath::getHingeLineRotations() const
{
    std::vector<double> tmp;
    for ( int i = 1; i <= steps.getControlSurfaceDeviceStepCount(); i++ ) {
        tmp.push_back( steps.getControlSurfaceDeviceStepByID(i).getHingeLineRotation() );
    }
    return tmp;
}



}
// end namespace tigl