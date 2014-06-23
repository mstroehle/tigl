/*
 * Copyright (C) 2007-2013 German Aerospace Center (DLR/SC)
 *
 * Created: 2010-08-13 Markus Litz <Markus.Litz@dlr.de>
 * Changed: $Id$
 *
 * Version: $Revision$
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
/**
 * @file
 * @brief  Implementation of ..
 */

#ifndef CCPACSHingeTranslation_H
#define CCPACSHingeTranslation_H

#include <string>
#include <vector>

#include "tixi.h"
#include "CTiglError.h"
#include "tigl_internal.h"

namespace tigl
{

class CCPACSHingeTranslation
{

private:

    double x;
    double y;
    double z;


public:
    TIGL_EXPORT CCPACSHingeTranslation();

    TIGL_EXPORT void ReadCPACS(TixiDocumentHandle tixiHandle,
            const std::string & HingeTranslationXPath);

    TIGL_EXPORT double getX();
    TIGL_EXPORT double getY();
    TIGL_EXPORT double getZ();

};

} // end namespace tigl

#endif // CCPACSHingeTranslation_H
