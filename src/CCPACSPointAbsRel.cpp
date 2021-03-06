/*
* Copyright (c) 2018 RISC Software GmbH
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


#include "CCPACSPointAbsRel.h"

namespace tigl
{
ECPACSTranslationType CCPACSPointAbsRel::defaultTranslationType = ABS_LOCAL;

CCPACSPointAbsRel::CCPACSPointAbsRel(CTiglUIDManager* uidMgr)
    : generated::CPACSPointAbsRel(uidMgr)
{
}

CTiglPoint CCPACSPointAbsRel::AsPoint() const
{
    CTiglPoint point;
    point.x = m_x ? *m_x : 0;
    point.y = m_y ? *m_y : 0;
    point.z = m_z ? *m_z : 0;

    return point;
}

void CCPACSPointAbsRel::SetAsPoint(const CTiglPoint& point)
{
    m_x = point.x;
    m_y = point.y;
    m_z = point.z;
}

ECPACSTranslationType CCPACSPointAbsRel::GetRefDefaultedType() const
{
    // TODO: CPACSGen does not yet support optional attributes with default values, remove this code and return m_ref when defaults become available
    if (m_refType) {
        return *m_refType;
    }
    else {
        return defaultTranslationType;
    }
}

} // namespace tigl
