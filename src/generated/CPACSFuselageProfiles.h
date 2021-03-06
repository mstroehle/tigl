// Copyright (c) 2018 RISC Software GmbH
//
// This file was generated by CPACSGen from CPACS XML Schema (c) German Aerospace Center (DLR/SC).
// Do not edit, all changes are lost when files are re-generated.
//
// Licensed under the Apache License, Version 2.0 (the "License")
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <tixi.h>
#include <vector>
#include "tigl_internal.h"
#include "UniquePtr.h"

namespace tigl
{
class CTiglUIDManager;

namespace generated
{
    class CPACSProfileGeometry;

    // This class is used in:
    // CPACSProfiles

    // generated from /xsd:schema/xsd:complexType[375]
    class CPACSFuselageProfiles
    {
    public:
        TIGL_EXPORT CPACSFuselageProfiles(CTiglUIDManager* uidMgr);
        TIGL_EXPORT virtual ~CPACSFuselageProfiles();

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::vector<unique_ptr<CPACSProfileGeometry> >& GetFuselageProfiles() const;
        TIGL_EXPORT virtual std::vector<unique_ptr<CPACSProfileGeometry> >& GetFuselageProfiles();

        TIGL_EXPORT virtual CPACSProfileGeometry& AddFuselageProfile();
        TIGL_EXPORT virtual void RemoveFuselageProfile(CPACSProfileGeometry& ref);

    protected:
        CTiglUIDManager* m_uidMgr;

        std::vector<unique_ptr<CPACSProfileGeometry> > m_fuselageProfiles;

    private:
#ifdef HAVE_CPP11
        CPACSFuselageProfiles(const CPACSFuselageProfiles&) = delete;
        CPACSFuselageProfiles& operator=(const CPACSFuselageProfiles&) = delete;

        CPACSFuselageProfiles(CPACSFuselageProfiles&&) = delete;
        CPACSFuselageProfiles& operator=(CPACSFuselageProfiles&&) = delete;
#else
        CPACSFuselageProfiles(const CPACSFuselageProfiles&);
        CPACSFuselageProfiles& operator=(const CPACSFuselageProfiles&);
#endif
    };
} // namespace generated

// CPACSFuselageProfiles is customized, use type CCPACSFuselageProfiles directly

// Aliases in tigl namespace
#ifdef HAVE_CPP11
using CCPACSProfileGeometry = generated::CPACSProfileGeometry;
#else
typedef generated::CPACSProfileGeometry CCPACSProfileGeometry;
#endif
} // namespace tigl
