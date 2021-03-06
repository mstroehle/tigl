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

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <CCPACSPointListXY.h>
#include <CCPACSSheetList.h>
#include <string>
#include <tixi.h>
#include "tigl_internal.h"

namespace tigl
{
class CTiglUIDManager;
class CCPACSStructuralProfiles;

namespace generated
{
    // This class is used in:
    // CPACSStructuralProfiles

    // generated from /xsd:schema/xsd:complexType[844]
    class CPACSStructuralProfile
    {
    public:
        TIGL_EXPORT CPACSStructuralProfile(CCPACSStructuralProfiles* parent, CTiglUIDManager* uidMgr);

        TIGL_EXPORT virtual ~CPACSStructuralProfile();

        TIGL_EXPORT CCPACSStructuralProfiles* GetParent() const;

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::string& GetUID() const;
        TIGL_EXPORT virtual void SetUID(const std::string& value);

        TIGL_EXPORT virtual const std::string& GetName() const;
        TIGL_EXPORT virtual void SetName(const std::string& value);

        TIGL_EXPORT virtual const boost::optional<std::string>& GetDescription() const;
        TIGL_EXPORT virtual void SetDescription(const boost::optional<std::string>& value);

        TIGL_EXPORT virtual const CCPACSPointListXY& GetPointList() const;
        TIGL_EXPORT virtual CCPACSPointListXY& GetPointList();

        TIGL_EXPORT virtual const CCPACSSheetList& GetSheetList() const;
        TIGL_EXPORT virtual CCPACSSheetList& GetSheetList();

    protected:
        CCPACSStructuralProfiles* m_parent;

        CTiglUIDManager* m_uidMgr;

        std::string                  m_uID;
        std::string                  m_name;
        boost::optional<std::string> m_description;
        CCPACSPointListXY            m_pointList;
        CCPACSSheetList              m_sheetList;

    private:
#ifdef HAVE_CPP11
        CPACSStructuralProfile(const CPACSStructuralProfile&) = delete;
        CPACSStructuralProfile& operator=(const CPACSStructuralProfile&) = delete;

        CPACSStructuralProfile(CPACSStructuralProfile&&) = delete;
        CPACSStructuralProfile& operator=(CPACSStructuralProfile&&) = delete;
#else
        CPACSStructuralProfile(const CPACSStructuralProfile&);
        CPACSStructuralProfile& operator=(const CPACSStructuralProfile&);
#endif
    };
} // namespace generated

// Aliases in tigl namespace
#ifdef HAVE_CPP11
using CCPACSStructuralProfile = generated::CPACSStructuralProfile;
#else
typedef generated::CPACSStructuralProfile CCPACSStructuralProfile;
#endif
} // namespace tigl
