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
    class CPACSRivet;

    // This class is used in:
    // CPACSStructuralElements

    // generated from /xsd:schema/xsd:complexType[762]
    class CPACSRivets
    {
    public:
        TIGL_EXPORT CPACSRivets(CTiglUIDManager* uidMgr);
        TIGL_EXPORT virtual ~CPACSRivets();

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::vector<unique_ptr<CPACSRivet> >& GetRivets() const;
        TIGL_EXPORT virtual std::vector<unique_ptr<CPACSRivet> >& GetRivets();

        TIGL_EXPORT virtual CPACSRivet& AddRivet();
        TIGL_EXPORT virtual void RemoveRivet(CPACSRivet& ref);

    protected:
        CTiglUIDManager* m_uidMgr;

        std::vector<unique_ptr<CPACSRivet> > m_rivets;

    private:
#ifdef HAVE_CPP11
        CPACSRivets(const CPACSRivets&) = delete;
        CPACSRivets& operator=(const CPACSRivets&) = delete;

        CPACSRivets(CPACSRivets&&) = delete;
        CPACSRivets& operator=(CPACSRivets&&) = delete;
#else
        CPACSRivets(const CPACSRivets&);
        CPACSRivets& operator=(const CPACSRivets&);
#endif
    };
} // namespace generated

// Aliases in tigl namespace
#ifdef HAVE_CPP11
using CCPACSRivets = generated::CPACSRivets;
using CCPACSRivet = generated::CPACSRivet;
#else
typedef generated::CPACSRivets CCPACSRivets;
typedef generated::CPACSRivet CCPACSRivet;
#endif
} // namespace tigl
