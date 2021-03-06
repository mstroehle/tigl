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
    class CPACSSheet3D;

    // This class is used in:
    // CPACSStructuralProfile3D

    // generated from /xsd:schema/xsd:complexType[798]
    class CPACSSheetList3D
    {
    public:
        TIGL_EXPORT CPACSSheetList3D(CTiglUIDManager* uidMgr);
        TIGL_EXPORT virtual ~CPACSSheetList3D();

        TIGL_EXPORT CTiglUIDManager& GetUIDManager();
        TIGL_EXPORT const CTiglUIDManager& GetUIDManager() const;

        TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
        TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;

        TIGL_EXPORT virtual const std::vector<unique_ptr<CPACSSheet3D> >& GetSheet3Ds() const;
        TIGL_EXPORT virtual std::vector<unique_ptr<CPACSSheet3D> >& GetSheet3Ds();

        TIGL_EXPORT virtual CPACSSheet3D& AddSheet3D();
        TIGL_EXPORT virtual void RemoveSheet3D(CPACSSheet3D& ref);

    protected:
        CTiglUIDManager* m_uidMgr;

        std::vector<unique_ptr<CPACSSheet3D> > m_sheet3Ds;

    private:
#ifdef HAVE_CPP11
        CPACSSheetList3D(const CPACSSheetList3D&) = delete;
        CPACSSheetList3D& operator=(const CPACSSheetList3D&) = delete;

        CPACSSheetList3D(CPACSSheetList3D&&) = delete;
        CPACSSheetList3D& operator=(CPACSSheetList3D&&) = delete;
#else
        CPACSSheetList3D(const CPACSSheetList3D&);
        CPACSSheetList3D& operator=(const CPACSSheetList3D&);
#endif
    };
} // namespace generated

// Aliases in tigl namespace
#ifdef HAVE_CPP11
using CCPACSSheetList3D = generated::CPACSSheetList3D;
using CCPACSSheet3D = generated::CPACSSheet3D;
#else
typedef generated::CPACSSheetList3D CCPACSSheetList3D;
typedef generated::CPACSSheet3D CCPACSSheet3D;
#endif
} // namespace tigl
