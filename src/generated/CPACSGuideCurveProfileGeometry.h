// Copyright (c) 2016 RISC Software GmbH
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

#include <tixi.h>
#include <string>
#include <boost/optional.hpp>
#include "tigl_internal.h"
#include <TiglSymmetryAxis.h>
#include <CCPACSPointListRelXYZ.h>

namespace tigl
{
    namespace generated
    {
        // This class is used in:
        // CPACSGuideCurveProfiles
        
        // generated from /xsd:schema/xsd:complexType[419]
        class CPACSGuideCurveProfileGeometry
        {
        public:
            TIGL_EXPORT CPACSGuideCurveProfileGeometry();
            TIGL_EXPORT virtual ~CPACSGuideCurveProfileGeometry();
            
            TIGL_EXPORT virtual void ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath);
            TIGL_EXPORT virtual void WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const;
            
            TIGL_EXPORT bool HasSymmetry() const;
            TIGL_EXPORT const TiglSymmetryAxis& GetSymmetry() const;
            TIGL_EXPORT void SetSymmetry(const TiglSymmetryAxis& value);
            
            TIGL_EXPORT const std::string& GetUID() const;
            TIGL_EXPORT void SetUID(const std::string& value);
            
            TIGL_EXPORT const std::string& GetName() const;
            TIGL_EXPORT void SetName(const std::string& value);
            
            TIGL_EXPORT bool HasDescription() const;
            TIGL_EXPORT const std::string& GetDescription() const;
            TIGL_EXPORT void SetDescription(const std::string& value);
            
            TIGL_EXPORT const CCPACSPointListRelXYZ& GetPointList() const;
            TIGL_EXPORT CCPACSPointListRelXYZ& GetPointList();
            
        protected:
            boost::optional<TiglSymmetryAxis> m_symmetry;
            std::string                       m_uID;
            std::string                       m_name;
            boost::optional<std::string>      m_description;
            CCPACSPointListRelXYZ             m_pointList;
            
        private:
            #ifdef HAVE_CPP11
            CPACSGuideCurveProfileGeometry(const CPACSGuideCurveProfileGeometry&) = delete;
            CPACSGuideCurveProfileGeometry& operator=(const CPACSGuideCurveProfileGeometry&) = delete;
            
            CPACSGuideCurveProfileGeometry(CPACSGuideCurveProfileGeometry&&) = delete;
            CPACSGuideCurveProfileGeometry& operator=(CPACSGuideCurveProfileGeometry&&) = delete;
            #else
            CPACSGuideCurveProfileGeometry(const CPACSGuideCurveProfileGeometry&);
            CPACSGuideCurveProfileGeometry& operator=(const CPACSGuideCurveProfileGeometry&);
            #endif
        };
    }
    
    // This type is customized, use type CCPACSGuideCurveProfile
}