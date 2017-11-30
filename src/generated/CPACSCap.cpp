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

#include "CPACSCap.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "TixiHelper.h"

namespace tigl
{
    namespace generated
    {
        CPACSCap::CPACSCap() :
            m_area(0) {}
        
        CPACSCap::~CPACSCap() {}
        
        void CPACSCap::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
        {
            // read element area
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/area")) {
                m_area = tixi::TixiGetElement<double>(tixiHandle, xpath + "/area");
            }
            else {
                LOG(ERROR) << "Required element area is missing at xpath " << xpath;
            }
            
            // read element material
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/material")) {
                m_material.ReadCPACS(tixiHandle, xpath + "/material");
            }
            else {
                LOG(ERROR) << "Required element material is missing at xpath " << xpath;
            }
            
        }
        
        void CPACSCap::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
        {
            // write element area
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/area");
            tixi::TixiSaveElement(tixiHandle, xpath + "/area", m_area);
            
            // write element material
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/material");
            m_material.WriteCPACS(tixiHandle, xpath + "/material");
            
        }
        
        const double& CPACSCap::GetArea() const
        {
            return m_area;
        }
        
        void CPACSCap::SetArea(const double& value)
        {
            m_area = value;
        }
        
        const CCPACSMaterialDefinition& CPACSCap::GetMaterial() const
        {
            return m_material;
        }
        
        CCPACSMaterialDefinition& CPACSCap::GetMaterial()
        {
            return m_material;
        }
        
    }
}
