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

#include <cassert>
#include "CCPACSRotorBlades.h"
#include "CCPACSWings.h"
#include "CPACSWing.h"
#include "CTiglError.h"
#include "CTiglLogging.h"
#include "CTiglUIDManager.h"
#include "TixiHelper.h"

namespace tigl
{
    namespace generated
    {
        CPACSWing::CPACSWing(CCPACSRotorBlades* parent, CTiglUIDManager* uidMgr) :
            m_uidMgr(uidMgr), 
            m_transformation(m_uidMgr), 
            m_sections(reinterpret_cast<CCPACSWing*>(this), m_uidMgr), 
            m_segments(reinterpret_cast<CCPACSWing*>(this), m_uidMgr)
        {
            //assert(parent != NULL);
            m_parent = parent;
            m_parentType = &typeid(CCPACSRotorBlades);
        }
        
        CPACSWing::CPACSWing(CCPACSWings* parent, CTiglUIDManager* uidMgr) :
            m_uidMgr(uidMgr), 
            m_transformation(m_uidMgr), 
            m_sections(reinterpret_cast<CCPACSWing*>(this), m_uidMgr), 
            m_segments(reinterpret_cast<CCPACSWing*>(this), m_uidMgr)
        {
            //assert(parent != NULL);
            m_parent = parent;
            m_parentType = &typeid(CCPACSWings);
        }
        
        CPACSWing::~CPACSWing()
        {
            if (m_uidMgr) m_uidMgr->TryUnregisterObject(m_uID);
        }
        
        CTiglUIDManager& CPACSWing::GetUIDManager()
        {
            return *m_uidMgr;
        }
        
        const CTiglUIDManager& CPACSWing::GetUIDManager() const
        {
            return *m_uidMgr;
        }
        
        void CPACSWing::ReadCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath)
        {
            // read attribute uID
            if (tixi::TixiCheckAttribute(tixiHandle, xpath, "uID")) {
                m_uID = tixi::TixiGetAttribute<std::string>(tixiHandle, xpath, "uID");
                if (m_uID.empty()) {
                    LOG(WARNING) << "Required attribute uID is empty at xpath " << xpath;
                }
            }
            else {
                LOG(ERROR) << "Required attribute uID is missing at xpath " << xpath;
            }
            
            // read attribute symmetry
            if (tixi::TixiCheckAttribute(tixiHandle, xpath, "symmetry")) {
                m_symmetry = stringToTiglSymmetryAxis(tixi::TixiGetAttribute<std::string>(tixiHandle, xpath, "symmetry"));
            }
            
            // read element name
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/name")) {
                m_name = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/name");
                if (m_name.empty()) {
                    LOG(WARNING) << "Required element name is empty at xpath " << xpath;
                }
            }
            else {
                LOG(ERROR) << "Required element name is missing at xpath " << xpath;
            }
            
            // read element description
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/description")) {
                m_description = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/description");
                if (m_description->empty()) {
                    LOG(WARNING) << "Optional element description is present but empty at xpath " << xpath;
                }
            }
            
            // read element parentUID
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/parentUID")) {
                m_parentUID = tixi::TixiGetElement<std::string>(tixiHandle, xpath + "/parentUID");
                if (m_parentUID->empty()) {
                    LOG(WARNING) << "Optional element parentUID is present but empty at xpath " << xpath;
                }
            }
            
            // read element transformation
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/transformation")) {
                m_transformation.ReadCPACS(tixiHandle, xpath + "/transformation");
            }
            else {
                LOG(ERROR) << "Required element transformation is missing at xpath " << xpath;
            }
            
            // read element sections
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/sections")) {
                m_sections.ReadCPACS(tixiHandle, xpath + "/sections");
            }
            else {
                LOG(ERROR) << "Required element sections is missing at xpath " << xpath;
            }
            
            // read element positionings
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/positionings")) {
                m_positionings = boost::in_place(m_uidMgr);
                try {
                    m_positionings->ReadCPACS(tixiHandle, xpath + "/positionings");
                } catch(const std::exception& e) {
                    LOG(ERROR) << "Failed to read positionings at xpath " << xpath << ": " << e.what();
                    m_positionings = boost::none;
                }
            }
            
            // read element segments
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/segments")) {
                m_segments.ReadCPACS(tixiHandle, xpath + "/segments");
            }
            else {
                LOG(ERROR) << "Required element segments is missing at xpath " << xpath;
            }
            
            // read element componentSegments
            if (tixi::TixiCheckElement(tixiHandle, xpath + "/componentSegments")) {
                m_componentSegments = boost::in_place(reinterpret_cast<CCPACSWing*>(this), m_uidMgr);
                try {
                    m_componentSegments->ReadCPACS(tixiHandle, xpath + "/componentSegments");
                } catch(const std::exception& e) {
                    LOG(ERROR) << "Failed to read componentSegments at xpath " << xpath << ": " << e.what();
                    m_componentSegments = boost::none;
                }
            }
            
            if (m_uidMgr) m_uidMgr->RegisterObject(m_uID, *this);
        }
        
        void CPACSWing::WriteCPACS(const TixiDocumentHandle& tixiHandle, const std::string& xpath) const
        {
            // write attribute uID
            tixi::TixiSaveAttribute(tixiHandle, xpath, "uID", m_uID);
            
            // write attribute symmetry
            if (m_symmetry) {
                tixi::TixiSaveAttribute(tixiHandle, xpath, "symmetry", TiglSymmetryAxisToString(*m_symmetry));
            } else {
                if (tixi::TixiCheckAttribute(tixiHandle, xpath, "symmetry")) {
                    tixi::TixiRemoveAttribute(tixiHandle, xpath, "symmetry");
                }
            }
            
            // write element name
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/name");
            tixi::TixiSaveElement(tixiHandle, xpath + "/name", m_name);
            
            // write element description
            if (m_description) {
                tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/description");
                tixi::TixiSaveElement(tixiHandle, xpath + "/description", *m_description);
            } else {
                if (tixi::TixiCheckElement(tixiHandle, xpath + "/description")) {
                    tixi::TixiRemoveElement(tixiHandle, xpath + "/description");
                }
            }
            
            // write element parentUID
            if (m_parentUID) {
                tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/parentUID");
                tixi::TixiSaveElement(tixiHandle, xpath + "/parentUID", *m_parentUID);
            } else {
                if (tixi::TixiCheckElement(tixiHandle, xpath + "/parentUID")) {
                    tixi::TixiRemoveElement(tixiHandle, xpath + "/parentUID");
                }
            }
            
            // write element transformation
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/transformation");
            m_transformation.WriteCPACS(tixiHandle, xpath + "/transformation");
            
            // write element sections
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/sections");
            m_sections.WriteCPACS(tixiHandle, xpath + "/sections");
            
            // write element positionings
            if (m_positionings) {
                tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/positionings");
                m_positionings->WriteCPACS(tixiHandle, xpath + "/positionings");
            } else {
                if (tixi::TixiCheckElement(tixiHandle, xpath + "/positionings")) {
                    tixi::TixiRemoveElement(tixiHandle, xpath + "/positionings");
                }
            }
            
            // write element segments
            tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/segments");
            m_segments.WriteCPACS(tixiHandle, xpath + "/segments");
            
            // write element componentSegments
            if (m_componentSegments) {
                tixi::TixiCreateElementIfNotExists(tixiHandle, xpath + "/componentSegments");
                m_componentSegments->WriteCPACS(tixiHandle, xpath + "/componentSegments");
            } else {
                if (tixi::TixiCheckElement(tixiHandle, xpath + "/componentSegments")) {
                    tixi::TixiRemoveElement(tixiHandle, xpath + "/componentSegments");
                }
            }
            
        }
        
        const std::string& CPACSWing::GetUID() const
        {
            return m_uID;
        }
        
        void CPACSWing::SetUID(const std::string& value)
        {
            if (m_uidMgr) {
                m_uidMgr->TryUnregisterObject(m_uID);
                m_uidMgr->RegisterObject(value, *this);
            }
            m_uID = value;
        }
        
        const boost::optional<TiglSymmetryAxis>& CPACSWing::GetSymmetry() const
        {
            return m_symmetry;
        }
        
        void CPACSWing::SetSymmetry(const TiglSymmetryAxis& value)
        {
            m_symmetry = value;
        }
        
        void CPACSWing::SetSymmetry(const boost::optional<TiglSymmetryAxis>& value)
        {
            m_symmetry = value;
        }
        
        const std::string& CPACSWing::GetName() const
        {
            return m_name;
        }
        
        void CPACSWing::SetName(const std::string& value)
        {
            m_name = value;
        }
        
        const boost::optional<std::string>& CPACSWing::GetDescription() const
        {
            return m_description;
        }
        
        void CPACSWing::SetDescription(const std::string& value)
        {
            m_description = value;
        }
        
        void CPACSWing::SetDescription(const boost::optional<std::string>& value)
        {
            m_description = value;
        }
        
        const boost::optional<std::string>& CPACSWing::GetParentUID() const
        {
            return m_parentUID;
        }
        
        void CPACSWing::SetParentUID(const std::string& value)
        {
            m_parentUID = value;
        }
        
        void CPACSWing::SetParentUID(const boost::optional<std::string>& value)
        {
            m_parentUID = value;
        }
        
        const CCPACSTransformation& CPACSWing::GetTransformation() const
        {
            return m_transformation;
        }
        
        CCPACSTransformation& CPACSWing::GetTransformation()
        {
            return m_transformation;
        }
        
        const CCPACSWingSections& CPACSWing::GetSections() const
        {
            return m_sections;
        }
        
        CCPACSWingSections& CPACSWing::GetSections()
        {
            return m_sections;
        }
        
        const boost::optional<CCPACSPositionings>& CPACSWing::GetPositionings() const
        {
            return m_positionings;
        }
        
        boost::optional<CCPACSPositionings>& CPACSWing::GetPositionings()
        {
            return m_positionings;
        }
        
        const CCPACSWingSegments& CPACSWing::GetSegments() const
        {
            return m_segments;
        }
        
        CCPACSWingSegments& CPACSWing::GetSegments()
        {
            return m_segments;
        }
        
        const boost::optional<CCPACSWingComponentSegments>& CPACSWing::GetComponentSegments() const
        {
            return m_componentSegments;
        }
        
        boost::optional<CCPACSWingComponentSegments>& CPACSWing::GetComponentSegments()
        {
            return m_componentSegments;
        }
        
        CCPACSPositionings& CPACSWing::GetPositionings(CreateIfNotExistsTag)
        {
            if (!m_positionings)
                m_positionings = boost::in_place(m_uidMgr);
            return *m_positionings;
        }
        
        void CPACSWing::RemovePositionings()
        {
            m_positionings = boost::none;
        }
        
        CCPACSWingComponentSegments& CPACSWing::GetComponentSegments(CreateIfNotExistsTag)
        {
            if (!m_componentSegments)
                m_componentSegments = boost::in_place(reinterpret_cast<CCPACSWing*>(this), m_uidMgr);
            return *m_componentSegments;
        }
        
        void CPACSWing::RemoveComponentSegments()
        {
            m_componentSegments = boost::none;
        }
        
    }
}
