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
#include <cctype>

#include "CTiglError.h"
#include "to_string.h"

namespace tigl
{
namespace generated
{
    // This enum is used in:
    // CPACSProfileBasedStructuralElement

    // generated from /xsd:schema/xsd:complexType[732]/xsd:complexContent/xsd:extension/xsd:sequence/xsd:choice[1]/xsd:sequence[2]/xsd:element[1]/xsd:complexType/xsd:simpleContent
    enum CPACSProfileBasedStructuralElement_standardProfileType
    {
        C,
        T,
        Z,
        L,
        HAT,
        ROD,
        TUBE,
        BAR,
        BOX
    };

    inline std::string CPACSProfileBasedStructuralElement_standardProfileTypeToString(const CPACSProfileBasedStructuralElement_standardProfileType& value)
    {
        switch(value) {
        case C: return "C";
        case T: return "T";
        case Z: return "Z";
        case L: return "L";
        case HAT: return "HAT";
        case ROD: return "ROD";
        case TUBE: return "TUBE";
        case BAR: return "BAR";
        case BOX: return "BOX";
        default: throw CTiglError("Invalid enum value \"" + std_to_string(static_cast<int>(value)) + "\" for enum type CPACSProfileBasedStructuralElement_standardProfileType");
        }
    }
    inline CPACSProfileBasedStructuralElement_standardProfileType stringToCPACSProfileBasedStructuralElement_standardProfileType(const std::string& value)
    {
        struct ToLower { std::string operator()(std::string str) { for (std::size_t i = 0; i < str.length(); i++) { str[i] = std::tolower(str[i]); } return str; } } toLower;
        if (toLower(value) == "c") { return C; }
        if (toLower(value) == "t") { return T; }
        if (toLower(value) == "z") { return Z; }
        if (toLower(value) == "l") { return L; }
        if (toLower(value) == "hat") { return HAT; }
        if (toLower(value) == "rod") { return ROD; }
        if (toLower(value) == "tube") { return TUBE; }
        if (toLower(value) == "bar") { return BAR; }
        if (toLower(value) == "box") { return BOX; }
        throw CTiglError("Invalid string value \"" + value + "\" for enum type CPACSProfileBasedStructuralElement_standardProfileType");
    }
} // namespace generated

// Aliases in tigl namespace
#ifdef HAVE_CPP11
using ECPACSProfileBasedStructuralElement_standardProfileType = generated::CPACSProfileBasedStructuralElement_standardProfileType;
#else
typedef generated::CPACSProfileBasedStructuralElement_standardProfileType ECPACSProfileBasedStructuralElement_standardProfileType;
#endif
using generated::C;
using generated::T;
using generated::Z;
using generated::L;
using generated::HAT;
using generated::ROD;
using generated::TUBE;
using generated::BAR;
using generated::BOX;
} // namespace tigl
