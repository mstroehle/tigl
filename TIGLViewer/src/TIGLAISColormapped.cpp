// Copyright (c) 1999-2012 OPEN CASCADE SAS
//
// The content of this file is subject to the Open CASCADE Technology Public
// License Version 6.5 (the "License"). You may not use the content of this file
// except in compliance with the License. Please obtain a copy of the License
// at http://www.opencascade.org and read it completely before using this file.
//
// The Initial Developer of the Original Code is Open CASCADE S.A.S., having its
// main offices at: 1, place des Freres Montgolfier, 78280 Guyancourt, France.
//
// The Original Code and all software distributed under the License is
// distributed on an "AS IS" basis, without warranty of any kind, and the
// Initial Developer hereby disclaims all such warranties, including without
// limitation, any warranties of merchantability, fitness for a particular
// purpose or non-infringement. Please see the License for the specific terms
// and conditions governing the rights and limitations under the License.

#include "tigl_internal.h"

#include <TIGLAISColormapped.h>
#include <AIS_InteractiveObject.hxx>
#include <Standard_DefineHandle.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <Poly_Triangulation.hxx>
#include "CTiglTriangularizer.h"
#include <Prs3d_Root.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <TShort_Array1OfShortReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_HArray1OfInteger.hxx>
#include <TShort_HArray1OfShortReal.hxx>
#include <Graphic3d_Group.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>
#include <Graphic3d_ArrayOfTriangles.hxx>
#include <Graphic3d_ArrayOfPolylines.hxx>
#include <Select3D_SensitiveBox.hxx>
#include <Select3D_SensitiveTriangulation.hxx>
#include <SelectMgr_Selection.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <Graphic3d_ArrayOfSegments.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Graphic3d_AspectLine3d.hxx>
#include <Prs3d_Presentation.hxx>
#include <Standard_Version.hxx>

#include <GeomLProp_SLProps.hxx>
#include <BRepTools.hxx>
#include "BRepMesh_IncrementalMesh.hxx"

#if OCC_VERSION_HEX < VERSION_HEX_CODE(6,9,0)
#include <AIS_Drawer.hxx>
#endif

IMPLEMENT_STANDARD_HANDLE(TIGLAISColormapped, AIS_InteractiveObject)
IMPLEMENT_STANDARD_RTTIEXT(TIGLAISColormapped, AIS_InteractiveObject)

namespace 
{
    inline double max(double a, double b) 
    {
        return a>b? a : b;
    }
    
    inline double min(double a, double b) 
    {
        return a<b? a : b;
    }
}

TIGLAISColormapped::TIGLAISColormapped(const TopoDS_Shape& Shape)
{
    myShape = Shape;
    myFlagColor     = 0;
    minx = DBL_MAX; 
    miny = DBL_MAX; 
    minz = DBL_MAX;
    maxx = DBL_MIN; 
    maxy = DBL_MIN; 
    maxz = DBL_MIN;
    
}

//=======================================================================
//function : Compute
//purpose  :
//=======================================================================
void TIGLAISColormapped::Compute(const Handle(PrsMgr_PresentationManager3d)& aPresentationManager,
                                   const Handle(Prs3d_Presentation)& aPresentation,
                                   const Standard_Integer aMode)
{

    std::cout << "Calling Compute function"<<std::endl;
    std::cout << "aMode = "<<aMode<<std::endl;

    aPresentation->Clear();
    Handle(Graphic3d_Group) TheGroup = Prs3d_Root::CurrentGroup(aPresentation);
    TheGroup->Clear();

    switch (aMode) {
        case 0:
        case 1: {

            Handle(Graphic3d_AspectFillArea3d) aspect = new Graphic3d_AspectFillArea3d();
            aspect->SetInteriorStyle(Aspect_IS_SOLID);

            Standard_Real ambient = aspect->FrontMaterial().Ambient();

            TheGroup->SetPrimitivesAspect(aspect);

            std::cout<<"Triangulation of shape..."<<std::flush;
            tigl::CTiglTriangularizer t(myShape,1e-3);
            std::cout<<"[OK]"<<std::endl;


            TopExp_Explorer faceExplorer;
            for (faceExplorer.Init(myShape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {

                std::cout<<"   Getting the triangulation of face..."<<std::flush;

                //Get the triangulation of each face
                TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
                TopLoc_Location loc;
                Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

                const TColgp_Array1OfPnt& Nodes = triangulation->Nodes();
                const TColgp_Array1OfPnt2d& UVNodes = triangulation->UVNodes();
                const Poly_Array1OfTriangle& triangles = triangulation->Triangles();

                Standard_Boolean hasVNormals = triangulation->HasNormals();
                Standard_Boolean hasVColors  = false;//(myFlagColor == 1);

                std::cout<<"[OK]"<<std::endl;
                std::cout<<"   Getting the surface properties of the face..."<<std::flush;

                // get the surface properties. These are used to define the vertex colors
                Standard_Real fUMin, fUMax, fVMin, fVMax;
                BRepTools::UVBounds(TopoDS::Face(face), fUMin, fUMax, fVMin, fVMax);

                Standard_Real fU = (fUMax-fUMin)/2.0;
                Standard_Real fV = (fVMax-fVMin)/2.0;
                Handle(Geom_Surface) surface = BRep_Tool::Surface(face,loc);
//                GeomLProp_SLProps aSLProps(surface,2,1e-8);
                GeomLProp_SLProps aSLProps(surface,fU, fV, 1, Precision::Confusion());

                std::cout<<"[OK]"<<std::endl;
                std::cout<<"   Creating an array of triangles..."<<std::flush;

                // create a new array of triangles
                Handle(Graphic3d_ArrayOfTriangles) anArray =
                    new Graphic3d_ArrayOfTriangles ( triangulation->NbNodes(),        //maxVertexs
                            triangulation->NbTriangles() * 3,//maxEdges
                            hasVNormals,      //hasVNormals
                            hasVColors,       //hasVColors
                            Standard_False    //hasTexels
                            );

                Handle(Graphic3d_ArrayOfPolylines) polylines =
                    new Graphic3d_ArrayOfPolylines ( triangles.Length()* 6,
                                                     triangles.Length()* 6 );
                std::cout<<"[OK]"<<std::endl;

#if 0
                std::cout<<"   Going through all triangles"<<std::endl;
                for (Standard_Integer i = 1; i< triangles.Length(); i++)
                {
                    Poly_Triangle triangle = triangles.Value(i);
                    int n1,n2,n3;
                    triangle.Get(n1,n2,n3);
                    gp_Pnt p1 = Nodes.Value(n1);
                    gp_Pnt p2 = Nodes.Value(n2);
                    gp_Pnt p3 = Nodes.Value(n3);

                    // edge1
                    polylines->AddBound ( 2 );
                    polylines->AddVertex ( p1.X(), p1.Y(), p1.Z() );
                    polylines->AddVertex ( p2.X(), p2.Y(), p2.Z() );

                    // edge1
                    polylines->AddBound ( 2 );
                    polylines->AddVertex ( p2.X(), p2.Y(), p2.Z() );
                    polylines->AddVertex ( p3.X(), p3.Y(), p3.Z() );

                    // edge1
                    polylines->AddBound ( 2 );
                    polylines->AddVertex ( p3.X(), p3.Y(), p3.Z() );
                    polylines->AddVertex ( p1.X(), p1.Y(), p1.Z() );

                    for (Standard_Integer k = 1; k <=3; k++)
                    {
                        int vertexIdx = triangle.Value(k);
                        std::cout<<"      vertex "<<vertexIdx<<std::flush;
                        if ( k==1 ) {
                            aSLProps.SetParameters(UVNodes(n1).X(), UVNodes(n1).Y());
                            std::cout<<", U = "<<UVNodes(n1).X()<<", V = "<<UVNodes(n1).Y()<<std::flush;
                        }
                        else if ( k==2 )
                        {
                            aSLProps.SetParameters(UVNodes(n2).X(), UVNodes(n2).Y());
                            std::cout<<", U = "<<UVNodes(n2).X()<<", V = "<<UVNodes(n2).Y()<<std::flush;
                        }
                        else
                        {
                            aSLProps.SetParameters(UVNodes(n3).X(), UVNodes(n3).Y());
                            std::cout<<", U = "<<UVNodes(n3).X()<<", V = "<<UVNodes(n3).Y()<<std::flush;
                        }
                        if( aSLProps.IsCurvatureDefined() )
                        {
                            // get the curvature of the vertices, e.g.
                            //      aSLProps.MinCurvature();
                            //      aSLProps.MaxCurvature();
                            //      aSLProps.MeanCurvature();
                            //      aSLProps.GaussianCurvature();
                            Standard_Real value = aSLProps.MaxCurvature();


                            std::cout<<", Gaussian Curvature = "<<value<<std::endl;

                            gp_Pnt vertex = Nodes.Value(vertexIdx);
                            anArray->AddVertex(vertex, Quantity_NOC_BLUEVIOLET);
                            TheGroup->Marker(Graphic3d_Vertex(vertex.X(),vertex.Y(),vertex.Z()));
                        }
                    }
                }

#else
                std::cout<<"   Goint through all vertices"<<std::endl;
                for (Standard_Integer i=UVNodes.Lower(); i<=UVNodes.Upper(); i++) {

                    std::cout<<"      vertex "<<i<<std::flush;

                    // get the curvature of the vertices, e.g.
                    //      aSLProps.MinCurvature();
                    //      aSLProps.MaxCurvature();
                    //      aSLProps.MeanCurvature();
                    //      aSLProps.GaussianCurvature();
                    aSLProps.SetParameters(UVNodes(i).X(),UVNodes(i).Y());
                    if( !aSLProps.IsCurvatureDefined() ){
                        std::cout<<"Aw fuck it..."<<std::endl;
                    }
                    Standard_Real value = aSLProps.MaxCurvature();

                    std::cout<<", U = "<<UVNodes(i).X()<<", V = "<<UVNodes(i).Y()<<std::flush;
                    std::cout<<", Gaussian Curvature = "<<value<<std::endl;

                    anArray->AddVertex(Nodes(i), Quantity_NOC_BLUEVIOLET);
                }
#endif

                TheGroup->AddPrimitiveArray(anArray);

//                // draw mesh edges
//                TheGroup->SetPrimitivesAspect ( new Graphic3d_AspectLine3d( Quantity_NameOfColor::Quantity_NOC_BLACK , Aspect_TOL_SOLID, 0.1 ) );
//                TheGroup->AddPrimitiveArray ( polylines );
            }
        }
        break;
     }
}

////=======================================================================
////function : ComputeSelection
////purpose  :
////=======================================================================
void TIGLAISColormapped::ComputeSelection(const Handle(SelectMgr_Selection)& aSelection,
                                            const Standard_Integer /*aMode*/)
{
//    Handle(SelectMgr_EntityOwner) eown = new SelectMgr_EntityOwner(this,0);

//    TopLoc_Location loc;
//    Handle(Select3D_SensitiveTriangulation) aSensitiveTria = new Select3D_SensitiveTriangulation(eown, myShape, loc);
//    aSelection->Add(aSensitiveTria);
}

//=======================================================================
//function : SetColor
//purpose  : Set the color for each node.
//           Each 32-bit color is Alpha << 24 + Blue << 16 + Green << 8 + Red
//           Order of color components is essential for further usage by OpenGL
//=======================================================================
void TIGLAISColormapped::SetColors(const Handle(TColStd_HArray1OfInteger)& aColor)
{
    myFlagColor = 1;
    myColor = aColor;
}

//=======================================================================
//function : GetColor
//purpose  : Get the color for each node.
//           Each 32-bit color is Alpha << 24 + Blue << 16 + Green << 8 + Red
//           Order of color components is essential for further usage by OpenGL
//=======================================================================

Handle(TColStd_HArray1OfInteger) TIGLAISColormapped::GetColors() const
{
    return myColor;
}


//=======================================================================
//function : SetShape
//purpose  : 
//=======================================================================
void TIGLAISColormapped::SetShape(const TopoDS_Shape& aShape)
{
    myShape = aShape;
}

//=======================================================================
//function : GetShape
//purpose  : 
//=======================================================================
TopoDS_Shape TIGLAISColormapped::GetShape() const
{
    return myShape;
}

//=======================================================================
//function : AttenuateColor
//purpose  : Attenuates 32-bit color by a given attenuation factor (0...1):
//           aColor = Alpha << 24 + Blue << 16 + Green << 8 + Red
//           All color components are multiplied by aComponent, the result is then packed again as 32-bit integer.
//           Color attenuation is applied to the vertex colors in order to have correct visual result 
//           after glColorMaterial(GL_AMBIENT_AND_DIFFUSE). Without it, colors look unnatural and flat.
//=======================================================================

Standard_Integer TIGLAISColormapped::AttenuateColor( const Standard_Integer aColor,
                                                       const Standard_Real aComposition)
{
    Standard_Integer  red,
                      green,
                      blue,
                      alpha;

    alpha = aColor&0xff000000;
    alpha >>= 24;

    blue = aColor&0x00ff0000;
    blue >>= 16;

    green = aColor&0x0000ff00;
    green >>= 8;

    red = aColor&0x000000ff;
    red >>= 0; 

    red   = (Standard_Integer)(aComposition * red);
    green = (Standard_Integer)(aComposition * green);
    blue  = (Standard_Integer)(aComposition * blue);

    Standard_Integer  color;
    color = red;
    color += green << 8;
    color += blue  << 16; 
    color += alpha << 24;
    return color;
}

