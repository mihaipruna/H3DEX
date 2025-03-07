using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
/* LICENSE INFO:
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

/*Helix Toolkit
 * 
 * The MIT License (MIT)

Copyright (c) 2012 Helix Toolkit contributors

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * */

namespace H3DExSampleProject
{
    class Program
    {
        //example program to create ans save a loft as STL
        static void Main(string[] args)
        {
            //add first section
            List<Point3D> profile1 = new List<Point3D>();
            //a triangle (bottom)
            profile1.Add(new Point3D(1, 0, 0));
            profile1.Add(new Point3D(1, 1, 0));
            profile1.Add(new Point3D(0, 1, 0));
            //second section
            List<Point3D> profile2 = new List<Point3D>();
            //another triangle, top
            profile2.Add(new Point3D(2, 0, 3));
            profile2.Add(new Point3D(0, 2, 3));
            profile2.Add(new Point3D(2, 2, 3));
            //make sure point order consistent
            Plane3D xyplane=new Plane3D(new Point3D(0,0,0),new Vector3D(0,0,1));
            profile1 = H3DEx.PointOperations.FastClockwisePoints(profile1, xyplane);
            Plane3D topxyplane = new Plane3D(new Point3D(0, 0, 3), new Vector3D(0, 0, 1));
            profile2 = H3DEx.PointOperations.FastClockwisePoints(profile2, topxyplane);
            //make sure both lists start near same point so we don't get a twisted loft
            Point3D setStart = new Point3D(-1, 0, 0);
            profile1 = H3DEx.PointOperations.SetStartPoint(profile1, setStart);
            profile2 = H3DEx.PointOperations.SetStartPoint(profile2, setStart);
            //make top profile a spline sampled at 15 points
            profile2 = H3DEx.PointOperations.MakeSplinePoints(profile2, 15);
            //make sure bottom profile has same number of points
            profile1 = H3DEx.PointOperations.SetNumberPoints(profile1, 15);
            //create loft contours list of lists
            List<List<Point3D>> loftContours = new List<List<Point3D>>();
            loftContours.Add(profile1);
            loftContours.Add(profile2);
            //create loft with closed ends
            MeshGeometry3D loftMesh=H3DEx.MeshOperations.MakeCappedLoft(loftContours);
            //save to stl file in same folder as executable
            H3DEx.IOOperations.SaveSTL(loftMesh, @".\myLoft.stl");
        }
    }
}
