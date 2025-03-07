using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;
using System.Reflection;

namespace H3DEx
{
    /// <summary>
    /// Settings, tolerances, etc
    /// </summary>
    public static class Constants
    {
        //where to join points
        /// <summary>
        /// Welding tolerance
        /// </summary>
        public static double geometryWeldTolerance = 0.01;
        /// <summary>
        /// This is needed as a reference large number since Double.Maxvalue trips some Helix functions
        /// </summary>
        public static double largeNumber = 10000000;
        /// <summary>
        /// Less , and vectors are collinear.
        /// </summary>
        public static double angleDegreeTolerance = 2.0;
        internal static readonly PropertyInfo Visual3DModelPropertyInfo = typeof(Visual3D).GetProperty("Visual3DModel", BindingFlags.Instance | BindingFlags.NonPublic);

    }
    /// <summary>
    /// Functions acting on Point3D or collections of Point3D
    /// </summary>
    public static class PointOperations
    {
        /// <summary>
        /// Returns true if 3 points collinear
        /// </summary>
        /// <param name="pt1">Point3D</param>
        /// <param name="pt2">Point3D</param>
        /// <param name="pt3">Point3D</param>
        /// <returns>bool</returns>
        public static bool PointsCollinear(Point3D pt1,Point3D pt2,Point3D pt3)
        {
            Vector3D v1 = pt2 - pt1;
            Vector3D v2 = pt3 - pt1;
            if (v1.Length == 0 || v2.Length == 0) return true;
            v1.Normalize();
            v2.Normalize();
            if (v1 == v2) return true;
            v1.Negate();
            if (v1 == v2) return true;
            return false;
        }


        /// <summary>
        /// Gets bounding box of a list of points in 3d space
        /// </summary>
        /// <param name="pcloud">list of Point3D</param>
        /// <returns>Rect3D</returns>
        public static Rect3D GetPointCloudBoundingBox(List<Point3D> pcloud)
        {
            Rect3D bb = new Rect3D();
            double maxx = double.MinValue;
            double maxy = double.MinValue;
            double maxz = double.MinValue;
            double minx = double.MaxValue;
            double miny = double.MaxValue;
            double minz = double.MaxValue;
            int pc = pcloud.Count;
            for (int i = 0; i < pc; i++)
            {
                if (pcloud[i].X > maxx) maxx = pcloud[i].X;
                if (pcloud[i].X < minx) minx = pcloud[i].X;

                if (pcloud[i].Y > maxy) maxy = pcloud[i].Y;
                if (pcloud[i].Y < miny) miny = pcloud[i].Y;

                if (pcloud[i].Z > maxz) maxz = pcloud[i].Z;
                if (pcloud[i].Z < minz) minz = pcloud[i].Z;

            }
            bb = new Rect3D(minx , miny , minz , maxx - minx, maxy - miny, maxz - minz);

            return bb;
        }
        /// <summary>
        /// gets a point in the middle of a list of 3d points
        /// </summary>
        /// <param name="listofpoints">list of Point3D</param>
        /// <returns>Point3D</returns>
        public static Point3D GetPtsCentroid(List<Point3D> listofpoints)
        {
            Point3D centroid = new Point3D();
            if (listofpoints.Count > 0)
            {
                double sumx = 0;
                double sumy = 0;
                double sumz = 0;
                for (int i = 0; i < listofpoints.Count; i++)
                {
                    sumx += listofpoints[i].X / listofpoints.Count;
                    sumy += listofpoints[i].Y / listofpoints.Count;
                    sumz += listofpoints[i].Z / listofpoints.Count;
                }
                centroid = new Point3D(sumx, sumy, sumz);
            }
            return centroid;
        }
       
        /// <summary>
        /// Smooths a contour based on distance between points and segments made from previous and next point
        /// </summary>
        /// <param name="originalPts">List of Point3D</param>
        /// <param name="maxDev">maximum distance to keep</param>
        /// <returns>smoothed list of Point3D</returns>
        public static List<Point3D> FirstOrderSmoothContour(List<Point3D> originalPts, double maxDev)
        {
            List<Point3D> smoothed = new List<Point3D>();
            smoothed.Add(originalPts[0]);
            for (int i = 1; i < originalPts.Count - 1; i++)
            {
                Point3D pp = smoothed[smoothed.Count - 1];
                Point3D tp = originalPts[i];
                Point3D np = originalPts[i + 1];
                Vector3D pp2tp = tp - pp;
                Vector3D pp2np = np - pp;
                Vector3D proji = ProjectVectorOnVector(pp2tp, pp2np);
                Point3D projpoint = pp + proji;
                if (tp.DistanceTo(projpoint) <= maxDev)
                {
                    smoothed.Add(tp);
                }
            }
            return smoothed;
        }
        internal static void mpl(Point3D pt, Plane3D pl, Vector3D xaxis, out double X, out double Y)
        {
            X = double.MinValue;
            Y = double.MinValue;
            Vector3D parm = pt - pl.Position;


            Point3D xaxistip = pl.Position + xaxis;
            Point3D? xaxisProjected = ProjectPointOnPlane(xaxistip, pl);
            Vector3D xaxisInPlane = (Point3D)xaxisProjected - pl.Position;
            Vector3D yaxis = Vector3D.CrossProduct(pl.Normal, xaxisInPlane);

            Point3D yaxistip = pl.Position + yaxis;
            Point3D? yaxisProjected = ProjectPointOnPlane(yaxistip, pl);
            Vector3D yaxisInPlane = (Point3D)yaxisProjected - pl.Position;



            X = Vector3D.DotProduct(parm, xaxisInPlane) / xaxisInPlane.Length;
            Y = Vector3D.DotProduct(parm, yaxisInPlane) / yaxisInPlane.Length;

        }

        internal static bool lineSegmentIntersection(
      double Ax,
      double Ay,
           double Bx,
           double By,
      double Cx,
           double Cy,
      double Dx,
           double Dy,
      out double X, out double Y)
        {

            //double X = double.MinValue;

            //double Y = double.MinValue;

            //double Ax = A.X;
            //double Ay = A.Y;
            //double Bx = B.X;
            //double By = B.Y;
            //double Cx = C.X;
            //double Cy = C.Y;
            //double Dx = D.X;
            //double Dy = D.Y;
            Y = double.MinValue;
            X = double.MinValue;
            double distAB, theCos, theSin, newX, ABpos;

            //  Fail if either line segment is zero-length.
            if (Ax == Bx && Ay == By || Cx == Dx && Cy == Dy) return false;

            //  Fail if the segments share an end-point.
            if (Ax == Cx && Ay == Cy)
            {
                X = Ax;
                Y = Ay;
                return true;
            }
            if (Bx == Cx && By == Cy)
            {
                X = Bx;
                Y = By;
                return true;
            }
            if (Ax == Dx && Ay == Dy)
            {
                X = Ax;
                Y = Ay;
                return true;
            }
            if (Bx == Dx && By == Dy)
            {
                X = Bx;
                Y = By;
                return false;
            }

            //  (1) Translate the system so that point A is on the origin.
            Bx -= Ax; By -= Ay;
            Cx -= Ax; Cy -= Ay;
            Dx -= Ax; Dy -= Ay;

            //  Discover the length of segment A-B.
            distAB = Math.Sqrt(Bx * Bx + By * By);

            //  (2) Rotate the system so that point B is on the positive X axis.
            theCos = Bx / distAB;
            theSin = By / distAB;
            newX = Cx * theCos + Cy * theSin;
            Cy = Cy * theCos - Cx * theSin; Cx = newX;
            newX = Dx * theCos + Dy * theSin;
            Dy = Dy * theCos - Dx * theSin; Dx = newX;

            //  Fail if segment C-D doesn't cross line A-B.
            if (Cy < 0 && Dy < 0 || Cy >= 0 && Dy >= 0) return false;

            //  (3) Discover the position of the intersection point along line A-B.
            ABpos = Dx + (Cx - Dx) * Dy / (Dy - Cy);

            //  Fail if segment C-D crosses line A-B outside of segment A-B.
            if (ABpos < 0 || ABpos > distAB) return false;

            //  (4) Apply the discovered position to line A-B in the original coordinate system.
            X = Ax + ABpos * theCos;
            Y = Ay + ABpos * theSin;

            //  Success.
            return true;
        }

        /// <summary>
        /// Removes points too close to other points
        /// </summary>
        /// <param name="originalPoints">List of Point3D</param>
        /// <param name="distanceToler">removal limit distance</param>
        /// <returns>pruned list of Point3D</returns>
        public static List<Point3D> PrunePoints(List<Point3D> originalPoints, double distanceToler)
        {
            List<Point3D> toprune = new List<Point3D>(originalPoints);
            List<Point3D> cleared = new List<Point3D>();
            for (int i1 = 0; i1 < toprune.Count; i1++)
            {
                Point3D p1 = toprune[i1];
                bool foundit = false;
                for (int i2 = 0; i2 < cleared.Count; i2++)
                {
                    Point3D p2 = cleared[i2];
                    if (p1.DistanceTo(p2) < distanceToler)
                    {
                        foundit = true;
                        break;
                    }
                }
                if (foundit == false)
                {
                    cleared.Add(p1);
                }
            }
            return cleared;
        }
        /// <summary>
        /// Returns a list of normal vectors pointing out from a closed loop, for every point in loop
        /// </summary>
        /// <param name="cpts">List of Point3D</param>
        /// <returns>List of Vector3D</returns>
        public static List<Vector3D> GetOutwardContourNormals(List<Point3D> cpts)
        {
            Point3D cen = GetPtsCentroid(cpts);
            double totalLength = GetContourLength(cpts);
            Vector3D norm = Vector3D.CrossProduct(cpts[0] - cen, cpts[cpts.Count / 2] - cen);
            Plane3D pointsPlane = new Plane3D(cen, norm);
            List<Vector3D> norms = new List<Vector3D>();

            Vector3D xaxis = cpts[0] - cen;
            for (int i = 0; i < cpts.Count; i++)
            {
                Vector3D thisdir = new Vector3D();
                if (i < cpts.Count - 1)
                {
                    thisdir = cpts[i + 1] - cpts[i];

                }
                else
                {
                    thisdir = cpts[0] - cpts[i];
                }
                Point3D A = cpts[i];

                Vector3D thisNorm = Vector3D.CrossProduct(thisdir, norm);
                thisNorm.Normalize();
                thisNorm = thisNorm * totalLength;
                Point3D B = A + thisNorm;
                //check intersection with other segments
                int otherCount = 0;
                for (int j = 0; j < cpts.Count; j++)
                {
                    if (i != j)
                    {
                        Point3D C = cpts[j];
                        Point3D D = new Point3D();
                        if (j < cpts.Count - 1)
                        {
                            D = cpts[j + 1];
                        }
                        else
                        {
                            D = cpts[0];
                        }

                        double OX = double.MinValue;
                        double OY = double.MinValue;
                        Double AX = double.MinValue;
                        Double AY = double.MinValue;
                        Double BX = double.MinValue;
                        Double BY = double.MinValue;
                        Double CX = double.MinValue;
                        Double CY = double.MinValue;
                        Double DX = double.MinValue;
                        Double DY = double.MinValue;
                        mpl(A, pointsPlane, xaxis, out AX, out AY);
                        mpl(B, pointsPlane, xaxis, out BX, out BY);
                        mpl(C, pointsPlane, xaxis, out CX, out CY);
                        mpl(D, pointsPlane, xaxis, out DX, out DY);




                        if (lineSegmentIntersection(AX, AY, BX, BY, CX, CY, DX, DY, out OX, out OY)) otherCount++;

                    }
                }
                if (otherCount % 2 != 0) thisNorm.Negate();
                norms.Add(thisNorm);


            }
            return norms;


        }
        /// <summary>
        /// Refactors points in a list to match a given number. used to make sure all loft contours have same number of points
        /// </summary>
        /// <param name="contourPoints">points list</param>
        /// <param name="newNumberPoints">numver</param>
        /// <returns>new points list</returns>
        public static List<Point3D> SetNumberPoints(List<Point3D> contourPoints, int newNumberPoints)
        {
            List<Point3D> newPts = new List<Point3D>();
            if (newNumberPoints < 2) return null;
            double getL = GetContourLength(contourPoints);
            if (getL <= 0) return null;
            if (contourPoints.Count < 2) return null;
            List<Point3D> newPlist = new List<Point3D>();
            for (int i = 0; i < newNumberPoints; i++)
            {
                double thisr = (double)i / (double)(newNumberPoints - 1); // * (double)getL;/
                if (thisr < 0) thisr = 0;
                if (thisr > 1) thisr = 1;
                newPlist.Add(GetPointOnContour(contourPoints, thisr));

            }
            if (newPlist.Count > 0) return newPlist;
            return null;
        }

        /// <summary>
        /// calculates length of a contour made up of points by adding segemnt lengths
        /// </summary>
        /// <param name="cpoints"list of points></param>
        /// <returns>length value</returns>
        public static double GetContourLength(List<Point3D> cpoints)
        {
            double le = 0;
            List<Point3D> plist = cpoints;
            for (int i = 0; i < plist.Count - 1; i++)
            {
                Vector3D lv = plist[i + 1] - plist[i];
                le = le + lv.Length;
            }

            return le;
        }

        /// <summary>
        ///  Gets a point on a contour at specific ration to contour length by interpolating
        /// </summary>
        /// <param name="contourPoints">list of points</param>
        /// <param name="lengthRatio">ratio 0-1</param>
        /// <returns>Point3D</returns>
        public static Point3D GetPointOnContour(List<Point3D> contourPoints, double lengthRatio)
        {
            Point3D myPoint = new Point3D();

            if (contourPoints.Count > 1)
            {

                if (lengthRatio >= 0 && lengthRatio <= 1)
                {
                    double wholength = GetContourLength(contourPoints);
                    double whereget = wholength * lengthRatio;
                    double clength = 0;
                    for (int i = 0; i < contourPoints.Count - 1; i++)
                    {
                        double cseg = contourPoints[i].DistanceTo(contourPoints[i + 1]);
                        if (clength + cseg < whereget)
                        {
                            clength += cseg;
                        }
                        else if (cseg > 0)
                        {
                            double diff = whereget - clength;
                            double pratio = diff / cseg;
                            return contourPoints[i] + (contourPoints[i + 1] - contourPoints[i]) * pratio;
                        }
                    }
                }

            }
            else
            {

            }
            return myPoint;
        }

        /// <summary>
        /// Projects a vector on another
        /// </summary>
        /// <param name="toProject">Vector3D to project</param>
        /// <param name="projOn">Vector3D to project on</param>
        /// <returns>projection as Vector3D</returns>
        public static Vector3D ProjectVectorOnVector(Vector3D toProject, Vector3D projOn)
        {


            Vector3D newproj = new Vector3D();
            if (projOn.Length == 0) return newproj;

            newproj = Vector3D.Multiply(Vector3D.DotProduct(toProject, projOn) / projOn.LengthSquared, projOn);
            return newproj;

        }

        /// <summary>
        /// projects a point ona  plane
        /// </summary>
        /// <param name="toProject">Point3D</param>
        /// <param name="plane">Plane3D</param>
        /// <returns>Point3D or null</returns>
        public static Point3D? ProjectPointOnPlane(Point3D toProject, Plane3D plane)
        {
            //make long line
            double bigNumber = Constants.largeNumber;// toProject.DistanceTo(plane.Position) * 2;   // Double.MaxValue * 0.5;
            Vector3D normie = plane.Normal;
            normie.Normalize();
            Point3D onextreme = toProject - normie * bigNumber;
            Point3D otherextreme = toProject + normie * bigNumber;
            object tpla = plane.LineIntersection(onextreme, otherextreme);
            if (tpla == null)
            {
                return null; ;
            }
            else
            {

                Point3D pao = (Point3D)tpla;
                // if (Math.Abs (pao.X) >= bigNumber || Math.Abs(pao.Y) >= bigNumber || Math.Abs(pao.Z )>= bigNumber) return null;
                return pao;
            }
        }
        /// <summary>
        /// Make point index middle of contour based on indices
        /// </summary>
        /// <param name="contour">list of Point3D</param>
        /// <param name="index">valid integer index</param>
        /// <param name="newindex">out new index of same point</param>
        /// <returns></returns>
        public static List<Point3D> MakePointMiddle(List<Point3D> contour, int index, out int newindex)
        {
            int nstart = index + contour.Count / 2;
            if (nstart >= contour.Count) nstart = nstart - contour.Count;
            List<Point3D> lala = SetStartPoint(contour, contour[nstart]);
            newindex = lala.IndexOf(contour[index]);
            return lala;

        }

        /// <summary>
        /// True if point is not on a straight segment
        /// </summary>
        /// <param name="contour">list Point3D</param>
        /// <param name="index">valid integer index</param>
        /// <returns>booleans</returns>
        public static bool PointIsCorner(List<Point3D> contour, int index)
        {
            Point3D thisP = contour[index];
            Point3D nextP = new Point3D();
            Point3D prevP = new Point3D();
            if (index == 0)
            {
                nextP = contour[index + 1];
                prevP = contour[contour.Count - 1];
            }
            else if (index == contour.Count - 1)
            {

                prevP = contour[index - 1];
                nextP = contour[0];
            }
            else
            {
                nextP = contour[index + 1];
                prevP = contour[index - 1];
            }

            Vector3D p2t = thisP - prevP;
            Vector3D t2n = nextP - thisP;
            if (Math.Abs(Vector3D.AngleBetween(p2t, t2n)) > Constants.angleDegreeTolerance) return true;
            return false;

        }

        /// <summary>
        /// Checks if a list of points contains origin
        /// </summary>
        /// <param name="pts">list of Point3D</param>
        /// <returns>true of false</returns>
        public static bool ContainsOrigin(List<Point3D> pts)
        {
            Point3D porigin = new Point3D(0, 0, 0);
            foreach (Point3D p in pts)
            {
                if (p.DistanceTo(porigin) < Constants.geometryWeldTolerance) return true;
            }
            return false;
        }

        /// <summary>
        /// Cycles through points list to start near start point
        /// </summary>
        /// <param name="pts">list of Point3D</param>
        /// <param name="nearStart">Point3D</param>
        /// <returns>updated list</returns>
        public static List<Point3D> SetStartPoint(List<Point3D> pts, Point3D nearStart)
        {

            int findex = NearestPointIndex(pts, nearStart);


            List<Point3D> neworder = new List<Point3D>();
            for (int i = findex; i < pts.Count; i++)
            {
                neworder.Add(pts[i]);
            }
            for (int i = 0; i < findex; i++)
            {
                neworder.Add(pts[i]);
            }

            return neworder;


        }
        /// <summary>
        /// Returns the index of the point in list nearest to specified point
        /// </summary>
        /// <param name="pts">Listof Point3d</param>
        /// <param name="findNear">Point3D</param>
        /// <returns>integer</returns>
        public static int NearestPointIndex(List<Point3D> pts, Point3D findNear)
        {
            int findex = -1;
            double dfound = Double.MaxValue;
            for (int i = 0; i < pts.Count; i++)
            {
                Point3D thisP = pts[i];
                double dc = thisP.DistanceTo(findNear);
                if (dc < dfound)
                {
                    dfound = dc;
                    findex = i;
                }
            }
            return findex;
        }
        /// <summary>
        ///  reverses points list order if more than half not clockwise by angle
        /// </summary>
        /// <param name="originalPoints">List of points, should be roughly in same plane</param>
        /// <param name="pointsPlane">Plane holding points</param>
        /// <returns>updated list</returns>
        public static List<Point3D> StatisticClockwisePoints(List<Point3D> originalPoints, Plane3D pointsPlane)
        {

            List<Point3D> newlist = new List<Point3D>(originalPoints);

            Point3D centroid = GetPtsCentroid(newlist);
            bool outoforder = false;
            Vector3D xaxis = Point3D.Subtract(newlist[0], centroid);

            int ooo = 0;
            for (int i1 = 0; i1 < newlist.Count - 1; i1++)
            {
                Vector3D ax1 = Point3D.Subtract(newlist[i1], centroid);
                int i2 = i1 + 1;
                Vector3D ax2 = Point3D.Subtract(newlist[i2], centroid);
                if (AngleInPlane(ax1, xaxis, pointsPlane) > AngleInPlane(ax2, xaxis, pointsPlane))
                {



                    ooo++;
                }
            }
            if (ooo > originalPoints.Count / 2) newlist.Reverse();


            return newlist;
        }



        /// <summary>
        ///  reverses points list order if any not clockwise as compared by angle. fastest
        /// </summary>
        /// <param name="originalPoints">List of points, should be roughly in same plane</param>
        /// <param name="pointsPlane">Plane holding points</param>
        /// <returns>updated list</returns>
        public static List<Point3D> FastClockwisePoints(List<Point3D> originalPoints, Plane3D pointsPlane)
        {

            List<Point3D> newlist = new List<Point3D>(originalPoints);

            Point3D centroid = GetPtsCentroid(newlist);
            bool outoforder = false;
            Vector3D xaxis = Point3D.Subtract(newlist[0], centroid);

            outoforder = false;
            for (int i1 = 0; i1 < newlist.Count - 1; i1++)
            {
                Vector3D ax1 = Point3D.Subtract(newlist[i1], centroid);
                int i2 = i1 + 1;
                Vector3D ax2 = Point3D.Subtract(newlist[i2], centroid);
                if (AngleInPlane(ax1, xaxis, pointsPlane) > AngleInPlane(ax2, xaxis, pointsPlane))
                {



                    outoforder = true;
                    break;
                }
            }
            if (outoforder == true) newlist.Reverse();


            return newlist;
        }

        /// <summary>
        /// Calculates angle between vectors based on plane
        /// </summary>
        /// <param name="myv">one vector</param>
        /// <param name="otherv">another vector</param>
        /// <param name="myplane">plane to base calculation on/param>
        /// <returns>angle in radians</returns>
        public static double AngleInPlane(Vector3D myv, Vector3D otherv, Plane3D myplane)
        {
            double ang = 0;
            try
            {
                if (myv.Length == 0 || otherv.Length == 0) return ang;
                myv.Normalize();
                otherv.Normalize();
                if (myv.X == otherv.X && myv.Y == otherv.Y && myv.Z == otherv.Z) return 0;
                if (myv.X == -otherv.X && myv.Y == -otherv.Y && myv.Z == -otherv.Z) return Math.PI;

                double thecosine = (Vector3D.DotProduct(myv, otherv) / myv.Length) / otherv.Length;
                Vector3D cp = Vector3D.CrossProduct(myv, otherv);
                double thesine = 0;
                //try
                //{
                thesine = cp.Length * (Math.Sign(Vector3D.DotProduct(cp, myplane.Normal)) / myv.Length) / otherv.Length;
                //}
                //catch
                //{
                //    thesine = 0;
                //}
                if (thesine == 0)
                {
                    if (thecosine > 0)
                    {
                        ang = 0;
                    }
                    else
                    {
                        ang = Math.PI;
                    }
                }
                else if (thecosine == 0)
                {
                    if (thesine > 0)
                    {
                        ang = Math.PI * 0.5;
                    }
                    else
                    {
                        ang = Math.PI * 1.5;
                    }

                }
                else
                {
                    if (thesine > 0)
                    {
                        ang = Math.Acos(thecosine);
                    }
                    else
                    {
                        ang = 2 * Math.PI - Math.Acos(thecosine);
                    }
                }
            }
            catch(Exception ex)
            {

            }
            return ang;
        }
        /// <summary>
        /// makes nice smooth curve through points
        /// </summary>
        /// <param name="seedPoints">list of points</param>
        /// <param name="numberPoints">number of points on spline</param>
        /// <returns></returns>
        public static List<Point3D> MakeSplinePoints(List<Point3D> seedPoints, int numberPoints)
        {
            List<Point3D> splinePts = new List<Point3D>();
            double ratioStep = 1.0 / (double)numberPoints;
            for (int i = 0; i < numberPoints; i++)
            {
                double cRatio = (double)i * 1.0 / (double)(numberPoints - 1);

                Point3D myPoint = new Point3D();
                List<Point3D> tempoints = new List<Point3D>(seedPoints);



                int ii = tempoints.Count - 1;
                while (ii > 0)
                {
                    for (int k = 0; k < ii; k++)
                    {
                        tempoints[k] = tempoints[k] + cRatio * (tempoints[k + 1] - tempoints[k]);
                    }
                    ii--;
                }

                myPoint = tempoints[0];


                splinePts.Add(myPoint);
            }



            return splinePts;
        }

        /// <summary>
        /// /Finds ratio of point to length of contour
        /// </summary>
        /// <param name="contourPoints">list of points</param>
        /// <param name="ptIndex">index</param>
        /// <returns>ratio (0-1)</returns>
        public static double GetContourPointRatio(List<Point3D> contourPoints, int ptIndex)
        {

            if (ptIndex == 0) return 0;
            if (ptIndex == contourPoints.Count - 1) return 1;
            double plenght = GetContourLength(contourPoints);
            if (plenght == 0) return 0;


            double sofar = 0;
            for (int i = 0; i < ptIndex; i++)
            {
                double cseg = contourPoints[i].DistanceTo(contourPoints[i + 1]);
                sofar += cseg;

            }
            return sofar / plenght;

        }

    }
    /// <summary>
    /// Mesh generation and contour cutting
    /// </summary>
    public static class MeshOperations
    {

       

        /// <summary>
        ///  returns contour points on plane cut
        /// </summary>
        /// <param name="myMesh">3D Mesh</param>
        /// <param name="myPlane">Plane</param>
        /// <returns>contour points</returns>
        public static List<Point3D> CutMeshContour(MeshGeometry3D myMesh, HelixToolkit.Wpf.Plane3D myPlane)
        {
            Vector3D n = myPlane.Normal;
           
            List<Point3D> retC = new List<Point3D>();
            var segments = MeshGeometryHelper.GetContourSegments(myMesh, myPlane.Position, n).ToList();
            //var degments = MeshGeometryHelper.GetContourSegments(myMesh, myPlane.Position, myPlane.Normal).ToList();
            //var wegments = MeshGeometryHelper.GetContourSegments(myMesh, myPlane.Position, myPlane.Normal).ToList();
            ////assume one contour (one mesh)
            var gg = MeshGeometryHelper.CombineSegments(segments, Constants.geometryWeldTolerance).ToList();
            //var gl = MeshGeometryHelper.CombineSegments(degments, geometryWeldTolerance*10).ToList();
            //var gll = MeshGeometryHelper.CombineSegments(wegments, geometryWeldTolerance / 10).ToList();
            foreach (var contour in gg)
            {
                if (contour.Count > 2)
                {
                    retC = new List<Point3D>();
                    retC.AddRange(contour);


                    return retC;
                }
            }
            return retC;
        }
        /// <summary>
        /// Cuts contours over objects which might have multiple children
        /// </summary>
        /// <param name="myModel">ModelVisual3D</param>
        /// <param name="myPlane">Plane3D</param>
        /// <returns>list of lists of Point3D</returns>
        public static List<List<Point3D>> CutContours(ModelVisual3D myModel, HelixToolkit.Wpf.Plane3D myPlane)
        {
            Vector3D n = myPlane.Normal;
            
            List<GeometryModel3D> lg = getGeometryChildren(myModel);
            List<List<Point3D>> cpts = new List<List<Point3D>>();
            foreach (GeometryModel3D mao in lg)
            {
                var segments = MeshGeometryHelper.GetContourSegments(mao.Geometry as MeshGeometry3D, myPlane.Position, n).ToList();
                //List<Point3D> mycpts = new List<Point3D>();
                foreach (var contour in MeshGeometryHelper.CombineSegments(segments, Constants.geometryWeldTolerance).ToList())
                {
                    if (contour.Count > 2)
                    {
                        List<Point3D> mycpts = new List<Point3D>();
                        mycpts.AddRange(contour);


                        cpts.Add(mycpts);
                        
                    }
                }
            }
            return cpts;
        }

         static List<GeometryModel3D> getGeometryChildren(ModelVisual3D m)
         {
             List<GeometryModel3D> listobjects =new List<GeometryModel3D>();
            if (m.Content != null)
            {
                GeometryModel3D g = m.Content as GeometryModel3D;
                if (g != null)
                    listobjects.Add(g);
            }
            foreach (ModelVisual3D mg in m.Children)
            {
                listobjects.AddRange(getGeometryChildren(mg));
            }


            return listobjects;
         }


       



        /// <summary>
        /// Creates loft geometry from sets of matching points
        /// </summary>
        /// <param name="contours">list of list of points</param>
        /// <returns>mesh geometry</returns>
        public static MeshGeometry3D MakeLoft(List<List<Point3D>> contours)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();
            var positions = mesh.Positions;
            var normals = mesh.Normals;
            var textureCoordinates = mesh.TextureCoordinates;
            var triangleIndices = mesh.TriangleIndices;
            mesh.Positions = null;
            mesh.Normals = null;
            mesh.TextureCoordinates = null;
            mesh.TriangleIndices = null;

            int index0 = positions.Count;
            int n = -1;
            for (int i = 0; i < contours.Count; i++)
            {
                List<Point3D> pc = contours[i];

                // check that all curves have same number of points
                if (n == -1)
                    n = pc.Count;
                if (pc.Count != n)
                    throw new InvalidOperationException("All curves should have the same number of points");

                //Point3D centroid = getPtsCentroid(pc);

                // add the points
                int pindex = 0;
                foreach (var p in pc)
                {
                    positions.Add(p);
                    pindex++;
                }
                List<Vector3D> nos = PointOperations.GetOutwardContourNormals(pc);
                foreach (Vector3D v in nos)
                {
                    v.Normalize();
                    normals.Add(v);
                }

                //close
                positions.Add(pc[0]);
                Vector3D v1 = normals[0];
                v1.Normalize();
                normals.Add(v1);

            }
            n++;
            for (int i = 0; i + 1 < contours.Count; i++)
            {
                for (int j = 0; j + 1 < n; j++)
                {
                    int i0 = index0 + i * n + j;
                    int i1 = i0 + n;
                    int i2 = i1 + 1;
                    int i3 = i0 + 1;
                    triangleIndices.Add(i0);
                    triangleIndices.Add(i1);
                    triangleIndices.Add(i2);

                    triangleIndices.Add(i2);
                    triangleIndices.Add(i3);
                    triangleIndices.Add(i0);
                }
            }

            mesh.Positions = positions;
            mesh.Normals = normals;
            mesh.TextureCoordinates = textureCoordinates;
            mesh.TriangleIndices = triangleIndices;
            return mesh;
        }


        /// <summary>
        /// Creates capped loft geometry from sets of matching points
        /// </summary>
        /// <param name="contours">list of list of points</param>
        /// <returns>mesh geometry</returns>
        public static MeshGeometry3D MakeCappedLoft(List<List<Point3D>> contours)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();
            var positions = mesh.Positions;
            var normals = mesh.Normals;
            var textureCoordinates = mesh.TextureCoordinates;
            var triangleIndices = mesh.TriangleIndices;
            mesh.Positions = null;
            mesh.Normals = null;
            mesh.TextureCoordinates = null;
            mesh.TriangleIndices = null;

            int index0 = positions.Count;
            int n = -1;
            List<int> startIndices = new List<int>();
            List<int> endIndices = new List<int>();
            int pindex = 0;
            //figure out end normals
            Point3D sc = PointOperations.GetPtsCentroid(contours[0]);
            Point3D sc1 = PointOperations.GetPtsCentroid(contours[1]);
            Vector3D sno = sc - sc1;
            sno.Normalize();

            Point3D ec = PointOperations.GetPtsCentroid(contours[contours.Count - 1]);
            Point3D ec1 = PointOperations.GetPtsCentroid(contours[contours.Count - 2]);
            Vector3D eno = ec - ec1;
            eno.Normalize();

            for (int i = 0; i < contours.Count; i++)
            {
                List<Point3D> pc = contours[i];

                // check that all curves have same number of points
                if (n == -1)
                    n = pc.Count;
                if (pc.Count != n)
                    throw new InvalidOperationException("All curves should have the same number of points");

                //Point3D centroid = getPtsCentroid(pc);

                // add the points

                foreach (var p in pc)
                {
                    if (i == 0)
                    {
                        startIndices.Add(pindex);

                    }
                    if (i == contours.Count - 1)
                    {
                        endIndices.Add(pindex);
                    }
                    positions.Add(p);
                    pindex++;
                }
                List<Vector3D> nos = PointOperations.GetOutwardContourNormals(pc);
                foreach (Vector3D v in nos)
                {
                    v.Normalize();
                    normals.Add(v);
                }

                //close
                positions.Add(pc[0]);
                pindex++;
                Vector3D v1 = normals[0];
                v1.Normalize();
                normals.Add(v1);

            }
            n++;
            for (int i = 0; i + 1 < contours.Count; i++)
            {
                for (int j = 0; j + 1 < n; j++)
                {
                    int i0 = index0 + i * n + j;
                    int i1 = i0 + n;
                    int i2 = i1 + 1;
                    int i3 = i0 + 1;
                    triangleIndices.Add(i0);
                    triangleIndices.Add(i1);
                    triangleIndices.Add(i2);

                    triangleIndices.Add(i2);
                    triangleIndices.Add(i3);
                    triangleIndices.Add(i0);
                }
            }



            mesh.Positions = positions;
            mesh.Normals = normals;

            mesh.TriangleIndices = triangleIndices;

            MeshBuilder capb = new MeshBuilder();
            capb.CreateTextureCoordinates = false;
            capb.Append(mesh);

            int normindex = capb.Normals.Count;
           capb.AddPolygonByCuttingEars(startIndices);
            MeshGeometry3D mcas=capb.ToMesh();
            //set normals for start
            
            for (int i = normindex; i < mcas.Positions.Count;i++ )
            {
               mcas.Normals[i] = sno;

            }
            MeshBuilder nmas = new MeshBuilder();
            nmas.CreateTextureCoordinates = false;
            nmas.Append(mcas);
            normindex = mcas.Positions.Count;
            nmas.AddPolygonByCuttingEars(endIndices);
            MeshGeometry3D ecas = nmas.ToMesh();



            //set normals for end

            for (int i = normindex; i < ecas.Positions.Count; i++)
            {
                ecas.Normals[i] =eno;

            }




            return ecas;
        }

       

      

       


       //public static List<Point3D> ScaleContour(List<Point3D> originalPoints,double scaleFactor)
       // {
       //     List<Vector3D> ON = GetOutwardContourNormals(originalPoints);
       //     int counter = 0;
       //    foreach (Point3D po in originalPoints)
       //    {

       //    }
       // }

       
      


        


        

        /// <summary>
        /// makes a simple  box geometry based on bounds
        /// </summary>
        /// <param name="bounds">Rect3D</param>
        /// <returns>geometry model 3d</returns>
        public static GeometryModel3D MakeTestBox(Rect3D bounds)
        {
            var meshBuilder = new MeshBuilder(false, false);
            meshBuilder.AddBox(bounds.Location, bounds.SizeX, bounds.SizeY, bounds.SizeZ);
            var mesh = meshBuilder.ToMesh(true);
            GeometryModel3D newModel = new GeometryModel3D();
            newModel.Geometry = mesh;
            return newModel;
        }

       

      
      

      

    }


    /// <summary>
    /// Geometry read from and save to file
    /// </summary>
    public static class IOOperations
    {

        /// <summary>
        /// loads a CAD model from file
        /// </summary>
        /// <param name="CADfile">valid path</param>
        /// <returns>3D model</returns>
        public static GeometryModel3D ReadCADGeometry(string CADfile)
        {

            FileModelVisual3D fv = new FileModelVisual3D();
            fv.Source = CADfile;

            if (fv.Source != null)
            {

                Model3DGroup ga = Constants.Visual3DModelPropertyInfo.GetValue(fv, null) as Model3DGroup;
                foreach (Model3D m in ga.Children)
                {


                    GeometryModel3D mg = m as GeometryModel3D;
                    if (mg != null) return mg;
                }

            }
            return null;
        }
        /// <summary>
        /// Reads a CAD file as ModelVisual3D
        /// </summary>
        /// <param name="CADfile">valid CAD file</param>
        /// <returns><ModelVisual3D/returns>
        public static ModelVisual3D ReadCADModelVisual3D(string CADfile)
        {
            ModelVisual3D mv = new ModelVisual3D();
            FileModelVisual3D fv = new FileModelVisual3D();
            fv.Source = CADfile;

            if (fv.Source != null)
            {

                Model3DGroup ga = Constants.Visual3DModelPropertyInfo.GetValue(fv, null) as Model3DGroup;
                if (ga.Children.Count > 1)
                {
                    foreach (Model3D m in ga.Children)
                    {

                        ModelVisual3D mvv = new ModelVisual3D();
                        mvv.Content = m;
                        mv.Children.Add(mvv);
                    }
                }
                else
                {
                    Model3D m = ga.Children[0] as Model3D;
                    mv.Content = m;
                }

            }
            return mv;
        }
        /// <summary>
        /// Saves geometry mesh as ASCII STL 
        /// </summary>
        /// <param name="mtd">mesh geometry</param>
        /// <param name="filePath">valid file path</param>
        public static void SaveSTL(MeshGeometry3D mtd, string filePath)
        {
            List<string> lines = new List<string>();
            if (mtd != null)
            {
                lines.Add("solid mysolid");
                //write the mesh

                for (int tri = 0; tri < mtd.TriangleIndices.Count / 3; tri++)
                {

                    Point3D pos1 = mtd.Positions[mtd.TriangleIndices[tri * 3]];

                    Point3D pos2 = mtd.Positions[mtd.TriangleIndices[tri * 3 + 1]];
                    Point3D pos3 = mtd.Positions[mtd.TriangleIndices[tri * 3 + 2]];

                    if (PointOperations.PointsCollinear(pos1, pos2, pos3)) continue;

                    Vector3D outnormal = new Vector3D(1, 1, 1);
                    if (mtd.Normals != null)
                    {
                        try
                        {
                            outnormal = mtd.Normals[mtd.TriangleIndices[tri * 3]] + mtd.Normals[mtd.TriangleIndices[tri * 3 + 1]] + mtd.Normals[mtd.TriangleIndices[tri * 3 + 2]];
                        }
                        catch
                        {
                            outnormal = new Vector3D(1, 1, 1);
                        }
                    }
                    outnormal = outnormal / 3;

                    if (outnormal.X != outnormal.X) outnormal.X = 1;
                    if (outnormal.Y != outnormal.Y) outnormal.Y = 1;
                    if (outnormal.Z != outnormal.Z) outnormal.Z = 1;
                    outnormal.Normalize();
                    //ordering
                    List<Point3D> plist = new List<Point3D>();
                    plist.Add(pos1);
                    plist.Add(pos2);
                    plist.Add(pos3);

                    Plane3D p1 = new Plane3D(pos1, outnormal);
                    List<Point3D> newlist = PointOperations.StatisticClockwisePoints(plist, p1);

                    pos1 = newlist[0];
                    pos2 = newlist[1];
                    pos3 = newlist[2];



                    lines.Add("facet normal " + outnormal.X.ToString() + " " + outnormal.Y.ToString() + " " + outnormal.Z.ToString());
                    lines.Add("    outer loop");
                    lines.Add("        vertex " + pos1.X.ToString() + " " + pos1.Y.ToString() + " " + pos1.Z.ToString());
                    lines.Add("        vertex " + pos2.X.ToString() + " " + pos2.Y.ToString() + " " + pos2.Z.ToString());
                    lines.Add("        vertex " + pos3.X.ToString() + " " + pos3.Y.ToString() + " " + pos3.Z.ToString());
                    lines.Add("    endloop");
                    lines.Add("endfacet");

                }
                lines.Add("endsolid mysolid");
                System.IO.File.WriteAllLines(filePath, lines.ToArray());
            }

        }
    }
}
