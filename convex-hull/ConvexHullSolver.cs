using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace _2_convex_hull
{
    class ConvexHullSolver
    {
        System.Drawing.Graphics g;
        System.Windows.Forms.PictureBox pictureBoxView;

        public ConvexHullSolver(System.Drawing.Graphics g, System.Windows.Forms.PictureBox pictureBoxView)
        {
            this.g = g;
            this.pictureBoxView = pictureBoxView;
        }

        public void Refresh()
        {
            // Use this especially for debugging and whenever you want to see what you have drawn so far
            pictureBoxView.Refresh();
        }

        public void Pause(int milliseconds)
        {
            // Use this especially for debugging and to animate your algorithm slowly
            pictureBoxView.Refresh();
            System.Threading.Thread.Sleep(milliseconds);
        }

        public void Solve(List<System.Drawing.PointF> pointList)
        {
            pointList.Sort(new HorizontalSort());
            List<PointF> solution = ConvexHull(pointList);
        }

        private List<PointF> ConvexHull(List<PointF> points)
        {
            if (points.Count > 3)
            {
                List<PointF> left = new List<PointF>();
                List<PointF> right = new List<PointF>();

                Divide(points, ref left, ref right);

                List<PointF> convexL = ConvexHull(left);
                List<PointF> convexR = ConvexHull(right);

                List<PointF> solution = Join(convexL, convexR);
                System.Diagnostics.Debug.Write("Answer Convex:\n");
                foreach (PointF point in solution)
                {
                    System.Diagnostics.Debug.Write("    (" + point.X + ", " + point.Y + ")\n");
                }
                return solution;
            }
            else
            {
                System.Diagnostics.Debug.Write("One Convex:\n");
                foreach (PointF point in points)
                {
                    System.Diagnostics.Debug.Write("    (" + point.X + ", " + point.Y + ")\n");
                }
                
                return points;
            }
        }

        private void Divide(List<PointF> points, ref List<PointF> left, ref List<PointF> right)
        {
            Array pointsA = points.ToArray();
            int leftSize = pointsA.Length / 2;

            for (int i = 0; i < leftSize; i++)
            {
                PointF point = (PointF)pointsA.GetValue(i);
                left.Add(point);
            }
            for (int i = leftSize; i < pointsA.Length; i++)
            {
                PointF point = (PointF)pointsA.GetValue(i);
                right.Add(point);
            }
        }

        public List<PointF> Join(List<PointF> left, List<PointF> right)
        {
            build(ref left, ref right, true);
            build(ref left, ref right, false);

            left.AddRange(right);
            return left;
        }

        public void build(ref List<PointF> left, ref List<PointF> right, Boolean upper = true)
        {
            PointF leftStart = left[left.Count - 1];
            PointF rightStart = right[0];

            float slope = Slope(leftStart, rightStart);
            float tempSlope = slope;
            Boolean found = false;
            do
            {
                PointF leftCheck;
                found = NextCounterClockwise(left, leftStart, out leftCheck, upper);
                
                tempSlope = Slope(leftCheck, rightStart);
                if ((upper && tempSlope <= slope) || (!upper && tempSlope > slope))
                {
                    left.Remove(leftStart);
                    leftStart = leftCheck;
                }
            } while (((upper && tempSlope <= slope) || (!upper && tempSlope > slope)) && found);

            do
            {
                PointF rightCheck;
                found = NextClockwise(right, rightStart, out rightCheck, upper);

                tempSlope = Slope(rightCheck, leftStart);
                if ((upper && tempSlope >= slope) || (!upper && tempSlope < slope))
                {
                    right.Remove(rightStart);
                    rightStart = rightCheck;
                }
            } while (((upper && tempSlope >= slope) || (!upper && tempSlope < slope)) && found);
        }

        public Boolean NextCounterClockwise(List<PointF> points, PointF currentPoint, out PointF nextPoint, Boolean upper = true)
        {

            if (upper)
            {
                // Go backward because in this algorithm, we will find the point on the top of the circle,
                // so we can assume that we'll be looking for a higher x.
                for (int i = points.Count - 1; i >= 0; i--)
                {
                    PointF compare = points[i];
                    if (compare.Y > currentPoint.Y)
                    {
                        nextPoint = compare;
                        return true;
                    }
                }
            }
            else
            {
                for (int i = points.Count - 1; i >= 0; i--)
                {
                    PointF compare = points[i];
                    if (compare.Y < currentPoint.Y)
                    {
                        nextPoint = compare;
                        return true;
                    }
                }
            }
            // this means that there is none higher
            nextPoint = currentPoint;
            return false;
        }

        public Boolean NextClockwise(List<PointF> points, PointF currentPoint, out PointF nextPoint, Boolean upper = true)
        {
            // Go backward because in this algorithm, we will find the point on the top of the circle,
            // so we can assume that we'll be looking for a higher x.
            if (upper)
            {
                for (int i = 0; i < points.Count; i++)
                {
                    PointF compare = points[i];
                    if (compare.Y > currentPoint.Y)
                    {
                        nextPoint = compare;
                        return true;
                    }
                }
            }
            else
            {
                for (int i = 0; i < points.Count; i++)
                {
                    PointF compare = points[i];
                    if (compare.Y < currentPoint.Y)
                    {
                        nextPoint = compare;
                        return true;
                    }
                }
            }
            // this means that there is none higher
            nextPoint = currentPoint;
            return false;
        }

        public float Slope(PointF one, PointF two)
        {
            return (one.X - two.X) / (one.Y - two.Y);
        }

        private PointF Center(List<PointF> points)
        {
            PointF center = new PointF();
            float minX = points[0].X;
            float maxX = points[0].X;

            float minY = points[0].Y;
            float maxY = points[0].Y;

            foreach (PointF point in points)
            {
                if (point.X < minX)
                {
                    minX = point.X;
                }
                if (point.X > maxX)
                {
                    maxX = point.X;
                }

                if (point.Y < minY)
                {
                    minY = point.Y;
                }
                if (point.Y > maxY)
                {
                    maxY = point.Y;
                }
            }

            center.X = maxX - minX;
            center.Y = maxY - minY;

            return center;
        }

    }

    class HorizontalSort : IComparer<PointF>
    {
        public int Compare(PointF original, PointF comparing)
        {
            if (original.X < comparing.X)
            {
                return -1;
            }
            else if (original.X == comparing.X)
            {
                if (original.Y == comparing.Y)
                    return 0;
                else
                    return (original.Y < comparing.Y) ? -1 : 1;
            }
            else
            {
                return 1;
            }
        }
    }
}
