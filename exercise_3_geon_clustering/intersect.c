// Find the points where the two circles intersect.
int FindCircleCircleIntersections(
    float cx0, float cy0, float radius0,
    float cx1, float cy1, float radius1)
{
    // Find the distance between the centers.
    float dx = cx0 - cx1;
    float dy = cy0 - cy1;
    double dist = sqrt(dx * dx + dy * dy);

    // See how many solutions there are.
    if (dist > radius0 + radius1)
    {
        // No solutions, the circles are too far apart.
        //intersection1 = new PointF(float.NaN, float.NaN);
        //intersection2 = new PointF(float.NaN, float.NaN);
        return 0;
    }
    else if (dist < fabsolute(radius0 - radius1))
    {
        // No solutions, one circle contains the other.
        //intersection1 = new PointF(float.NaN, float.NaN);
        //intersection2 = new PointF(float.NaN, float.NaN);
        return 0;
    }
    else if ((dist == 0) && (radius0 == radius1))
    {
        // No solutions, the circles coincide.
        //intersection1 = new PointF(float.NaN, float.NaN);
        //intersection2 = new PointF(float.NaN, float.NaN);
        return 0;
    }
    else
    {
        // Find a and h.
        double a = (radius0 * radius0 -
            radius1 * radius1 + dist * dist) / (2 * dist);
        double h = Math.Sqrt(radius0 * radius0 - a * a);

        // Find P2.
        double cx2 = cx0 + a * (cx1 - cx0) / dist;
        double cy2 = cy0 + a * (cy1 - cy0) / dist;

        // Get the points P3.
        intersect[intersect_counter].x = (int)(cx2 + h * (cy1 - cy0) / dist);
        intersect[intersect_counter].y = (int)(cy2 - h * (cx1 - cx0) / dist);
        intersect_counter++;
        if (dist == radius0 + radius1) return 1;
        intersect[intersect_counter].x = (int)(cx2 - h * (cy1 - cy0) / dist);
        intersect[intersect_counter].y = (int)(cy2 + h * (cx1 - cx0) / dist);
        intersect_counter++;
        // // See if we have 1 or 2 solutions.
        // if (dist == radius0 + radius1) return 1;
        return 2;
    }
}
