using GXPEngine;
using GXPEngine.Core;
using System;
using System.Runtime.Remoting.Messaging;

public static class Collisions
{

    public static void PointSegmentDistance (Vector2 p, Vector2 a, Vector2 b, out float distanceSquared, out Vector2 cp)
    {
        Vector2 ab = b - a;
        Vector2 ap = p - a;

        float proj = Mathf.Dot(ap, ab);
        float abLenSq = ab.LengthSquared();
        float d = proj / abLenSq;

        if (d <= 0f)
        {
            cp = a;
        }
        else if (d >= 1f)
        {
            cp = b;
        }
        else
        {
            cp = a + ab * d; 
        }

        distanceSquared = Mathf.DistanceSquared(p, cp);
    }
    public static void FindContactPoints (RigidBody bodyA, RigidBody bodyB, out Vector2 contact1, out Vector2 contact2, out int contactCount)
    {
        contact1 = new Vector2 ();
        contact2 = new Vector2 ();
        contactCount = 0;

        ShapeType shapeTypeA = bodyA.shapeType;
        ShapeType shapeTypeB = bodyB.shapeType;

        if (shapeTypeA is ShapeType.Box)
        {
            if (shapeTypeB is ShapeType.Box)
            {
                Collisions.FindPolygonsContactPoints(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(), out contact1, out contact2, out contactCount); 
            }
            else if (shapeTypeB is ShapeType.Circle)
            {
                Collisions.FindCirclePolygonContactPoint(bodyB.position, bodyB.radius, bodyA.position, bodyA.GetTransformedVertices(), out contact1);
                contactCount = 1;
            }
        }
        else if (shapeTypeA is ShapeType.Circle)
        {
            if (shapeTypeB is ShapeType.Box)
            {
                Collisions.FindCirclePolygonContactPoint(bodyA.position, bodyA.radius, bodyB.position, bodyB.GetTransformedVertices(), out contact1);
                contactCount = 1;
            }
            else if (shapeTypeB is ShapeType.Circle)
            {
                Collisions.FindCirclesContactPoints(bodyA.position, bodyA.radius, bodyB.position, out contact1);
                contactCount = 1;
            }
        }
    }

    private static void FindPolygonsContactPoints(Vector2[] verteciesA, Vector2[] verteciesB, out Vector2 contact1, out Vector2 contact2, out int contactCount)
    {
        contact1 = new Vector2();
        contact2 = new Vector2();
        contactCount = 0;

        float minDistSqr = float.MaxValue;
        for (int i = 0; i < verteciesA.Length; i++)
        {
            Vector2 p = verteciesA[i];

            for (int j = 0; j < verteciesB.Length; j++)
            {
                Vector2 va = verteciesB[j];
                Vector2 vb = verteciesB[(j + 1) % verteciesB.Length];

                Collisions.PointSegmentDistance(p, va, vb, out float distSqr, out Vector2 cp);

                if (Mathf.NearlyEqual(distSqr, minDistSqr))
                {
                    if (!Vector2.NearlyEqual (cp, contact1))
                    {
                        contact2 = cp;
                        contactCount = 2;
                    }
                }
                else if (distSqr < minDistSqr)
                {
                    minDistSqr = distSqr;
                    contact1 = cp;
                    contactCount = 1;
                }
            }
        }

        for (int i = 0; i < verteciesB.Length; i++)
        {
            Vector2 p = verteciesB[i];

            for (int j = 0; j < verteciesA.Length; j++)
            {
                Vector2 va = verteciesA[j];
                Vector2 vb = verteciesB[(j + 1) % verteciesB.Length];

                Collisions.PointSegmentDistance(p, va, vb, out float distSqr, out Vector2 cp);

                if (Mathf.NearlyEqual(distSqr, minDistSqr))
                {
                    if (!Vector2.NearlyEqual(cp, contact1))
                    {
                        contact2 = cp;
                        contactCount = 2;
                    }
                }
                else if (distSqr < minDistSqr)
                {
                    minDistSqr = distSqr;
                    contact1 = cp;
                    contactCount = 1;
                }
            }
        }
    }
    private static void FindCirclesContactPoints(Vector2 centerA, float raduisA, Vector2 centerB, out Vector2 cp)
    {
        Vector2 ab = centerB - centerA;
        Vector2 direction = ab.Normalized();

        cp = centerA + direction * raduisA;
    }

    private static void FindCirclePolygonContactPoint(Vector2 circleCenter, float circleRadius, Vector2 polygonCenter, Vector2[] polygonVertecies, out Vector2 cp)
    {
        cp = new Vector2();

        float minDistSqr = float.MaxValue;

        for (int i = 0; i < polygonVertecies.Length; i++)
        {
            Vector2 va = polygonVertecies[i];
            Vector2 vb = polygonVertecies[(i + 1) % polygonVertecies.Length];

            Collisions.PointSegmentDistance(circleCenter, va, vb, out float distSqr, out Vector2 contact);

            if (distSqr < minDistSqr)
            {
                minDistSqr = distSqr;
                cp = contact;
            }
        }
    }

    public static bool Collide(RigidBody bodyA, RigidBody bodyB, out Vector2 normal, out float depth)
    {
        normal = new Vector2();
        depth = 0f;

        ShapeType shapeTypeA = bodyA.shapeType;
        ShapeType shapeTypeB = bodyB.shapeType;

        if (shapeTypeA is ShapeType.Box)
        {
            if (shapeTypeB is ShapeType.Box)
            {
                return Collisions.IntersectPolygons(bodyA.position,
                        bodyA.GetTransformedVertices(),
                        bodyB.position, bodyB.GetTransformedVertices(),
                        out normal, out depth);
            }
            else if (shapeTypeB is ShapeType.Circle)
            {
                bool result = Collisions.IntersectCirlcePolygon(bodyB.position,
                                bodyB.radius, bodyA.position,
                                bodyA.GetTransformedVertices(), out normal, out depth);

                // reverse the normal since i want to pull bodyA out of bodyB
                normal = -normal;

                return result;
            }
        }
        else if (shapeTypeA is ShapeType.Circle)
        {
            if (shapeTypeB is ShapeType.Box)
            {
                return Collisions.IntersectCirlcePolygon(bodyA.position, bodyA.radius, bodyB.position, bodyB.GetTransformedVertices(), out normal, out depth);
            }
            else if (shapeTypeB is ShapeType.Circle)
            {
                return Collisions.IntersectCircles(bodyA.position, bodyA.radius, bodyB.position, bodyB.radius, out normal, out depth);
            }
        }

        return false;
    }


    // Used if the center of the polygons are passed
    public static bool IntersectPolygons(Vector2 centerA ,Vector2[] verticesA, Vector2 centerB, Vector2[] verticesB, out Vector2 normal, out float depth)
    {
        // Initialize the normal vector and depth to default values
        normal = new Vector2();
        depth = float.MaxValue;

        // Loop through each vertex of polygon A
        for (int i = 0; i < verticesA.Length; i++)
        {
            // Get the current vertex and the next vertex to form an edge
            Vector2 vectorA = verticesA[i];
            Vector2 vectorB = verticesA[(i + 1) % verticesA.Length];

            // Compute the edge vector from vertex A to vertex B
            Vector2 edge = vectorB - vectorA;

            // Compute the axis perpendicular to the edge (normal of each edge)
            Vector2 axis = new Vector2(-edge.y, edge.x);
            axis.Normalize();

            // Project vertices of both polygons onto this axis and get the minimum and maximum projections
            Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
            Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

            // Check for overlap between the projections
            if (minA >= maxB || minB >= maxA)
            {
                // If there is no overlap, polygons are separated along this axis, return false
                return false;
            }

            // Compute the penetration depth along this axis
            float axisDepth = Mathf.Min(maxB - minA, maxA - minB);

            // If this is the smallest penetration depth so far, update normal and depth
            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }
        }

        // Repeat the same process for each edge of polygon B
        for (int i = 0; i < verticesB.Length; i++)
        {
            // Get the current vertex and the next vertex to form an edge for polygon B
            Vector2 vectorA = verticesB[i];
            Vector2 vectorB = verticesB[(i + 1) % verticesB.Length];

            // Compute the edge vector from vertex A to vertex B
            Vector2 edge = vectorB - vectorA;

            // Compute the axis perpendicular to the edge (normal of each edge)
            Vector2 axis = new Vector2(-edge.y, edge.x);
            axis.Normalize();

            // Project vertices of both polygons onto this axis and get the minimum and maximum projections
            Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
            Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

            // Check for overlap between the projections
            if (minA >= maxB || minB >= maxA)
            {
                // If there is no overlap, polygons are separated along this axis, return false
                return false;
            }

            // Compute the penetration depth along this axis
            float axisDepth = Mathf.Min(maxB - minA, maxA - minB);

            // If this is the smallest penetration depth so far, update normal and depth
            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }
        }

        // Compute the direction vector from centerA to centerB
        Vector2 direction = centerB - centerA;

        // Check if the direction vector aligns with the normal vector
        // If not, flip the normal vector
        if (Mathf.Dot(direction, normal) < 0f)
        {
            normal = -normal;
        }

        // If no separation along any axis, polygons intersect, return true
        return true;
    }

    private static void ProjectVertices(Vector2[] vertices, Vector2 axis, out float min, out float max)
    {
        // Initialize min and max projection values
        min = float.MaxValue;
        max = float.MinValue;

        // Loop through each vertex of the polygon
        for (int i = 0; i < vertices.Length; i++)
        {
            // Get the current vertex
            Vector2 v = vertices[i];

            // Project the vertex onto the separation axis
            float proj = Mathf.Dot(v, axis);

            // Update min and max projection values
            if (proj < min) { min = proj; } // Update min if the projection is smaller
            if (proj > max) { max = proj; } // Update max if the projection is larger
        }
    }

    public static bool IntersectCircles(Vector2 centerA, float radiusA, Vector2 centerB, float radiusB, out Vector2 normal, out float depth)
    {
        // Initialize normal vector and depth
        normal = new Vector2();
        depth = 0;

        // Calculate distance between the centers of the circles
        float distance = Mathf.Distance(centerA, centerB);

        // Calculate the combined radius of both circles
        float radiusOfBothCircles = radiusA + radiusB;

        // Check if the distance between the centers is greater than or equal to the sum of the radii
        if (distance >= radiusOfBothCircles)
        {
            // If the circles are not overlapping, return false
            return false;
        }

        // Calculate the normal vector pointing from circle A to circle B
        normal = centerB - centerA;
        normal.Normalize();

        // Calculate the penetration depth (overlap) between the circles
        depth = radiusOfBothCircles - distance;

        // Circles are intersecting, return true
        return true;
    }


    // Used if the center of the polygon is passed
    public static bool IntersectCirlcePolygon(Vector2 circleCenter, float circleRadius, Vector2 polygonCenter, Vector2[] vertices, out Vector2 normal, out float depth)
    {
        // Initialize the normal vector and depth value
        normal = new Vector2();
        depth = float.MaxValue;

        // Temporary variables to store axis, axis depth, and projection values
        Vector2 axis = new Vector2();
        float axisDepth = 0;
        float minA, maxA, minB, maxB = 0;

        // Loop through each edge of the polygon
        for (int i = 0; i < vertices.Length; i++)
        {
            // Get current and next vertices of the polygon to define an edge
            Vector2 vectorA = vertices[i];
            Vector2 vectorB = vertices[(i + 1) % vertices.Length];

            // Calculate the edge vector and its normal (perpendicular) axis
            Vector2 edge = vectorB - vectorA;
            axis = new Vector2(-edge.y, edge.x);
            axis.Normalize();

            // Project vertices of the polygon and the circle onto the separation axis
            Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
            Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

            // Check for overlap between the projections
            if (minA >= maxB || minB >= maxA)
            {
                // If there is no overlap, polygons are separated along this axis, return false
                return false;
            }

            // Calculate the depth of penetration along the separation axis
            axisDepth = Mathf.Min(maxB - minA, maxA - minB);

            // Update the minimum depth and corresponding normal if the depth is smaller
            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }
        }

        // Find the closest point on the polygon to the circle's center
        int cpIndex = Collisions.FindClosestPointToPolygon(circleCenter, vertices);
        Vector2 cp = vertices[cpIndex];

        // Calculate the axis towards the closest point
        axis = cp - circleCenter;
        axis.Normalize();

        // Project vertices of the polygon and the circle onto the separation axis
        Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
        Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

        // Check for overlap between the projections
        if (minA >= maxB || minB >= maxA)
        {
            // If there is no overlap, polygons are separated along this axis, return false
            return false;
        }

        // Calculate the depth of penetration along the separation axis
        axisDepth = Mathf.Min(maxB - minA, maxA - minB);

        // Update the minimum depth and corresponding normal if the depth is smaller
        if (axisDepth < depth)
        {
            depth = axisDepth;
            normal = axis;
        }

        // Adjust the normal vector based on the direction from the circle's center to the polygon's center
        Vector2 direction = polygonCenter - circleCenter;
        if (Mathf.Dot(direction, normal) < 0f)
        {
            normal = -normal;
        }

        // Collision detected, return true
        return true;
    }

    private static int FindClosestPointToPolygon(Vector2 circleCenter, Vector2[] vertices)
    {
        // Initialize variables to store the index of the closest point and the minimum distance
        int result = -1;               // Index of the closest point (initialized to -1)
        float minDistance = float.MaxValue;  // Minimum distance (initialized to maximum possible value)

        // Loop through each vertex of the polygon
        for (int i = 0; i < vertices.Length; i++)
        {
            // Get the current vertex
            Vector2 v = vertices[i];

            // Calculate the distance between the current vertex and the circle's center
            float distance = Mathf.Distance(v, circleCenter);

            // Check if the current distance is smaller than the minimum distance found so far
            if (distance < minDistance)
            {
                // Update the minimum distance and the index of the closest point
                minDistance = distance;
                result = i;
            }
        }

        // Return the index of the closest point
        return result;
    }


    private static void ProjectCircle(Vector2 center, float radius, Vector2 axis, out float min, out float max)
    {
        // Normalize the axis to ensure it has unit length
        Vector2 direction = axis.Normalized();

        // Calculate the vector representing the direction of the axis multiplied by the radius of the circle
        Vector2 directionAndRadius = direction * radius;

        // Calculate the two points on the circle's circumference projected onto the axis
        Vector2 p1 = center + directionAndRadius;  // Point at the positive end of the axis
        Vector2 p2 = center - directionAndRadius;  // Point at the negative end of the axis

        // Project the points onto the axis and assign them to min and max
        min = Mathf.Dot(p1, axis);  // Projection of the positive point onto the axis
        max = Mathf.Dot(p2, axis);  // Projection of the negative point onto the axis

        // Ensure that min and max are ordered correctly (min should be less than or equal to max)
        if (min > max)
        {
            // Swap the values if min is greater than max
            float t = min;
            min = max;
            max = t;
        }
    }

}