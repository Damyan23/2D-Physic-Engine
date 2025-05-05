using GXPEngine;
using GXPEngine.Core;
using System;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;

public enum ShapeType
{
    Circle = 0,
    Box = 1
}

public class RigidBody : GameObject
{
    public Vector2 position;
    public Vector2 linearVelocity;
    public float angle;
    public float angularVelocity;

    private Vector2 force;

    public readonly ShapeType shapeType;    
    public readonly float density;
    public readonly float mass;
    public readonly float invMass;
    public readonly float restitution;
    public readonly float area;
    public readonly float inertia;
    public readonly float invInertia;
    public readonly float gravity;
    public readonly  bool isStatic;
    public readonly float radius;
    public readonly float width;
    public readonly float height;
    public readonly float staticFriction;
    public readonly float dynamicFriction;

    private readonly Vector2[] vertices;
    // A array to store the transformed vertices since if we transfrom the already transformed ones, the transformation wont be correct
    private Vector2[] transformedVertices;

    public bool transformUpdateRequire;
    public Vector2 LinearVelocity
    {
        get { return this.linearVelocity; }
        internal set { this.linearVelocity = value; }
    }

    public float AngularVelocity
    {
        get { return this.angularVelocity; }
        internal set { this.angularVelocity = value; }
    }

    private RigidBody(float density, float mass, float inertia, float restitution, float area,
                        bool isStatic, float radius, float width, float height, Vector2[]vertecies, ShapeType shapeType) : base ()
    {
        this.position = new Vector2 ();
        this.linearVelocity = new Vector2 ();
        this.rotation = 0f;
        this.angularVelocity = 0f;

        this.force = new Vector2 ();

        this.density = density;
        this.mass = mass;
        this.invMass = mass > 0f ? 1f / mass : 0f;
        this.inertia = inertia;
        this.invInertia = inertia > 0f ? 1f / inertia : 0f;
        this.restitution = restitution;
        this.area = area;
        this.isStatic = isStatic;
        this.shapeType = shapeType;
        this.radius = radius;
        this.width = width;
        this.height = height;
        this.staticFriction = 0.6f;
        this.dynamicFriction = 0.4f;


        if (shapeType is ShapeType.Box)
        {
            this.vertices = vertecies;
            this.transformedVertices = new Vector2[4];
        }
        else
        {
            this.vertices = null;
            this.transformedVertices = null;
        }

        this.transformUpdateRequire = true;
    }

    public static Vector2[] CreateRectVertices(float width, float height)
    {
        float left = -width / 2f;
        float right = left + width;
        float bottom = -height / 2;
        float top = bottom + height;

        Vector2[] vertices = new Vector2[4];
        vertices[0] = new Vector2(left, bottom);
        vertices[1] = new Vector2(left, top);
        vertices[2] = new Vector2(right, top);
        vertices[3] = new Vector2(right, bottom);

        return vertices;
    }

    //If has to be updated, loops trough all vertecies, and then transforms one and puts in ito the transformed vertices array
    public Vector2[] GetTransformedVertices()
    {
        if (this.transformUpdateRequire)
        {
            FlatTransform transform = new FlatTransform(position, rotation);

            for (int i = 0; i < this.vertices.Length; i++)
            {
                Vector2 v = this.vertices[i];
                this.transformedVertices[i] = Vector2.Transform(v, transform);
            }
        }

        this.transformUpdateRequire = false;

        return this.transformedVertices;
    }

    //public Vector2[] GetTransformedVertices()
    //{
    //    // Ensure that the object has a parent
    //    if (parent != null)
    //    {
    //        // Get the parent's position, width, and height
    //        float parentX;
    //        float parentY;
    //        float parentWidth = this.width;
    //        float parentHeight = this.height;

    //        MyGame myGame = MyGame.Game;

    //        if (parent.parent != null && parent.parent != myGame)
    //        {
    //            parentX = parent.parent.x - parent.x;
    //            parentY = parent.parent.y - parent.y;

    //        }
    //        else
    //        {
    //            parentX = parent.x;
    //            parentY = parent.y;
    //        }

    //        // Calculate the corners of the parent object relative to its center
    //        Vector2[] parentCorners = new Vector2[4];
    //        parentCorners[0] = new Vector2(-parentWidth / 2, -parentHeight / 2); // Top-left corner
    //        parentCorners[1] = new Vector2(parentWidth / 2, -parentHeight / 2); // Top-right corner
    //        parentCorners[2] = new Vector2(parentWidth / 2, parentHeight / 2); // Bottom-right corner
    //        parentCorners[3] = new Vector2(-parentWidth / 2, parentHeight / 2); // Bottom-left corner

    //        // Apply the parent's rotation to the corners
    //        float parentRotation = parent.rotation * Mathf.PI / 180.0f;
    //        for (int i = 0; i < parentCorners.Length; i++)
    //        {
    //            float rotatedX = parentCorners[i].x * Mathf.Cos(parentRotation) - parentCorners[i].y * Mathf.Sin(parentRotation);
    //            float rotatedY = parentCorners[i].x * Mathf.Sin(parentRotation) + parentCorners[i].y * Mathf.Cos(parentRotation);
    //            parentCorners[i] = new Vector2(rotatedX, rotatedY);
    //        }

    //        // Calculate the transformed vertices by adding the parent's position
    //        for (int i = 0; i < parentCorners.Length; i++)
    //        {
    //            vertices[i] = new Vector2(parentX + parentCorners[i].x, parentY + parentCorners[i].y);
    //            this.transformedVertices[i] = vertices[i];
    //        }

    //        // Indicate that the vertices have been updated
    //        this.transformUpdateRequire = true;
    //    }
    //    return this.transformedVertices;
    //}

    public static bool CreateBoxBody(float width, float height, float density, bool isStatic, float restitution, out RigidBody body, out string erroMassage)
    {
        body = null;
        erroMassage = string.Empty;

        float area = width * height;

        if (area < World.MinBodySize)
        {
            erroMassage = "Area is too small";
            return false;
        }
        else if (area > World.MaxBodySize)
        {
            erroMassage = "Area radius is too big";
            return true;
        }

        if (density < World.minBodyDensity)
        {
            erroMassage = "The density is too small";
            return false;
        }
        else if (density > World.maxBodyDensity)
        {
            erroMassage = "The density is too big";
            return false;
        }

        restitution = Mathf.Clamp(restitution, 0.0f, 1.0f);

        float mass = 0;
        float inertia = 0;


        if (!isStatic)
        {
            mass = area * density;
            inertia = (1f / 12) * mass * (height * height + width * width);
        }

        Vector2[] vertecies = RigidBody.CreateRectVertices(width, height); 

        body = new RigidBody(density, mass, inertia, restitution, area, isStatic, 0f, width, height, vertecies, ShapeType.Box);
        World.AddBody(body);
        return true;
    }

    public static bool CreateCircleBody(float radius, float density, bool isStatic, float restitution, out RigidBody body, out string erroMassage)
    {
        body = null;
        erroMassage = string.Empty;

        float area = radius  * radius * Mathf.PI;

        if (area < World.MinBodySize) 
        {
            erroMassage = "Curcle radius is too small";
            return false;
        } else if (area > World.MaxBodySize) 
        {
            erroMassage = "Circle radius is too big";
            return true;
        }

        if (density < World.minBodyDensity)
        {
            erroMassage = "The density is too small";
            return false;
        } else if (density > World.maxBodyDensity) 
        {
            erroMassage = "The density is too big";
            return false;
        }

        restitution = Mathf.Clamp (restitution, 0.0f, 1.0f);

        float mass = 0f;
        float inertia = 0f;

        if (!isStatic)
        {
            mass = area * density;
            inertia = (1f / 2) * mass * radius * radius;
        }

        body = new RigidBody(density, mass, inertia, restitution, area, isStatic, radius, 0f, 0f, null, ShapeType.Circle);
        World.AddBody(body);
        return true;
    }

    public void MoveTo (Vector2 position)
    {
        this.position = position;
        this.transformUpdateRequire = true;
    }

    public void Move (Vector2 moveBy)
    {
        this.position += moveBy;
        //this.transformUpdateRequire = true;
    }

    public void Rotate(float amount)
    {
        this.rotation += amount;
        this.transformUpdateRequire = true;
    }

    public void RotateTo(float angle)
    {
        this.angle = angle;
        this.transformUpdateRequire = true;
    }

    public void Step (float time, Vector2 gravity, int iterations)
    {
        if (this.isStatic) 
        {
            return;
        }

        time /= (float)iterations;

        this.linearVelocity += gravity * time;

        this.position += this.linearVelocity * time;

        this.rotation += this.angularVelocity * time;

        this.force = new Vector2(0, 0);

        this.transformUpdateRequire = true;
    }

    public void ApplyForce (Vector2 amount) 
    {
        this.force = amount;    
        this.transformUpdateRequire = true;
    }


}
