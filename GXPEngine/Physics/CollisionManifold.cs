// Contains all the info we need to resolve the collision between two bodies. 

using GXPEngine.Core;

class CollisionManifold
{
    public readonly RigidBody bodyA;
    public readonly RigidBody bodyB;
    public readonly Vector2 normal;
    public readonly float depth;
    public readonly Vector2 Contact1;
    public readonly Vector2 Contact2;
    public readonly int ContactCount;

    public CollisionManifold (RigidBody bodyA, RigidBody bodyB, Vector2 normal, 
                                float depth, Vector2 pointOfContact1, Vector2 pointOfContact2, 
                                int pointsOfContactCount)
    {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.normal = normal;
        this.depth = depth;
        Contact1 = pointOfContact1;
        Contact2 = pointOfContact2;
        ContactCount = pointsOfContactCount;
    }
}