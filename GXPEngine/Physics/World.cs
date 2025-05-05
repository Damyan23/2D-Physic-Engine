using GXPEngine;
using GXPEngine.Core;
using System;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using System.IO;
using System.Threading;

class World
{
    public static readonly float MinBodySize = 0.01f * 0.01f;
    public static readonly float MaxBodySize = float.MaxValue;

    public static readonly float minBodyDensity = 0.000000001f;
    public static readonly float maxBodyDensity = 21.4f;

    private static List<RigidBody> bodyList = new List<RigidBody> ();
    private Vector2 gravity;

    public static readonly int minIterations = 1;
    public static readonly int maxIterations = 64;

    private List<(int, int)> contactPairs = new List<(int, int)> ();

    public List<Vector2> ContactPointList = new List<Vector2>();

    private Vector2[] contactList = new Vector2[2];
    private Vector2[] impulseList = new Vector2[2];
    private Vector2[] frictionImpulseList = new Vector2[2];
    private Vector2[] raList = new Vector2[2];
    private Vector2[] rbList = new Vector2[2];
    private float[] jList = new float[2];
    public World ()
    {
        this.gravity = new Vector2(0f, 980.7f);
    }

    // Add a body to the world
    public static void AddBody (RigidBody body)
    {
        bodyList.Add(body);
    }

    // Remove a body from the world
    public bool RemoveBody (RigidBody body) 
    {
        body.parent.LateDestroy ();
        return bodyList.Remove(body);
    }

    public void Step (float time, int totalIterations)
    {
        totalIterations = Mathf.Clamp (totalIterations, World.minIterations, World.maxIterations);
        ContactPointList.Clear();   

        for (int currentIteration = 0; currentIteration < totalIterations; currentIteration++)
        {
            this.contactPairs.Clear ();
            StepBodies(time, totalIterations);
            BroadPhase();
            NarrowPhase(currentIteration, totalIterations);
        }
    }

    public void BroadPhase ()
    {
        for (int i = 0; i < bodyList.Count - 1; i++)
        {
            RigidBody bodyA = bodyList[i];


            for (int j = i + 1; j < bodyList.Count; j++)
            {
                RigidBody bodyB = bodyList[j];

                // If both objects are static
                if (bodyA.isStatic && bodyB.isStatic)
                {
                    continue;
                }

                this.contactPairs.Add((i, j));
            }
        }
    }
        
    public void NarrowPhase (int currentIteration, int totalIterations)
    {
        for (int i = 0; i < this.contactPairs.Count; i++)
        {
            (int, int) pair = contactPairs[i];

            RigidBody bodyA = World.bodyList[pair.Item1];
            RigidBody bodyB = World.bodyList[pair.Item2];
            CollisionManifold contact = null;

            if (Collisions.Collide(bodyA, bodyB, out Vector2 normal, out float depth))
            {
                SeparateBodies(bodyA, bodyB, normal * depth);

                Collisions.FindContactPoints(bodyA, bodyB, out Vector2 contact1, out Vector2 contact2, out int contactCount);

                contact = new CollisionManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                this.ResolveCollisionBasic(contact);
            }

            if (contact != null)
            {
                if (currentIteration == totalIterations - 1)
                {
                    if (!this.ContactPointList.Contains(contact.Contact1))
                    {
                        this.ContactPointList.Add(contact.Contact1);
                    }

                    if (contact.ContactCount > 1)
                    {
                        if (!this.ContactPointList.Contains(contact.Contact2))
                        {
                            this.ContactPointList.Add(contact.Contact2);
                        }
                    }
                }
            }
        }
    }

    public void StepBodies (float time, int totalIterations)
    {
        // movement step
        for (int i = 0; i < bodyList.Count; i++)
        {
            bodyList[i].Step(time, gravity, totalIterations);
        }
    }

    public void SeparateBodies (RigidBody bodyA, RigidBody bodyB, Vector2 mtv)
    {
        // Move the objects out of each other
        if (bodyA.isStatic)
        {
            bodyB.Move(mtv);
        }
        else if (bodyB.isStatic)
        {
            bodyA.Move(-mtv);
        }
        else
        {
            bodyA.Move(-mtv / 2f);
            bodyB.Move(mtv / 2f);
        }
    }

    public void ResolveCollisionBasic(in CollisionManifold contact)
    {
        RigidBody bodyA = contact.bodyA;
        RigidBody bodyB = contact.bodyB;
        Vector2 normal = contact.normal;
        float depth = contact.depth;

        Vector2 relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

        if (Mathf.Dot(relativeVelocity, normal) > 0)
        {
            return;
        }

        float e = Mathf.Min(bodyA.restitution, bodyB.restitution);

        float j = -(1f + e) * Mathf.Dot(relativeVelocity, normal);
        j /= bodyA.invMass + bodyB.invMass;
        Vector2 impulse = j * normal;

        bodyA.LinearVelocity += -impulse * bodyA.invMass;
        bodyB.LinearVelocity += impulse * bodyB.invMass;

        // Tangent to the collision normal
        Vector2 tangent = relativeVelocity - Mathf.Dot(relativeVelocity, normal) * normal;
        if (tangent.LengthSquared() > 0.0001f)
            tangent = tangent.Normalized ();

        // Calculate magnitude of friction impulse
        float jt = -Mathf.Dot(relativeVelocity, tangent);
        jt /= bodyA.invMass + bodyB.invMass;

        // Get average friction coefficient
        float mu = Mathf.Sqrt(0.08f * 0.06f);

        // Clamp magnitude to Coulomb's law
        Vector2 frictionImpulse;
        if (Mathf.Abs(jt) < j * mu)
        {
            frictionImpulse = jt * tangent; // Static friction
        }
        else
        {
            frictionImpulse = -j * mu * tangent; // Dynamic friction
        }

        // Apply friction impulse
        bodyA.LinearVelocity -= frictionImpulse * bodyA.invMass;
        bodyB.LinearVelocity += frictionImpulse * bodyB.invMass;
    }

    public void ResolveCollisionWithRotation(in CollisionManifold contact)
    {
        RigidBody bodyA = contact.bodyA;
        RigidBody bodyB = contact.bodyB;
        Vector2 normal = contact.normal;
        Vector2 contact1 = contact.Contact1;
        Vector2 contact2 = contact.Contact2;
        int contactCount = contact.ContactCount;

        float e = Mathf.Min(bodyA.restitution, bodyB.restitution);

        this.contactList[0] = contact1;
        this.contactList[1] = contact2;

        for (int i = 0; i < contactCount; i++)
        {
            this.impulseList[i] = new Vector2();
            this.raList[i] = new Vector2 ();
            this.rbList[i] = new Vector2 ();
        }

        for (int i = 0; i < contactCount; i++)
        {
            Vector2 ra = contactList[i] - bodyA.position;
            Vector2 rb = contactList[i] - bodyB.position;

            raList[i] = ra;
            rbList[i] = rb;

            Vector2 raPerp = new Vector2(-ra.y, ra.x);
            Vector2 rbPerp = new Vector2(-rb.y, rb.x);

            Vector2 angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
            Vector2 angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

            Vector2 relativeVelocity =
                (bodyB.LinearVelocity + angularLinearVelocityB) -
                (bodyA.LinearVelocity + angularLinearVelocityA);

            float contactVelocityMag = Mathf.Dot(relativeVelocity, normal);

            if (contactVelocityMag > 0f)
            {
                continue;
            }

            float raPerpDotN = Mathf.Dot(raPerp, normal);
            float rbPerpDotN = Mathf.Dot(rbPerp, normal);

            float denom = bodyA.invMass + bodyB.invMass +
                (raPerpDotN * raPerpDotN) * bodyA.invInertia +
                (rbPerpDotN * rbPerpDotN) * bodyB.invInertia;

            float j = -(1f + e) * contactVelocityMag;
            j /= denom;
            j /= (float)contactCount;

            Vector2 impulse = j * normal;
            impulseList[i] = impulse;
        }

        for (int i = 0; i < contactCount; i++)
        {
            Vector2 impulse = impulseList[i];
            Vector2 ra = raList[i];
            Vector2 rb = rbList[i];

            bodyA.LinearVelocity += -impulse * bodyA.invMass;
            bodyA.AngularVelocity += -Vector2.Cross(ra, impulse) * bodyA.invInertia;
            bodyB.LinearVelocity += impulse * bodyB.invMass;
            bodyB.AngularVelocity += Vector2.Cross(rb, impulse) * bodyB.invInertia;
        }
    }

    public void ResolveCollisionWithRotationAndFriction(in CollisionManifold contact)
    {
        RigidBody bodyA = contact.bodyA;
        RigidBody bodyB = contact.bodyB;
        Vector2 normal = contact.normal;
        Vector2 contact1 = contact.Contact1;
        Vector2 contact2 = contact.Contact2;
        int contactCount = contact.ContactCount;

        float e = Mathf.Min(bodyA.restitution, bodyB.restitution);

        float sf = (bodyA.staticFriction + bodyB.staticFriction) / 2;
        float df = (bodyA.dynamicFriction + bodyB.dynamicFriction) / 2;

        this.contactList[0] = contact1;
        this.contactList[1] = contact2;

        for (int i = 0; i < contactCount; i++)
        {
            this.impulseList[i] = new Vector2();
            this.frictionImpulseList[i] = new Vector2();
            this.raList[i] = new Vector2();
            this.rbList[i] = new Vector2();
            this.jList[i] = 0f;
        }

        for (int i = 0; i < contactCount; i++)
        {
            Vector2 ra = contactList[i] - bodyA.position;
            Vector2 rb = contactList[i] - bodyB.position;

            raList[i] = ra;
            rbList[i] = rb;

            Vector2 raPerp = new Vector2(-ra.y, ra.x);
            Vector2 rbPerp = new Vector2(-rb.y, rb.x);

            Vector2 angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
            Vector2 angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

            Vector2 relativeVelocity =
                (bodyB.LinearVelocity + angularLinearVelocityB) -
                (bodyA.LinearVelocity + angularLinearVelocityA);

            float contactVelocityMag = Mathf.Dot(relativeVelocity, normal);

            if (contactVelocityMag > 0f)
            {
                continue;
            }

            float raPerpDotN = Mathf.Dot(raPerp, normal);
            float rbPerpDotN = Mathf.Dot(rbPerp, normal);

            float denom = bodyA.invMass + bodyB.invMass +
                (raPerpDotN * raPerpDotN) * bodyA.invInertia +
                (rbPerpDotN * rbPerpDotN) * bodyB.invInertia;

            float j = -(1f + e) * contactVelocityMag;
            j /= denom;
            j /= (float)contactCount;

            jList[i] = j;

            Vector2 impulse = j * normal;
            impulseList[i] = impulse;
        }

        for (int i = 0; i < contactCount; i++)
        {
            Vector2 impulse = impulseList[i];
            Vector2 ra = raList[i];
            Vector2 rb = rbList[i];

            bodyA.LinearVelocity += -impulse * bodyA.invMass;
            bodyA.AngularVelocity += -Vector2.Cross(ra, impulse) * bodyA.invInertia;
            bodyB.LinearVelocity += impulse * bodyB.invMass;
            bodyB.AngularVelocity += Vector2.Cross(rb, impulse) * bodyB.invInertia;
        }

        for (int i = 0; i < contactCount; i++)
        {
            Vector2 ra = contactList[i] - bodyA.position;
            Vector2 rb = contactList[i] - bodyB.position;

            raList[i] = ra;
            rbList[i] = rb;

            Vector2 raPerp = new Vector2(-ra.y, ra.x);
            Vector2 rbPerp = new Vector2(-rb.y, rb.x);

            Vector2 angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
            Vector2 angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

            Vector2 relativeVelocity =
                (bodyB.LinearVelocity + angularLinearVelocityB) -
                (bodyA.LinearVelocity + angularLinearVelocityA);

            Vector2 tangent = relativeVelocity - Mathf.Dot(relativeVelocity, normal) * normal;

            if (Vector2.NearlyEqual (tangent, new Vector2()))
            {
                continue;
            }
            else
            {
                tangent.Normalize();
            }

            float raPerpDotT = Mathf.Dot(raPerp, tangent);
            float rbPerpDotT = Mathf.Dot(rbPerp, tangent);

            float denom = bodyA.invMass + bodyB.invMass +
                (raPerpDotT * raPerpDotT) * bodyA.invInertia +
                (rbPerpDotT * rbPerpDotT) * bodyB.invInertia;

            float jt = -Mathf.Dot(relativeVelocity, tangent); ;
            jt /= denom;
            jt /= (float)contactCount;


            float j = jList[i];
            Vector2 firctionImpulse = new Vector2();

            if (Mathf.Abs(jt) <= j * sf)
            {
                firctionImpulse = jt * tangent;
            }
            else
            {
                firctionImpulse = -j * tangent * df;
            }


            frictionImpulseList[i] = firctionImpulse;
        }

        for (int i = 0; i < contactCount; i++)
        {
            Vector2 firctionImpulse = frictionImpulseList[i];
            Vector2 ra = raList[i];
            Vector2 rb = rbList[i];

            bodyA.LinearVelocity += -firctionImpulse * bodyA.invMass;
            bodyA.AngularVelocity += -Vector2.Cross(ra, firctionImpulse) * bodyA.invInertia;
            bodyB.LinearVelocity += firctionImpulse * bodyB.invMass;
            bodyB.AngularVelocity += Vector2.Cross(rb, firctionImpulse) * bodyB.invInertia;
        }
    }
}
