using System;
using System.Runtime.Remoting.Messaging;

namespace GXPEngine.Core
{
	public struct Vector2
	{
        public float x;
        public float y;

        public Vector2(float pX = 0, float pY = 0)
        {
            x = pX;
            y = pY;
        }

        public void SetXY(float x, float y)
        {
            this.SetXY(x, y);
        }

        public float Length()
        {
            // TODO: return the vector length
            return Mathf.Sqrt(x * x + y * y);
        }

        public float LengthSquared ()
        {
            return x * x + y * y;
        }

        public void Normalize()
        {
            float length = Length();
            if (length != 0)
            {
                x /= length;
                y /= length;
            }
        }

        public Vector2 Normalized()
        {
            float length = Length();
            return new Vector2(x / length, y / length);
        }

        // TODO: Implement subtract, scale operators

        public static Vector2 operator +(Vector2 left, Vector2 right)
        {
            return new Vector2(left.x + right.x, left.y + right.y);
        }

        public static Vector2 operator -(Vector2 left, Vector2 right)
        {
            return new Vector2(left.x - right.x, left.y - right.y);
        }

        public static Vector2 operator *(Vector2 left, float scaleFactor)
        {
            return new Vector2(left.x * scaleFactor, left.y * scaleFactor);
        }
        public static Vector2 operator *(float scaleFactor, Vector2 left)
        {
            return new Vector2(left.x * scaleFactor, left.y * scaleFactor);
        }

        public static Vector2 operator / (Vector2 left, float scaleFactor)
        {
            return new Vector2(left.x / scaleFactor, left.y / scaleFactor);
        }

        public static Vector2 operator -(Vector2 vector)
        {
            return new Vector2(-vector.x, -vector.y);
        }

        public static Vector2 Transform (Vector2 vector, FlatTransform transform)
        {
            // Formulas for 2d rotation
            // x?=xcos(?)–ysin(?)
            // y?=xsin(?)+ycos(?)

            return new Vector2 (
                transform.Cos * vector.x - transform.Sin * vector.y + transform.PositionX,
                transform.Sin * vector.x + transform.Cos * vector.y + transform.PositionY);
        }

        public override string ToString()
        {
            return String.Format("({0},{1})", x, y);
        }

        public static bool NearlyEqual (Vector2 a, Vector2 b)
        {
            return Mathf.DistanceSquared (a, b) < Mathf.VerySmallAmount * Mathf.VerySmallAmount;
        }

        public static float Cross(Vector2 a, Vector2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

    }
}

