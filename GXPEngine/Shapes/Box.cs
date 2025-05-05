using System;
using System.Drawing;
using GXPEngine;
using GXPEngine.Core;

public class Box : EasyDraw
{
    public float boxWidth;
    public float boxHeight;

    RigidBody _rigidBody;

    Random _random = new Random();

    public float Mass()
    {
        return _rigidBody.mass;
    }

    public Box(float boxWidth, float boxHeight, Vector2 pPosition, float density, float restitution, bool isStatic = false) : base((int)boxWidth + 1, (int)boxHeight + 1)
    {
        this.boxWidth = boxWidth;
        this.boxHeight = boxHeight;

        string errorMessage;
        bool success = RigidBody.CreateBoxBody(boxWidth, boxHeight, density, isStatic, restitution, out _rigidBody, out errorMessage);
        _rigidBody.MoveTo(pPosition);
        if (!success)
        {
            Console.WriteLine("Error creating rigid body: " + errorMessage);
        }


        this.SetXY(pPosition.x, pPosition.y);

        SetOrigin(width /2, height /2);
        if (!isStatic)
        {
            Draw(_random.Next(0, 255), _random.Next(0, 255), _random.Next(0, 255));
        }
        else
        {
            DrawStatic(40, 40, 40);
        }
    }


    void Draw(int red, int green, int blue)
    {
        Fill(red, green, blue);
        Stroke(red, green, blue);
        Rect(width /2, height /2, width, height);
    }

    void DrawStatic(int red, int green, int blue)
    {
        Fill(red, green, blue);
        Stroke(Color.Red);
        Rect(boxWidth / 2, boxHeight / 2, boxWidth, boxHeight);
    }

    public void Rotate (float amount)
    {
        _rigidBody.Rotate (amount);
        //_rigidBody.transformUpdateRequire = true;
    }

    void Update()
    {
        Step();
    }

    void UpdateScreenPosition()
    {
        x = _rigidBody.position.x;
        y = _rigidBody.position.y;

        this.rotation = _rigidBody.rotation;
    }

    public void ApplyForce(Vector2 amount)
    {
        _rigidBody.ApplyForce(amount);
    }

    public void Step()
    {
        UpdateScreenPosition();
    }
}
