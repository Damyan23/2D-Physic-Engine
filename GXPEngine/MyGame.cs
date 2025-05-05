using System;
using GXPEngine;
using System.Drawing;
using GXPEngine.Core;
using System.Collections.Generic;
using GXPEngine.EasyDraw_Shapes;

public class MyGame : Game
{
    static void Main()
    {
        new MyGame().Start();
    }

    public static MyGame Game;

    Ball _ball1;

    Box ground ;
    Box slope1;
    Box slope2;

    EasyDraw _text;

    World world;

    Random rand;

    List<RectangleD> drawnContactPoints = new List<RectangleD>();
    bool debugMode = false;

    public MyGame() : base(800, 600, false, false)
    {
        Game = this;
        SetUp ();
    }

    void SetUp()
    {
        world = new World();
        rand = new Random();

        ground = new Box(width - 100, 60, new Vector2(width / 2, height - 80), 1f, 0.5f, true);
        AddChild(ground);  
    }


    void Update()
    {
        this.targetFps = 60;

        world.Step(Time.deltaTimeInSeconds, 20);

        //float dx = 0f;
        //float dy = 0f;
        //float forceMagnitude = 500;

        //if (Input.GetKey(Key.A)) { dx--; }
        //else if (Input.GetKey(Key.D)) { dx++; }

        //if (Input.GetKey(Key.W)) { dy--; }
        //else if (Input.GetKey(Key.S)) { dy++; }

        //if (dx != 0f || dy != 0f)
        //{
        //    Vector2 forceDirection = new Vector2(dx, dy).Normalized();

        //    Vector2 force = forceDirection * forceMagnitude;

        //    box1.ApplyForce(force);
        //}
        if (debugMode) drawContactPoints();

        if (Input.GetMouseButtonDown (0))
        {
            Ball ball = new Ball (rand.Next(15,40), new Vector2 (Input.mouseX, Input.mouseY), 0.1f, 1f);
            AddChild(ball);
        }
        else if (Input.GetMouseButtonDown(1)) 
        {
            Box box = new Box(rand.Next(15, 40), rand.Next (15, 40), new Vector2(Input.mouseX, Input.mouseY), 0.0005f, 0.5f);
            AddChild(box);
        }

        if (Input.GetKey(Key.R)) 
        {
            Restart ();
        }
    }

    void drawContactPoints ()
    {
        clearDrawnContactPoints();
        List<Vector2> contactPoints = new List<Vector2>(world.ContactPointList);

        for (int i = 0; i < contactPoints.Count; i++)
        {
            Vector2 contactPoint = contactPoints[i];
            RectangleD rect = new RectangleD(contactPoint,10.0f, 10.0f);
            AddChild(rect);
            drawnContactPoints.Add(rect);
        }
    }

    void clearDrawnContactPoints ()
    {
        if (drawnContactPoints.Count > 0)
        {
            foreach (RectangleD rect in drawnContactPoints)
            {
                if (rect.lifeTime > 0) return;
                rect.LateDestroy();
            }
            drawnContactPoints.Clear();
        }
    }

    void Restart ()
    {
        //foreach (GameObject child in this.GetChildren()) 
        //{
        //    if (child is Ball)
        //    {
        //        foreach (RigidBody rigidBody in child.GetChildren()) 
        //        {
        //            rigidBody.LateDestroy();
        //        }
        //    }

        //    child.LateDestroy();
        //}

        

        //SetUp();
    }
}

