using GXPEngine;
using GXPEngine.Core;

public readonly struct FlatTransform
{
    public readonly float PositionX;
    public readonly float PositionY;
    public readonly float Sin;
    public readonly float Cos;

    public readonly static FlatTransform Zero = new FlatTransform(0f, 0f, 0f);

    public FlatTransform (Vector2 position, float angle)
    {
        this.PositionX = position.x; 
        this.PositionY = position.y;
        this.Sin = Mathf.Sin(angle);
        this.Cos = Mathf.Cos(angle);
    }

    public FlatTransform(float x, float y, float angle)
    {
        this.PositionX = x;
        this.PositionY = y;
        this.Sin = Mathf.Sin(angle);
        this.Cos = Mathf.Cos(angle);
    }
}