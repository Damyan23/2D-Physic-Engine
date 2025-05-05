using GXPEngine.Core;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GXPEngine.EasyDraw_Shapes
{
    public class RectangleD : EasyDraw
    {
        public float lifeTime = 50;
        public RectangleD (Vector2 position,float width, float height) : base((int)width + 1, (int)height + 1)
        {
            SetOrigin(width / 2, height / 2);
            this.SetXY(position.x, position.y);
            Draw(width, height);
        }

        void Draw(float width, float height)
        {
            Fill(255, 0, 0);
            Stroke(255, 0, 1);
            Rect(width / 2, height / 2, width, height);
        }

        void Update ()
        {
            if (lifeTime > 0)
            lifeTime -= Time.deltaTime;
        }
    }
}
