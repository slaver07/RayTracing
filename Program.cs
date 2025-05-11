using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RayTracing
{
    class Program
    {
        static void Main(string[] args)
        {
            using (View view = new View(1200, 1200))
            {
                view.Run();
            }
        }
    }
}
