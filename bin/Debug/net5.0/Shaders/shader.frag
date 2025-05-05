#version 460 core

/*** CONSTANTS ***/
#define EPSILON 0.001
#define BIG 1000000.0
const int DIFFUSE = 1;
const int REFLECTION = 2;
const int REFRACTION = 3;

/*** DATA STRUCTURES ***/
struct SCamera
{
    vec3 Position;
    vec3 View;
    vec3 Up;
    vec3 Side;
    vec2 Scale;
};

struct SRay
{
    vec3 Origin;
    vec3 Direction;
};

struct SSphere
{
    vec3 Center;
    float Radius;
    int MaterialIdx;
};

struct STriangle
{
    vec3 v1;
    vec3 v2;
    vec3 v3;
    int MaterialIdx;
};

struct SIntersection
{
    float Time;
    vec3 Point;
    vec3 Normal;
    vec3 Color;
    vec4 LightCoeffs;
    float ReflectionCoef;
    float RefractionCoef;
    int MaterialType;
};

struct SLight
{
    vec3 Position;
};

struct SMaterial
{
    vec3 Color;
    vec4 LightCoeffs;
    float ReflectionCoef;
    float RefractionCoef;
    int MaterialType;
};

struct STracingRay
{
    SRay ray;
    float contribution;
    int depth;
};

/*** SHADER INPUT/OUTPUT ***/
in vec3 glPosition;
out vec4 FragColor;

/*** SCENE DATA ***/
STriangle triangles[10];
SSphere spheres[2];

SLight uLight;
SMaterial materials[6];
STracingRay rayStack[10];
int stackSize = 0;

/*** FUNCTIONS ***/
SRay GenerateRay(SCamera uCamera)
{
    vec2 coords = glPosition.xy * uCamera.Scale;
    vec3 direction = uCamera.View + uCamera.Side * coords.x + uCamera.Up * coords.y;
    return SRay(uCamera.Position, normalize(direction));
}

SCamera initializeDefaultCamera()
{
    SCamera camera;
    camera.Position = vec3(0.0, 0.0, -8.0);
    camera.View = vec3(0.0, 0.0, 1.0);
    camera.Up = vec3(0.0, 1.0, 0.0);
    camera.Side = vec3(1.0, 0.0, 0.0);
    camera.Scale = vec2(1.0);
    return camera;
}

bool IntersectSphere(SSphere sphere, SRay ray, float start, float final, out float time)
{
    ray.Origin -= sphere.Center;
    float A = dot(ray.Direction, ray.Direction);
    float B = dot(ray.Direction, ray.Origin);
    float C = dot(ray.Origin, ray.Origin) - sphere.Radius * sphere.Radius;
    float D = B * B - A * C;
    
    if (D > 0.0)
    {
        D = sqrt(D);
        float t1 = (-B - D) / A;
        float t2 = (-B + D) / A;
        
        if(t1 < 0 && t2 < 0)
            return false;
            
        if(min(t1, t2) < 0)
        {
            time = max(t1, t2);
            return true;
        }
        
        time = min(t1, t2);
        return true;
    }
    return false;
}

bool IntersectTriangle(SRay ray, vec3 v1, vec3 v2, vec3 v3, out float time)
{
    time = -1;
    vec3 A = v2 - v1;
    vec3 B = v3 - v1;
    vec3 N = cross(A, B);
    
    float NdotRayDirection = dot(N, ray.Direction);
    if (abs(NdotRayDirection) < EPSILON)
        return false;
        
    float d = dot(N, v1);
    float t = -(dot(N, ray.Origin) - d) / NdotRayDirection;
    
    if (t < 0)
        return false;
        
    vec3 P = ray.Origin + t * ray.Direction;
    
    // Inside-out test
    vec3 C;
    vec3 edge1 = v2 - v1;
    vec3 VP1 = P - v1;
    C = cross(edge1, VP1);
    if (dot(N, C) < 0)
        return false;
    
    vec3 edge2 = v3 - v2;
    vec3 VP2 = P - v2;
    C = cross(edge2, VP2);
    if (dot(N, C) < 0)
        return false;
    
    vec3 edge3 = v1 - v3;
    vec3 VP3 = P - v3;
    C = cross(edge3, VP3);
    if (dot(N, C) < 0)
        return false;
    
    time = t;
    return true;
}

bool Raytrace(SRay ray, SSphere spheres[2], STriangle triangles[10], float start, float final, inout SIntersection intersect)
{
    bool result = false;
    float test = start;
    intersect.Time = final;
    
    // Intersect with spheres
    for(int i = 0; i < 2; i++)
    {
        if(IntersectSphere(spheres[i], ray, start, final, test) && test < intersect.Time)
        {
            intersect.Time = test;
            intersect.Point = ray.Origin + ray.Direction * test;
            intersect.Normal = normalize(intersect.Point - spheres[i].Center);
            intersect.Color = vec3(1, 0, 0);
            intersect.LightCoeffs = vec4(0);
            intersect.ReflectionCoef = 0.0;
            intersect.RefractionCoef = 0.0;
            intersect.MaterialType = 0;
            result = true;
        }
    }
    
    // Intersect with triangles
    for(int i = 0; i < 10; i++)
    {
        if(IntersectTriangle(ray, triangles[i].v1, triangles[i].v2, triangles[i].v3, test) && test < intersect.Time)
        {
            intersect.Time = test;
            intersect.Point = ray.Origin + ray.Direction * test;
            intersect.Normal = normalize(cross(triangles[i].v1 - triangles[i].v2, triangles[i].v3 - triangles[i].v2));
            intersect.Color = vec3(0, 1, 0);
            intersect.LightCoeffs = vec4(0);
            intersect.ReflectionCoef = 0.0;
            intersect.RefractionCoef = 0.0;
            intersect.MaterialType = 0;
            result = true;
        }
    }
    
    return result;
}

void pushRay(STracingRay ray)
{
    rayStack[stackSize] = ray;
    stackSize++;
}

STracingRay popRay()
{
    stackSize--;
    return rayStack[stackSize];
}

bool isEmpty()
{
    return stackSize <= 0;
}

void initializeDefaultScene(out STriangle triangles[10], out SSphere spheres[2])
{
    /* TRIANGLES */
    /* left wall */
    triangles[0].v1 = vec3(-5.0,-5.0,-5.0);
    triangles[0].v2 = vec3(-5.0, 5.0, 5.0);
    triangles[0].v3 = vec3(-5.0, 5.0,-5.0);
    triangles[0].MaterialIdx = 0;

    triangles[1].v1 = vec3(-5.0,-5.0,-5.0);
    triangles[1].v2 = vec3(-5.0,-5.0, 5.0);
    triangles[1].v3 = vec3(-5.0, 5.0, 5.0);
    triangles[1].MaterialIdx = 0;

    /* back wall */
    triangles[2].v1 = vec3(-5.0,-5.0, 5.0);
    triangles[2].v2 = vec3(5.0,-5.0, 5.0);
    triangles[2].v3 = vec3(-5.0, 5.0, 5.0);
    triangles[2].MaterialIdx = 0;
    
    triangles[3].v1 = vec3(5.0, 5.0, 5.0);
    triangles[3].v2 = vec3(-5.0, 5.0, 5.0);
    triangles[3].v3 = vec3(5.0,-5.0, 5.0);
    triangles[3].MaterialIdx = 0;

    /* SPHERES */
    spheres[0].Center = vec3(-1.0,-1.0,-2.0);
    spheres[0].Radius = 2.0;
    spheres[0].MaterialIdx = 0;
    
    spheres[1].Center = vec3(2.0,1.0,2.0);
    spheres[1].Radius = 1.0;
    spheres[1].MaterialIdx = 0;
}

void initializeDefaultLightMaterials(out SLight light, out SMaterial materials[6])
{
    light.Position = vec3(0.0, 2.0, -4.0f);
    
    vec4 lightCoefs = vec4(0.4, 0.9, 0.0, 512.0);
    materials[0].Color = vec3(0.0, 1.0, 0.0);
    materials[0].LightCoeffs = lightCoefs;
    materials[0].ReflectionCoef = 0.5;
    materials[0].RefractionCoef = 1.0;
    materials[0].MaterialType = DIFFUSE;
    
    materials[1].Color = vec3(0.0, 0.0, 1.0);
    materials[1].LightCoeffs = lightCoefs;
    materials[1].ReflectionCoef = 0.5;
    materials[1].RefractionCoef = 1.0;
    materials[1].MaterialType = DIFFUSE;
}


void main()
{
    float start = 0;
    float final = BIG;
    
    SCamera uCamera = initializeDefaultCamera();
    SRay ray = GenerateRay(uCamera);
    
    SIntersection intersect;
    intersect.Time = BIG;
    vec3 resultColor = vec3(0, 0, 0);
    
    initializeDefaultScene(triangles, spheres);
    
    if(Raytrace(ray, spheres, triangles, start, final, intersect))
    {
        resultColor = intersect.Color;
    }
    
    FragColor = vec4(resultColor, 1.0);
}