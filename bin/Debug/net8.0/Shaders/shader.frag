#version 460 

#define EPSILON 0.001
#define BIG 1000000.0

#define STACK_SIZE 200
#define DEPTH 10

#define LIGHTS_COUNT 10

const int DIFFUSE = 1;
const int REFLECTION = 2;
const int REFRACTION = 3;
const int DIFFUSE_REFLECTION = 1;
const int MIRROR_REFLECTION = 2;

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

struct STriangle
{
    vec3 v1;
    vec3 v2;
    vec3 v3;
    int MaterialIdx;
};

struct SSphere
{
    vec3 Center;
    float Radius;
    int MaterialIdx;
};

struct SCube {
    vec3 Center;
    float Size;
    int MaterialIdx;
};

struct SLight
{
    vec3 Position[LIGHTS_COUNT];
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

struct SStack 
{
    int top;
    STracingRay container[STACK_SIZE];
};

STriangle triangles[12];
SSphere spheres[2];
SCube cubes[1];
SCamera uCamera;
SStack stack;

SLight uLight;
SMaterial materials[6];
float Unit = 1;

out vec4 FragColor; 
in vec3 glPosition; 

void initializeDefaultScene()
{
    triangles[0].v1 = vec3(-5.0,-5.0,-5.0);
    triangles[0].v2 = vec3(-5.0, 5.0, 5.0);
    triangles[0].v3 = vec3(-5.0, 5.0,-5.0);
    triangles[0].MaterialIdx = 2;

    triangles[1].v1 = vec3(-5.0,-5.0,-5.0);
    triangles[1].v2 = vec3(-5.0,-5.0, 5.0);
    triangles[1].v3 = vec3(-5.0, 5.0, 5.0);
    triangles[1].MaterialIdx = 2;

    triangles[2].v1 = vec3(-5.0,-5.0, 5.0);
    triangles[2].v2 = vec3( 5.0,-5.0, 5.0);
    triangles[2].v3 = vec3(-5.0, 5.0, 5.0);
    triangles[2].MaterialIdx = 5;

    triangles[3].v1 = vec3( 5.0, 5.0, 5.0);
    triangles[3].v2 = vec3(-5.0, 5.0, 5.0);
    triangles[3].v3 = vec3( 5.0,-5.0, 5.0);
    triangles[3].MaterialIdx = 5;

    triangles[4].v1 = vec3(5.0,-5.0,-5.0);
    triangles[4].v2 = vec3(5.0, 5.0,-5.0);
    triangles[4].v3 = vec3(5.0, 5.0, 5.0);
    triangles[4].MaterialIdx = 3;

    triangles[5].v1 = vec3(5.0,-5.0,-5.0);
    triangles[5].v2 = vec3(5.0, 5.0, 5.0);
    triangles[5].v3 = vec3(5.0,-5.0, 5.0);
    triangles[5].MaterialIdx = 3;

    triangles[6].v1 = vec3(-5.0,-5.0,-5.0);
    triangles[6].v2 = vec3( 5.0,-5.0,-5.0);
    triangles[6].v3 = vec3(-5.0,-5.0, 5.0);
    triangles[6].MaterialIdx = 4;

    triangles[7].v1 = vec3( 5.0,-5.0,-5.0);
    triangles[7].v2 = vec3( 5.0,-5.0, 5.0);
    triangles[7].v3 = vec3(-5.0,-5.0, 5.0);
    triangles[7].MaterialIdx = 4;

    triangles[8].v1 = vec3(-5.0,5.0,-5.0);
    triangles[8].v2 = vec3(-5.0,5.0, 5.0);
    triangles[8].v3 = vec3( 5.0,5.0,-5.0);
    triangles[8].MaterialIdx = 0;

    triangles[9].v1 = vec3( 5.0,5.0, 5.0);
    triangles[9].v2 = vec3( 5.0,5.0,-5.0);
    triangles[9].v3 = vec3(-5.0,5.0, 5.0);
    triangles[9].MaterialIdx = 0;

    triangles[10].v1 = vec3(-5.0,-5.0,-5.0);
    triangles[10].v2 = vec3(-5.0, 5.0,-5.0);
    triangles[10].v3 = vec3( 5.0, 5.0,-5.0);
    triangles[10].MaterialIdx = 5;

    triangles[11].v1 = vec3(-5.0,-5.0,-5.0);
    triangles[11].v2 = vec3( 5.0, 5.0,-5.0);
    triangles[11].v3 = vec3( 5.0,-5.0,-5.0);
    triangles[11].MaterialIdx = 5;


    spheres[0].Center = vec3(-1.5, -1.0, -1.0);
    spheres[0].Radius = 1.1;
    spheres[0].MaterialIdx = 1;

    spheres[1].Center = vec3(2.0, -1.0, -1.3);
    spheres[1].Radius = 1.5;
    spheres[1].MaterialIdx = 4;

    cubes[0].Center = vec3(-1.1, 1.2, 0.5);
    cubes[0].Size = 1.8;
    cubes[0].MaterialIdx = 3;
}

void initializeDefaultLightMaterials()  
{
    uLight.Position[0] = vec3(0.3, 4.1, -4.2);
    uLight.Position[1] = vec3(0.5, 4.0, -4.0);
    uLight.Position[2] = vec3(0.1, 4.2, -4.1);
    uLight.Position[3] = vec3(0.4, 4.0, -4.3);
    uLight.Position[4] = vec3(0.2, 4.1, -4.0);
    uLight.Position[5] = vec3(0.35, 4.05, -4.15);
    uLight.Position[6] = vec3(0.25, 4.15, -4.25);
    uLight.Position[7] = vec3(0.15, 4.05, -4.05);
    uLight.Position[8] = vec3(0.45, 4.15, -4.35);
    uLight.Position[9] = vec3(0.05, 4.25, -4.15);

    vec4 lightCoefs = vec4(0.4, 0.9, 0.0, 512.0);
    
    materials[0].Color = vec3(0.9, 0.3, 0.2);
    materials[0].LightCoeffs = vec4(lightCoefs);
    materials[0].ReflectionCoef = 0.5;
    materials[0].RefractionCoef = 1.0;
    materials[0].MaterialType = DIFFUSE_REFLECTION;

    materials[1].Color = vec3(0.2, 0.8, 0.3);
    materials[1].LightCoeffs = vec4(lightCoefs);
    materials[1].ReflectionCoef = 0.7;
    materials[1].RefractionCoef = 1.5;
    materials[1].MaterialType = REFRACTION;
    
    materials[2].Color = vec3(0.4, 0.4, 0.9);
    materials[2].LightCoeffs = vec4(lightCoefs);
    materials[2].ReflectionCoef = 0.5;
    materials[2].RefractionCoef = 1.0;
    materials[2].MaterialType = DIFFUSE_REFLECTION;

    materials[3].Color = vec3(0.9, 0.8, 0.1);
    materials[3].LightCoeffs = vec4(lightCoefs);
    materials[3].ReflectionCoef = 0.5;
    materials[3].RefractionCoef = 1.0;
    materials[3].MaterialType = DIFFUSE_REFLECTION;

    materials[4].Color = vec3(0.8, 0.1, 0.8);
    materials[4].LightCoeffs = vec4(lightCoefs);
    materials[4].ReflectionCoef = 0.5;
    materials[4].RefractionCoef = 1.0;
    materials[4].MaterialType = MIRROR_REFLECTION;

    materials[5].Color = vec3(0.1, 0.5, 0.8);
    materials[5].LightCoeffs = vec4(lightCoefs);
    materials[5].ReflectionCoef = 0.3;
    materials[5].RefractionCoef = 1.0;
    materials[5].MaterialType = DIFFUSE_REFLECTION;
}

void initializeDefaultCamera()
{
    uCamera.Position = vec3(0.3, -1.1, -4.8);
    uCamera.View = vec3(0.0, 0.0, 1.0);
    uCamera.Up = vec3(0.0, 1.0, 0.0);
    uCamera.Side = vec3(1.0, 0.0, 0.0);
    uCamera.Scale = vec2(1.5);
}

SRay GenerateRay(SCamera uCamera)
{
    vec2 coords = glPosition.xy * uCamera.Scale;
    vec3 direction = uCamera.View + uCamera.Side * coords.x + uCamera.Up * coords.y;
    return SRay(uCamera.Position, normalize(direction));
}

bool IntersectTriangle(SRay ray, vec3 v1, vec3 v2, vec3 v3, out float time)
{
    time = -1;
    vec3 A = v2 - v1;
    vec3 B = v3 - v1;
    vec3 N = cross(A, B);
    float NdotRayDirection = dot(N, ray.Direction);

    if(abs(NdotRayDirection) < 0.001)
        return false;

    float d = dot(N, v1);
    float t = -(dot(N, ray.Origin) - d) / NdotRayDirection;

    if(t < 0)
        return false;
    
    vec3 P = ray.Origin + t * ray.Direction;
    vec3 C;
    vec3 edge1 = v2 - v1;
    vec3 VP1 = P - v1;
    C = cross(edge1, VP1);
    if(dot(N, C) < 0)
        return false;
    
    vec3 edge2 = v3 - v2;
    vec3 VP2 = P - v2;
    C = cross(edge2, VP2);
    if(dot(N, C) < 0)
        return false;
    
    vec3 edge3 = v1 - v3;
    vec3 VP3 = P - v3;
    C = cross(edge3, VP3);
    if(dot(N, C) < 0)
        return false;
    
    time = t;
    return true;
}

bool IntersectSphere(SSphere sphere, SRay ray, float start, float final, out float time)
{
    ray.Origin -= sphere.Center;
    float A = dot(ray.Direction, ray.Direction);
    float B = dot(ray.Direction, ray.Origin);
    float C = dot(ray.Origin, ray.Origin) - sphere.Radius * sphere.Radius;
    float D = B * B - A * C;
    if(D > 0.0)
    {
        D = sqrt(D);
        float t1 = (-B - D) / A;
        float t2 = (-B + D) / A;
        
        if(t1 < 0 && t2 < 0)
            return false;

        if(min(t1, t2) < 0)
        {
            time = max(t1,t2);
            return true;
        }
        time = min(t1, t2);
        return true;
    }
    return false;
}

bool IntersectCube(SCube cube, SRay ray, float start, float final, out float time) {
    vec3 minBounds = cube.Center - vec3(cube.Size/2.0);
    vec3 maxBounds = cube.Center + vec3(cube.Size/2.0);
    
    vec3 invDir = 1.0 / ray.Direction;
    vec3 t0 = (minBounds - ray.Origin) * invDir;
    vec3 t1 = (maxBounds - ray.Origin) * invDir;
    
    vec3 tmin = min(t0, t1);
    vec3 tmax = max(t0, t1);
    
    float tNear = max(max(tmin.x, tmin.y), tmin.z);
    float tFar = min(min(tmax.x, tmax.y), tmax.z);
    
    if (tNear > tFar || tFar < 0.0) return false;
    
    time = tNear > 0.0 ? tNear : tFar;
    return time >= start && time <= final;
}

bool Raytrace(SRay ray, float start, float final, inout SIntersection intersect)
{
    bool result = false;
    float test = start;
    intersect.Time = final;

    for(int i = 0; i < 12; i++)
    {
        STriangle triangle = triangles[i];
        if(IntersectTriangle(ray, triangle.v1, triangle.v2, triangle.v3, test) && test < intersect.Time)
        {
            intersect.Time = test;
            intersect.Point = ray.Origin + ray.Direction * test;
            intersect.Normal = normalize(cross(triangles[i].v1 - triangles[i].v2, triangles[i].v3 - triangles[i].v2));
            intersect.Color = materials[triangles[i].MaterialIdx].Color;
            intersect.LightCoeffs = materials[triangles[i].MaterialIdx].LightCoeffs;
            intersect.ReflectionCoef = materials[triangles[i].MaterialIdx].ReflectionCoef;
            intersect.RefractionCoef = materials[triangles[i].MaterialIdx].RefractionCoef;
            intersect.MaterialType = materials[triangles[i].MaterialIdx].MaterialType;
            result = true;
        }
    }

    for(int i = 0; i < 2; i++)
    {
        SSphere sphere = spheres[i];
        if(IntersectSphere(sphere, ray, start, final, test) && test < intersect.Time)
        {
            intersect.Time = test;
            intersect.Point = ray.Origin + ray.Direction * test;
            intersect.Normal = normalize(intersect.Point - spheres[i].Center);
            intersect.Color = materials[spheres[i].MaterialIdx].Color;
            intersect.LightCoeffs = materials[spheres[i].MaterialIdx].LightCoeffs;
            intersect.ReflectionCoef = materials[spheres[i].MaterialIdx].ReflectionCoef;
            intersect.RefractionCoef = materials[spheres[i].MaterialIdx].RefractionCoef;
            intersect.MaterialType = materials[spheres[i].MaterialIdx].MaterialType;
            result = true;
        }
    }

    for(int i = 0; i < 1; i++) {
        if(IntersectCube(cubes[i], ray, start, final, test) && test < intersect.Time) {
            intersect.Time = test;
            intersect.Point = ray.Origin + ray.Direction * test;
            vec3 localPoint = intersect.Point - cubes[i].Center;
            vec3 absPoint = abs(localPoint);
            if(absPoint.x > absPoint.y && absPoint.x > absPoint.z) {
                intersect.Normal = vec3(sign(localPoint.x), 0.0, 0.0);
            } else if(absPoint.y > absPoint.z) {
                intersect.Normal = vec3(0.0, sign(localPoint.y), 0.0);
            } else {
                intersect.Normal = vec3(0.0, 0.0, sign(localPoint.z));
            }
            intersect.Color = materials[cubes[i].MaterialIdx].Color;
            intersect.LightCoeffs = materials[cubes[i].MaterialIdx].LightCoeffs;
            intersect.ReflectionCoef = materials[cubes[i].MaterialIdx].ReflectionCoef;
            intersect.RefractionCoef = materials[cubes[i].MaterialIdx].RefractionCoef;
            intersect.MaterialType = materials[cubes[i].MaterialIdx].MaterialType;
            result = true;
        }
    }

    return result;
}

vec3 Phong(SIntersection intersect, SLight lights, float shadows[LIGHTS_COUNT])
{
    vec3 result = vec3(0.0);
    float lightIntensity = 0.2;
    for (int i = 0; i < LIGHTS_COUNT; i++)
    {
        vec3 light = normalize(lights.Position[i] - intersect.Point);
        float diffuse = max(dot(light, intersect.Normal), 0.0);
        vec3 view = normalize(uCamera.Position - intersect.Point);
        vec3 reflected = reflect(-view, intersect.Normal);
        float specular = pow(max(dot(reflected, light), 0.0), intersect.LightCoeffs.w);

        result += lightIntensity * (intersect.LightCoeffs.x * intersect.Color * (1.0/float(LIGHTS_COUNT)) + 
                 intersect.LightCoeffs.y * diffuse * intersect.Color * shadows[i] + 
                 intersect.LightCoeffs.z * specular * Unit * (1.0/float(LIGHTS_COUNT)));
    }
    return result;
}

void calculateShadows(SLight lights, SIntersection intersect, out float shadows[LIGHTS_COUNT])
{
    for (int i = 0; i < LIGHTS_COUNT; i++)
    {
        shadows[i] = 1.0;
        vec3 direction = normalize(lights.Position[i] - intersect.Point);
        float distanceLight = distance(lights.Position[i], intersect.Point);
        SRay shadowRay = SRay(intersect.Point + direction * EPSILON, direction);
        SIntersection shadowIntersect;
        shadowIntersect.Time = BIG;
        if(Raytrace(shadowRay, 0, distanceLight, shadowIntersect))
        {
            shadows[i] = 0.0;
        }
    }
}

bool isEmpty()
{
    return stack.top < 0;
}

STracingRay popRay()
{
    return stack.container[stack.top--];
}

void pushRay(STracingRay t_ray)
{
    if(stack.top == STACK_SIZE - 1) return;
    stack.container[stack.top + 1] = t_ray;
    stack.top++;
}

void main(void)
{
    stack.top = -1;
    float start = 0;
    float final = BIG;

    initializeDefaultCamera();
    initializeDefaultLightMaterials();
    initializeDefaultScene();

    SRay ray = GenerateRay(uCamera);
    SIntersection intersect;
    intersect.Time = BIG;
    vec3 resultColor = vec3(0,0,0);

    STracingRay trRay = STracingRay(ray, 1, 0);
    pushRay(trRay);
    while(!isEmpty())
    {
        STracingRay trRay = popRay();
        if(trRay.depth >= DEPTH) { continue; }
        
        ray = trRay.ray;
        intersect.Time = BIG;
        start = 0;
        final = BIG;
        if(Raytrace(ray, start, final, intersect))
        {
            float attenuation = 0.7;
            float currentContribution = trRay.contribution * pow(attenuation, float(trRay.depth));
            switch(intersect.MaterialType)
            {
            case DIFFUSE_REFLECTION:
            {
                float shadows[LIGHTS_COUNT];
                calculateShadows(uLight, intersect, shadows);
                resultColor += currentContribution * Phong(intersect, uLight, shadows);
                break;
            }
            case MIRROR_REFLECTION:
            {
                if(intersect.ReflectionCoef < 1.0)
                {
                    float diffuseContrib = currentContribution * (1.0 - intersect.ReflectionCoef);
                    float tempShadows[LIGHTS_COUNT];
                    calculateShadows(uLight, intersect, tempShadows);
                    resultColor += diffuseContrib * Phong(intersect, uLight, tempShadows);
                }
                vec3 reflectDirection = reflect(ray.Direction, intersect.Normal);
                float reflectContrib = currentContribution * intersect.ReflectionCoef;
                STracingRay reflectRay = STracingRay(
                    SRay(intersect.Point + reflectDirection * EPSILON,
                    reflectDirection),
                    reflectContrib, trRay.depth + 1);
                pushRay(reflectRay);
                break;
            }
            case REFRACTION:
            {
                float eta = 1.0 / intersect.RefractionCoef;
                vec3 refractDir = refract(normalize(ray.Direction), intersect.Normal, eta);
                if(length(refractDir) > 0.0) {
                    STracingRay refractRay = STracingRay(
                        SRay(intersect.Point + refractDir * EPSILON, refractDir),
                        currentContribution,
                        trRay.depth + 1);
                    pushRay(refractRay);
                }
                if(intersect.ReflectionCoef > 0.0)
                {
                    vec3 reflectDir = reflect(normalize(ray.Direction), intersect.Normal);
                    STracingRay reflectRay = STracingRay(
                        SRay(intersect.Point + reflectDir * EPSILON, reflectDir),
                        currentContribution * intersect.ReflectionCoef, trRay.depth + 1);
                    pushRay(reflectRay);
                }
                resultColor += currentContribution * intersect.Color * 0.2;
                break;
            }
            }
        }
    }

    FragColor = vec4(resultColor, 1.0);
}