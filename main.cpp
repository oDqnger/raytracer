#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <vector>
#include <random>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "./stb_image_write.h"
 
#define WIDTH 800 
#define HEIGHT 800
#define COMP 3
 
#define TBOUND 10000000

#define MAX_BOUNCE_REFLECTION 30
 
using std::vector;
 
typedef struct {
  float x;
  float y;
  float z;
} Vector;

typedef struct {
  float islight;
  unsigned char color[3];
  float shinniness;
  float reflectiveness;
} Material;

typedef struct {
  Vector center;
  float radius;
  Material material;
} Sphere;
 
enum LightType {
  DIRECTIONAL=0,
  POSITIONAL =1,
};
 
typedef struct {
  float intensity;
  enum LightType type;
  Vector position;
} Light;

typedef struct {
  Vector normal;
  Vector a1;
  Vector a2;
  Vector a3;
  Material material;
} Triangle;

float dot_prod(Vector* a, Vector *b) {
  return a->x * b->x + a->y * b->y + a->z * b->z;
}
 
Vector sub_vec(Vector* a, Vector* b) {
  Vector dvector;
  dvector.x = a->x - b->x;
  dvector.y = a->y - b->y;
  dvector.z = a->z - b->z;
 
  return dvector;
}
 
Vector add_vec(Vector* a, Vector* b) {
  Vector dvector;
  dvector.x = a->x + b->x;
  dvector.y = a->y + b->y;
  dvector.z = a->z + b->z;
 
  return dvector;
}
 
Vector scale_vec(Vector* a, float scale) {
  Vector svec;
  svec.x = a->x * scale;
  svec.y = a->y * scale;
  svec.z = a->z * scale;
  return svec;
}
 
Vector reflect(Vector* normal, Vector* incident) {
  Vector neg_inc = scale_vec(incident, -1);
  Vector first_part = scale_vec(normal, 2 * dot_prod(normal, &neg_inc));
  Vector rvec = sub_vec(&first_part, &neg_inc);
  return rvec;
}
 
inline float magnitude(Vector* vec) {
  return sqrt((vec->x*vec->x + vec->y * vec->y + vec->z * vec->z));
}
 
inline Vector normalize_vec(Vector* vec) {
  Vector nvec;
  float mag = magnitude(vec);
  nvec.x = vec->x / mag;
  nvec.y = vec->y / mag;
  nvec.z = vec->z / mag;
  return nvec;
}

Vector cross_product(Vector* a, Vector* b) {
  Vector cp = {};
  cp.x = a->y*b->z - a->z*b->y;
  cp.y = a->z*b->x - a->x*b->z;
  cp.z = a->x*b->y - a->y*b->x;
  return cp;
}
 
void display_vector(Vector* a) {
  std::cout << "(" << a->x << ", " << a->y << ", " << a->z << ")\n";
}

float calculate_triangle_area(Vector* a1, Vector *a2, Vector *a3) {
  Vector a1a2 = sub_vec(a2, a1);
  Vector a1a3 = sub_vec(a3, a1);
  Vector cp = cross_product(&a1a2, &a1a3);
  float total_area = magnitude(&cp) / 2;

  return total_area;
}

bool point_on_triangle(Vector* a1, Vector* a2, Vector *a3, Vector *p) {
  float total_area = calculate_triangle_area(a1, a2, a3);
  float u = calculate_triangle_area(p, a1, a2);
  float v = calculate_triangle_area(p, a1, a3);
  float w = calculate_triangle_area(p, a2, a3);
  if ((u+v+w == total_area) && (u>=0 && v>=0 && w>=0)) {
    return true;
  }
  return false;
}
 
inline float canvas_to_viewport(int x, int viewport_w, int t) {
  return x * ((float) viewport_w / t) - ((float)viewport_w / 2);
}

float ray_plane_interception(Vector* o, Vector* a, Vector* n, Vector* d, float min_bound) {
  Vector top_part = sub_vec(o, a);

  float t1 = -(dot_prod(&top_part, n))/(dot_prod(d, n));
  if (t1 > min_bound) {
    return t1;
  }
  return 0;
}

float ray_sphere_interception(Vector* d, Vector* o, Vector* center, float radius, float min_bound) {
  Vector co = sub_vec(o, center);
  float a = dot_prod(d, d);
  float b = 2 * (dot_prod(&co, d));
  float c = dot_prod(&co, &co) - (radius * radius);
  float discriminant = b*b - 4*a*c;
  if (discriminant >= 0) {
    float sq = sqrt(discriminant);
    float t1 = (-b + sq) / (2 * a);
    float t2 = (-b - sq) / (2 * a);
    if (t1 > min_bound && t2 > min_bound) {
      if (t1 <= t2) {
        return t1;
      } else {
        return t2;
      }
    } else if (t1 > min_bound) {
      return t1;
    } else if (t2 > min_bound){
      return t2;
    }
  }
  return 0;
}
 
float illumination_equation(vector<Light> light, Vector* normal, Vector* sphere_coords) {
  float total_illum = 0;
  Vector light_dir;
  for (int i = 0; i<light.size(); i++) {
    if (light[i].type == POSITIONAL) {
      light_dir = sub_vec(&(light[i].position), sphere_coords);
    } else {
      light_dir = light[i].position;
    }
    float diffuse_dot = dot_prod(normal, &light_dir);
    if (diffuse_dot > 0) {
      total_illum += (light[i].intensity * ((diffuse_dot) / (magnitude(normal) * magnitude(&light_dir))));
    }
  }
 
  return total_illum;
}
 
Vector get_coords_from_ray(Vector *camera_pos, float time, Vector* direction) {
  Vector first_part = scale_vec(direction, time);
  return add_vec(camera_pos, &first_part);
}
 
typedef struct {
  Vector start_pos;
  Vector direction;
  float intensity;
  float starting_t;
  int bounces;
} Ray;

inline double random_double() {
    return std::rand() / (RAND_MAX + 1.0);
}

inline float random_double(double min, double max) {
    return min + (max-min)*random_double();
}

Vector gen_rand_vec() {
  Vector random_vec = {random_double(-1, 1), random_double(-1, 1), random_double(-1, 1)};
  while (magnitude(&random_vec) > 1) {
    random_vec = {random_double(-1, 1), random_double(-1, 1), random_double(-1, 1)};
  }
  random_vec = normalize_vec(&random_vec);
  return random_vec;
}
 
Vector return_color(vector<Sphere> spheres, vector<Light> lights, Ray ray, vector<Triangle> triangles) {
  Vector color = {135, 206, 250};
  Vector reflective_color = {0,0,0};
  float min_time = INFINITY;
  Sphere closest_sphere;
  for (int i = 0; i<spheres.size(); i++) {
    float t = ray_sphere_interception(&(ray.direction), &(ray.start_pos), &(spheres[i].center), spheres[i].radius, ray.starting_t);
    if (t > 0 && t < min_time) {
      min_time = t;
      closest_sphere = spheres[i];
    }
  }
  
  Triangle closest_triangle;
  for (int i = 0; i<triangles.size(); i++) {
    float t = ray_plane_interception(&(ray.start_pos), &(triangles[i].a1), &(triangles[i].normal), &(ray.direction), ray.starting_t);
    if (t>0 && t < min_time) {
      Vector triang_coords = get_coords_from_ray(&(ray.start_pos), t, &(ray.direction));
      bool istriangle = point_on_triangle(&(triangles[i].a1), &(triangles[i].a2), &(triangles[i].a3), &triang_coords);
      if (istriangle) {
        min_time = t;
        closest_triangle = triangles[i];
        closest_sphere = {{}, -1};
      }
    }
  }
  Material material_object = closest_sphere.radius == -1 ? closest_triangle.material : closest_sphere.material;

  if (min_time < INFINITY) {
    if (material_object.islight > 0) {
      return {
        std::min((float)material_object.color[0] * material_object.islight, 255.0f),
        std::min((float)material_object.color[1] * material_object.islight, 255.0f),
        std::min((float)material_object.color[2] * material_object.islight, 255.0f),
      };
    }
    Vector coords = get_coords_from_ray(&(ray.start_pos), min_time, &(ray.direction));
     
    Vector normal;
    if (closest_sphere.radius != -1) {
      Vector summat = sub_vec(&coords, &(closest_sphere.center));
      normal = normalize_vec(&summat);
    } else {
      normal = closest_triangle.normal;
    }

    bool in_shadow = false;
    for (int l = 0; l<lights.size(); l++) {
      Vector something = sub_vec(&(lights[l].position), &coords);
      for (int k = 0; k<spheres.size(); k++) {
        float time = ray_sphere_interception(&something, &coords, &(spheres[k].center), spheres[k].radius, 0.001);
        if (time != 0) {
          in_shadow = true;
          break;
        }
      }
      if (!in_shadow) {
        for (int k = 0; k<triangles.size(); k++) {
          float time = ray_plane_interception(&coords, &(triangles[k].a1), &(triangles[k].normal), &something, 0.001);
          if (time > 0) {
            Vector triang_coords = get_coords_from_ray(&(coords), time, &something);
            bool istriangle = point_on_triangle(&(triangles[k].a1), &(triangles[k].a2), &(triangles[k].a3), &triang_coords);
            if (istriangle) {
              in_shadow = true;
              break;
            }
          }
        }
      }
    }
    if (!in_shadow) {
      float total_illum = illumination_equation(lights, &normal, &coords);
     
      color.x = material_object.color[0] * total_illum > 255 ? 255 : material_object.color[0] * total_illum;
      color.y = material_object.color[1] * total_illum > 255 ? 255 : material_object.color[1] * total_illum;
      color.z = material_object.color[2] * total_illum > 255 ? 255 : material_object.color[2] * total_illum;
    } else {
      color.x = 0;
      color.y = 0;
      color.z = 0;
    }
    bool did_reflect = false;
    if (closest_sphere.material.reflectiveness > 0) {
      ray.bounces += 1;
      if (ray.bounces <= MAX_BOUNCE_REFLECTION) {
        Vector direction = reflect(&normal, &(ray.direction));
        Ray reflected_ray = {coords, direction, 1, 0.01, ray.bounces};
        Vector shiny_vec = {(float)rand(), (float)rand(), (float)rand()};
        shiny_vec = normalize_vec(&shiny_vec);
        shiny_vec = scale_vec(&shiny_vec, material_object.shinniness);
        reflected_ray.direction = add_vec(&(reflected_ray.direction), &shiny_vec);
        reflected_ray.direction = normalize_vec(&(reflected_ray.direction));
        reflective_color = return_color(spheres, lights, reflected_ray, triangles);
        reflective_color.x = reflective_color.x;
        reflective_color.y = reflective_color.y;
        reflective_color.z = reflective_color.z;
      }
      did_reflect = true;
    } 
    color.x = color.x * (1-material_object.reflectiveness) * ray.intensity + reflective_color.x * material_object.reflectiveness;
    color.y = color.y * (1-material_object.reflectiveness) * ray.intensity + reflective_color.y * material_object.reflectiveness;
    color.z = color.z * (1-material_object.reflectiveness) * ray.intensity + reflective_color.z * material_object.reflectiveness;
    if (ray.bounces <= MAX_BOUNCE_REFLECTION && did_reflect == false) {

      Vector random_dir = gen_rand_vec();
      if (dot_prod(&random_dir, &normal) < 0) {
        random_dir = scale_vec(&random_dir, -1);
      }
      random_dir = add_vec(&normal, &random_dir);
      random_dir = normalize_vec(&random_dir);
      Ray diffuse_ray = {coords, random_dir,ray.intensity*0.5f, 0.001, ray.bounces};
      Vector diffuse_color = return_color(spheres, lights, diffuse_ray, triangles);
      color.x = color.x + diffuse_color.x * material_object.color[0] / 255.0f;
      color.y = color.y + diffuse_color.y * material_object.color[1] / 255.0f;
      color.z = color.z + diffuse_color.z * material_object.color[2] / 255.0f;
      color.x = std::min(color.x, 255.0f);
      color.y = std::min(color.y, 255.0f);
      color.z = std::min(color.z, 255.0f);
    }
  }
  return color;
}
 
int main() {
  int index = 0;
  unsigned char data[WIDTH * HEIGHT * COMP];
 
  const float viewport_w = 1;
  const float viewport_h = 1;
  const int d = 1;
 
  Vector camera_pos = {0.5,0,0};

vector<Triangle> planes = {
  // Front face (z = 2.25)
  {{},{-0.5f,-0.5f,2.25f},{0.5f,-0.5f,2.25f},{0.5f,0.5f,2.25f},{0,{255,0,0},0,0}},
  {{},{-0.5f,-0.5f,2.25f},{0.5f,0.5f,2.25f},{-0.5f,0.5f,2.25f},{0,{255,0,0},0,0}},
  // Back face (z = 1.75)
  {{},{0.5f,-0.5f,1.75f},{-0.5f,-0.5f,1.75f},{-0.5f,0.5f,1.75f},{0,{255,0,0},0,0}},
  {{},{0.5f,-0.5f,1.75f},{-0.5f,0.5f,1.75f},{0.5f,0.5f,1.75f},{0,{255,0,0},0,0}},
  // Left face (x = -0.5)
  {{},{-0.5f,-0.5f,1.75f},{-0.5f,-0.5f,2.25f},{-0.5f,0.5f,2.25f},{0,{255,0,0},0,0}},
  {{},{-0.5f,-0.5f,1.75f},{-0.5f,0.5f,2.25f},{-0.5f,0.5f,1.75f},{0,{255,0,0},0,0}},
  // Right face (x = 0.5)
  {{},{0.5f,-0.5f,2.25f},{0.5f,-0.5f,1.75f},{0.5f,0.5f,1.75f},{0,{255,0,0},0,0}},
  {{},{0.5f,-0.5f,2.25f},{0.5f,0.5f,1.75f},{0.5f,0.5f,2.25f},{0,{255,0,0},0,0}},
  // Top face (y = 0.5)
  {{},{-0.5f,0.5f,2.25f},{0.5f,0.5f,2.25f},{0.5f,0.5f,1.75f},{0,{255,0,0},0,0}},
  {{},{-0.5f,0.5f,2.25f},{0.5f,0.5f,1.75f},{-0.5f,0.5f,1.75f},{0,{255,0,0},0,0}},
  // Bottom face (y = -0.5)
  {{},{-0.5f,-0.5f,1.75f},{0.5f,-0.5f,1.75f},{0.5f,-0.5f,2.25f},{0,{255,0,0},0,0}},
  {{},{-0.5f,-0.5f,1.75f},{0.5f,-0.5f,2.25f},{-0.5f,-0.5f,2.25f},{0,{255,0,0},0,0}},
};

for (auto& tri : planes) {
  Vector d1 = sub_vec(&tri.a3, &tri.a1);
  Vector d2 = sub_vec(&tri.a2, &tri.a1);
  tri.normal = cross_product(&d1, &d2);
  tri.normal = normalize_vec(&tri.normal);
}
 
  vector<Light> lights = {
    // {1.0, DIRECTIONAL, {0,1,0}},
    {1.2, POSITIONAL, {0,1,0.8}},
    // {2, POSITIONAL, {-1,1,3}},
  };

  vector<Sphere> spheres = {
    // {{-1.2,0,4}, 0.3, {0, {255, 0, 0}, 0, 0}},
    // {{-0.6,0,4}, 0.3, {0, {255, 0,0}, 0.5, 0.5}},
    // {{0.0,0,4}, 0.3, {0, {255, 0,0}, 0.0, 1.00}},
    // {{0.6,0,4}, 0.3, {0, {255, 0,0}, 0.5, 0.98}},
    // {{1.2,0,4}, 0.3, {0, {255, 0,0}, 1.0, 0.4}},
    {{0,-5001,0}, 5000, {0, {128, 128, 128}, 0, 0}},
    // {lights[0].position, 0.2, {1, {255, 255,255},0,0}},
  };

  constexpr int ray_samples = 1;
 
  for (int y = 0; y<HEIGHT; y++) {
    std::cout << y << "/" << HEIGHT << std::endl;
    for (int x = 0; x<WIDTH; x++) {
      Vector final_color = {0,0,0};
      Vector viewport_coords = {canvas_to_viewport(x, viewport_w, WIDTH),-canvas_to_viewport(y, viewport_h, HEIGHT),d};
      Vector direction = sub_vec(&viewport_coords, &camera_pos);
      for (int i = 0; i<ray_samples; i++) {
        float angle = 20.0 / 360 * 2*3.14;
        // direction.y = cosf(angle) * direction.y + -sinf(angle) * direction.z + 0;
        // direction.z = sinf(angle) * direction.y + cosf(angle) * direction.z + 0;
        direction.x = cosf(angle) * direction.x + sinf(angle) * direction.z + 0;
        direction.z = -sinf(angle) * direction.x + cosf(angle) * direction.z + 0;
        Ray ray = {camera_pos, direction, 1, 1, 0};
        Vector color = return_color(spheres, lights, ray, planes);
        final_color.x += color.x;
        final_color.y += color.y;
        final_color.z += color.z;
      }
      data[index++] = final_color.x / ray_samples;
      data[index++] = final_color.y / ray_samples;
      data[index++] = final_color.z / ray_samples;
    }
  }
 
  int success = stbi_write_png("image.png", WIDTH, HEIGHT, COMP, data, WIDTH * COMP);
  return 0;
}
