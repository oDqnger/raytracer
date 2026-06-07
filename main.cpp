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
  Vector fixed_point;
} Plane;

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
 
void display_vector(Vector* a) {
  std::cout << "(" << a->x << ", " << a->y << ", " << a->z << ")\n";
}
 
inline float canvas_to_viewport(int x, int viewport_w, int t) {
  return x * ((float) viewport_w / t) - ((float)viewport_w / 2);
}

float ray_plane_interception(Vector* o, Vector* a, Vector* n, Vector* d) {
  Vector top_part = sub_vec(o, a);
  return -(dot_prod(&top_part, n))/(dot_prod(d, n));
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
 
Vector return_color(vector<Sphere> spheres, vector<Light> lights, Ray ray, vector<Plane> planes) {
  Vector color = {135, 206, 235};
  Vector reflective_color = {135,206,235};
  float min_time = INFINITY;
  Sphere closest_sphere;
  for (int i = 0; i<spheres.size(); i++) {
    float t = ray_sphere_interception(&(ray.direction), &(ray.start_pos), &(spheres[i].center), spheres[i].radius, ray.starting_t);
    if (t > 0 && t < min_time) {
      min_time = t;
      closest_sphere = spheres[i];
    }
  }
  
  if (min_time < INFINITY) {
    if (closest_sphere.material.islight > 0) {
      return {(float)closest_sphere.material.color[0] * closest_sphere.material.islight, (float)closest_sphere.material.color[1] * closest_sphere.material.islight, (float)closest_sphere.material.color[2] * closest_sphere.material.islight};
    }
    Vector sphere_coords = get_coords_from_ray(&(ray.start_pos), min_time, &(ray.direction));

    Vector summat = sub_vec(&sphere_coords, &(closest_sphere.center));
    Vector normal = normalize_vec(&summat);

    bool in_shadow = false;
    for (int l = 0; l<lights.size(); l++) {
      for (int k = 0; k<spheres.size(); k++) {
        Vector something = sub_vec(&(lights[l].position), &sphere_coords);
        float time = ray_sphere_interception(&something, &sphere_coords, &(spheres[k].center), spheres[k].radius, 0.001);
        if (time != 0) {
          in_shadow = true;
        }
      }
    }
    if (!in_shadow) {
      float total_illum = illumination_equation(lights, &normal, &sphere_coords);
     
      color.x = closest_sphere.material.color[0] * total_illum > 255 ? 255 : closest_sphere.material.color[0] * total_illum;
      color.y = closest_sphere.material.color[1] * total_illum > 255 ? 255 : closest_sphere.material.color[1] * total_illum;
      color.z = closest_sphere.material.color[2] * total_illum > 255 ? 255 : closest_sphere.material.color[2] * total_illum;
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
        Ray reflected_ray = {sphere_coords, direction, 1, 0.01, ray.bounces};
        Vector shiny_vec = {(float)rand(), (float)rand(), (float)rand()};
        shiny_vec = normalize_vec(&shiny_vec);
        shiny_vec = scale_vec(&shiny_vec, closest_sphere.material.shinniness);
        reflected_ray.direction = add_vec(&(reflected_ray.direction), &shiny_vec);
        reflected_ray.direction = normalize_vec(&(reflected_ray.direction));
        reflective_color = return_color(spheres, lights, reflected_ray, planes);
        reflective_color.x = reflective_color.x;
        reflective_color.y = reflective_color.y;
        reflective_color.z = reflective_color.z;
      }
      did_reflect = true;
    } 
    color.x = color.x * (1-closest_sphere.material.reflectiveness) * ray.intensity + reflective_color.x * closest_sphere.material.reflectiveness;
    color.y = color.y * (1-closest_sphere.material.reflectiveness) * ray.intensity + reflective_color.y * closest_sphere.material.reflectiveness;
    color.z = color.z * (1-closest_sphere.material.reflectiveness) * ray.intensity + reflective_color.z * closest_sphere.material.reflectiveness;
    if (ray.bounces <= MAX_BOUNCE_REFLECTION && did_reflect == false) {

      Vector random_dir = gen_rand_vec();
      if (dot_prod(&random_dir, &normal) < 0) {
        random_dir = scale_vec(&random_dir, -1);
      }
      random_dir = add_vec(&normal, &random_dir);
      random_dir = normalize_vec(&random_dir);
      Ray diffuse_ray = {sphere_coords, random_dir,ray.intensity*0.5f, 0.001, ray.bounces};
      Vector diffuse_color = return_color(spheres, lights, diffuse_ray, planes);
      color.x = color.x + diffuse_color.x * closest_sphere.material.color[0] / 255.0f;
      color.y = color.y + diffuse_color.y * closest_sphere.material.color[1] / 255.0f;
      color.z = color.z + diffuse_color.z * closest_sphere.material.color[2] / 255.0f;
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
 
  Vector camera_pos = {0,0,0};
  vector<Sphere> spheres = {
    {{-1.2,0,4}, 0.3, {0, {255, 0, 0}, 0, 0}},
    {{-0.6,0,4}, 0.3, {0, {255, 0,0}, 0.5, 0.5}},
    {{0.0,0,4}, 0.3, {0, {255, 0,0}, 0.0, 1.00}},
    {{0.6,0,4}, 0.3, {0, {255, 0,0}, 0.5, 0.98}},
    {{1.2,0,4}, 0.3, {0, {255, 0,0}, 0.95, 0.5}},
    {{0,-5001,0}, 5000, {0, {128, 128, 128}, 0, 0}},
    {{0,1,3}, 0.1, {1, {255, 255,255},0,0}},
  };

  vector<Plane> planes = {
    // {{0,1,0}, {0,-5,0}},
  };
 
  vector<Light> lights = {
    // {1.0, DIRECTIONAL, {0,1,0}},
    {1, POSITIONAL, {0,1,3}},
    // {2, POSITIONAL, {-1,1,3}},
  };

  constexpr int ray_samples = 50;
 
  for (int y = 0; y<HEIGHT; y++) {
    // std::cout << y << "/" << HEIGHT << std::endl;
    for (int x = 0; x<WIDTH; x++) {
      Vector final_color = {0,0,0};
      Vector viewport_coords = {canvas_to_viewport(x, viewport_w, WIDTH),-canvas_to_viewport(y, viewport_h, HEIGHT),d};
      Vector direction = sub_vec(&viewport_coords, &camera_pos);
      for (int i = 0; i<ray_samples; i++) {
        // float angle = 20.0 / 360 * 2*3.14;
        // direction.y = cosf(angle) * direction.y + -sinf(angle) * direction.z + 0;
        // direction.z = sinf(angle) * direction.y + cosf(angle) * direction.z + 0;
        // direction.x = cosf(angle) * direction.x + sinf(angle) * direction.z + 0;
        // direction.z = -sinf(angle) * direction.x + cosf(angle) * direction.z + 0;
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
 
  int success = stbi_write_png("test.png", WIDTH, HEIGHT, COMP, data, WIDTH * COMP);
  return 0;
}
