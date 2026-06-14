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
 
#define MAX_BOUNCE_REFLECTION 10
 
using std::vector;

enum LightType {
  DIRECTIONAL=0,
  POSITIONAL =1,
};

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

typedef struct {
  Vector start_pos;
  Vector direction;
  float intensity;
  float starting_t;
  int bounces;
} Ray;

typedef struct {
  float intensity;
  enum LightType type;
  Vector position;
} Light;
 
inline float dot_prod(Vector a, Vector b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
 
inline Vector sub_vec(Vector a, Vector b) {
  return {a.x - b.x, a.y-b.y, a.z - b.z};
}
 
inline Vector add_vec(Vector a, Vector b) {
  return {a.x + b.x, a.y+b.y, a.z+b.z};
}
 
inline Vector scale_vec(Vector a, float scale) {
  return {a.x * scale, a.y*scale, a.z*scale};
}
 
inline Vector reflect(Vector normal, Vector incident) {
  Vector neg_inc = scale_vec(incident, -1);
  return sub_vec(scale_vec(normal, 2 * dot_prod(normal, neg_inc)), neg_inc);
}
 
inline float magnitude(Vector vec) {
  return sqrt((vec.x*vec.x + vec.y * vec.y + vec.z * vec.z));
}
 
inline Vector normalize_vec(Vector vec) {
  float mag = magnitude(vec);
  return {vec.x / mag, vec.y / mag, vec.z / mag};
}
 
inline Vector cross_product(Vector a, Vector b) {
  return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
 
Vector rotate_around_x(Vector a, float angle, Vector pivot) {
  a = sub_vec(a, pivot);
  Vector b = a;
  float radians = angle * 3.141592 / 180;
  b.x = a.x * 1 + a.y * 0 + a.z * 0;
  b.y = a.x * 0 + cosf(radians) * a.y - sinf(radians) * a.z;
  b.z = a.x * 0 + sinf(radians) * a.y + cosf(radians) * a.z;
  return add_vec(b, pivot);
}

Vector rotate_around_y(Vector a, float angle, Vector pivot) {
  a = sub_vec(a, pivot);
  Vector b = a;
  float radians = angle * 3.141592 / 180;
  b.x = a.x * cosf(radians) + a.y * 0 + a.z * sinf(radians);
  b.y = a.x * 0 + 1 * a.y + 0 * a.z;
  b.z = a.x * -sinf(radians) + 0 * a.y + cosf(radians) * a.z;
  return add_vec(b, pivot);
}

Vector rotate_around_z(Vector a, float angle, Vector pivot) {
  a = sub_vec(a, pivot);
  Vector b = a;
  float radians = angle * 3.141592 / 180;
  b.x = a.x * cosf(radians) - a.y * sinf(radians) + a.z * 0;
  b.y = a.x * sinf(radians) + cosf(radians) * a.y + 0 * a.z;
  b.z = a.x * 0 + 0 * a.y + 1 * a.z;
  return add_vec(b, pivot);
}
 
void display_vector(Vector a) {
  std::cout << "(" << a.x << ", " << a.y << ", " << a.z << ")\n";
}
 
inline float calculate_triangle_area(Vector a1, Vector a2, Vector a3) {
  return magnitude(cross_product(sub_vec(a2, a1), sub_vec(a3, a1))) / 2;
}

inline bool check_within_range(float value, float upper, float lower, float comparision) {
  if ((value >= comparision - lower) && (value <= comparision + upper)) {
    return true;
  }
  return false;
}

bool point_on_triangle(Vector a1, Vector a2, Vector a3, Vector p) {
  float total_area = calculate_triangle_area(a1, a2, a3);
  float u = calculate_triangle_area(p, a1, a2);
  float v = calculate_triangle_area(p, a1, a3);
  float w = calculate_triangle_area(p, a2, a3);
  if (check_within_range(u+w+v, 0.001, 0.001, total_area) && (u>=0 && v>=0 && w>=0)) {
    return true;
  }
  return false;
}
 
inline float canvas_to_viewport(int x, int viewport_w, int t) {
  return x * ((float) viewport_w / t) - ((float)viewport_w / 2);
}
 
float ray_plane_interception(Vector o, Vector a, Vector n, Vector d, float min_bound) {
  float t1 = -(dot_prod(sub_vec(o, a), n))/(dot_prod(d, n));
  if (t1 > min_bound) {
    return t1;
  }
  return 0;
}
 
float ray_sphere_interception(Vector d, Vector o, Vector center, float radius, float min_bound) {
  Vector co = sub_vec(o, center);
  float a = dot_prod(d, d);
  float b = 2 * (dot_prod(co, d));
  float c = dot_prod(co, co) - (radius * radius);
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
 
float illumination_equation(vector<Light> light, Vector normal, Vector sphere_coords) {
  float total_illum = 0;
  Vector light_dir;
  for (int i = 0; i<light.size(); i++) {
    if (light[i].type == POSITIONAL) {
      light_dir = sub_vec((light[i].position), sphere_coords);
    } else {
      light_dir = light[i].position;
    }
    float diffuse_dot = dot_prod(normal, light_dir);
    if (diffuse_dot > 0) {
      total_illum += (light[i].intensity * ((diffuse_dot) / (magnitude(normal) * magnitude(light_dir))));
    }
  }
 
  return total_illum;
}
 
inline Vector get_coords_from_ray(Vector camera_pos, float time, Vector direction) {
  return add_vec(camera_pos, scale_vec(direction, time));
}
 
inline double random_double() {
    return std::rand() / (RAND_MAX + 1.0);
}
 
inline float random_double(double min, double max) {
    return min + (max-min)*random_double();
}

float min(vector<float> values) {
  float min_val = INFINITY;
  for (int i = 0; i<values.size(); i++) {
    if (values[i] < min_val) {
      min_val = values[i];
    }
  }

  return min_val;
}

float max(vector<float> values) {
  float max_val = -INFINITY;
  for (int i = 0; i<values.size(); i++) {
    if (values[i] > max_val) {
      max_val = values[i];
    }
  }

  return max_val;
}

class Triangle {
  public:
    Vector a1;
    Vector a2;
    Vector a3;
    Vector normal;
    Material material;
    Triangle(Vector ua1, Vector ua2, Vector ua3, Material m, Vector n) {
      a1 = ua1;
      a2 = ua2;
      a3 = ua3;
      material = m;
      normal = n;
    }
    Triangle() {}
    void translate(Vector translation_vector) {
      a1 = add_vec(a1, translation_vector);
      a2 = add_vec(a2, translation_vector);
      a3 = add_vec(a3, translation_vector);
    }

    void scale(Vector scale, Vector pivot) {
      a1 = sub_vec(a1, pivot);
      a2 = sub_vec(a2, pivot);
      a3 = sub_vec(a3, pivot);
      a1.x = a1.x * scale.x;
      a1.y = a1.y * scale.y;
      a1.z = a1.z * scale.z;

      a2.x = a2.x * scale.x;
      a2.y = a2.y * scale.y;
      a2.z = a2.z * scale.z;

      a3.x = a3.x * scale.x;
      a3.y = a3.y * scale.y;
      a3.z = a3.z * scale.z;

      a1 = add_vec(a1, pivot);
      a2 = add_vec(a2, pivot);
      a3 = add_vec(a3, pivot);

      normal.x = normal.x * 1/scale.x;
      normal.y = normal.y * 1/scale.y;
      normal.z = normal.z * 1/scale.z;
    }
};

class Mesh {
  private:
    void calc_pivot() {
      vector<float> all_x;
      vector<float> all_y;
      vector<float> all_z;
      for (int i = 0;i<triangles.size(); i++) {
        all_x.push_back(triangles[i].a1.x);
        all_x.push_back(triangles[i].a2.x);
        all_x.push_back(triangles[i].a3.x);

        all_y.push_back(triangles[i].a1.y);
        all_y.push_back(triangles[i].a2.y);
        all_y.push_back(triangles[i].a3.y);

        all_z.push_back(triangles[i].a1.z);
        all_z.push_back(triangles[i].a2.z);
        all_z.push_back(triangles[i].a3.z);

      pivot = {(min(all_x) + max(all_x)) / 2, (min(all_y) + max(all_y))/2, (min(all_z) + max(all_z)) / 2};
      }
    }
  public:
    vector<Triangle> triangles;
    Vector pivot;
    Mesh(vector<Triangle> t) {
      triangles = t;
      calc_pivot(); 
    }
    void rotate_x(float angle) {
      for (int i = 0; i<triangles.size(); i++) {
        triangles[i].a1 = rotate_around_x(triangles[i].a1, angle, pivot);
        triangles[i].a2 = rotate_around_x(triangles[i].a2, angle, pivot);
        triangles[i].a3 = rotate_around_x(triangles[i].a3, angle, pivot);
        triangles[i].normal = rotate_around_x(triangles[i].normal, angle, {0,0,0});
      }
      calc_pivot();
    }
    void rotate_y(float angle) {
      for (int i = 0; i<triangles.size(); i++) {
        triangles[i].a1 = rotate_around_y(triangles[i].a1, angle, pivot);
        triangles[i].a2 = rotate_around_y(triangles[i].a2, angle, pivot);
        triangles[i].a3 = rotate_around_y(triangles[i].a3, angle, pivot);
        triangles[i].normal = rotate_around_y(triangles[i].normal, angle, {0,0,0});
      }
      calc_pivot();
    }

    void rotate_z(float angle) {
      for (int i = 0; i<triangles.size(); i++) {
        triangles[i].a1 = rotate_around_z(triangles[i].a1, angle, pivot);
        triangles[i].a2 = rotate_around_z(triangles[i].a2, angle, pivot);
        triangles[i].a3 = rotate_around_z(triangles[i].a3, angle, pivot);
        triangles[i].normal = rotate_around_z(triangles[i].normal, angle, {0,0,0});
      }
      calc_pivot();
    }
    void scale(Vector factor) {
      for (int i = 0; i<triangles.size(); i++) {
        triangles[i].scale(factor, pivot);
      }
      calc_pivot();
    }

    void translate(Vector t_vec) {
      for (int i =0; i<triangles.size(); i++) {
        triangles[i].translate(t_vec);
      }
      calc_pivot();
    }
};
 
Vector gen_rand_vec() {
  Vector random_vec = {random_double(-1, 1), random_double(-1, 1), random_double(-1, 1)};
  while (magnitude(random_vec) > 1) {
    random_vec = {random_double(-1, 1), random_double(-1, 1), random_double(-1, 1)};
  }
  return normalize_vec(random_vec);
}

class Object {
  public:
    Sphere sphere;
    Triangle triangle;
    Material material;
    Vector normal;
    bool is_sphere;
    bool is_object;
    float min_time;
    Object(Sphere s, float m) {
      sphere = s;
      material = sphere.material;
      is_sphere = true;
      is_object = true;
      min_time = m;
    }
    Object(Triangle t, float m) {
      triangle = t;
      material = triangle.material;
      is_sphere = false;
      is_object = true;
      min_time = m;
      normal = triangle.normal;
    }
    Object() {
      is_object = false;
    }
    void calc_normal(Vector coords) {
      normal = normalize_vec(sub_vec(coords, sphere.center));
    }
};

Object return_intersection(vector<Sphere> spheres, vector<Triangle> triangles, Ray ray) {
  float min_time = INFINITY;
  Object closest_object;
  for (int i = 0; i<spheres.size(); i++) {
    float t = ray_sphere_interception(ray.direction, ray.start_pos, spheres[i].center, spheres[i].radius, ray.starting_t);
    if (t > 0 && t < min_time) {
      min_time = t;
      closest_object = Object(spheres[i], min_time);
    }
  }
 
  for (int i = 0; i<triangles.size(); i++) {
    float t = ray_plane_interception(ray.start_pos, triangles[i].a1, triangles[i].normal, ray.direction, ray.starting_t);
    if (t>0 && t < min_time) {
      Vector triang_coords = get_coords_from_ray((ray.start_pos), t, ray.direction);
      bool istriangle = point_on_triangle(triangles[i].a1, triangles[i].a2, triangles[i].a3, triang_coords);
      if (istriangle) {
        min_time = t;
        closest_object = Object(triangles[i], min_time);
      }
    }
  }

  return closest_object;
}

bool shadow_calc(vector<Light> lights, vector<Sphere> spheres, vector<Triangle> triangles, Object closest_object, Ray ray) {
  Vector coords = get_coords_from_ray(ray.start_pos, closest_object.min_time, ray.direction);
  bool in_shadow = false;
  for (int l = 0; l<lights.size(); l++) {
    Vector something = sub_vec(lights[l].position, coords);
    for (int k = 0; k<spheres.size(); k++) {
      float time;
      if (spheres[k].material.islight > 0) {
        continue;
      } else {
        time = ray_sphere_interception(something, coords, spheres[k].center, spheres[k].radius, 0.001);
        if (time > 0 && time < 1) {
          in_shadow = true;
          break;
        }
      }
    }

    if (!in_shadow) {
      for (int k = 0; k<triangles.size(); k++) {
        float time = ray_plane_interception(coords, triangles[k].a2, triangles[k].normal, something, 0.01);
        if (time > 0 && time < 1) {
          Vector triang_coords = get_coords_from_ray((coords), time, something);
          bool istriangle = point_on_triangle((triangles[k].a1), triangles[k].a2, triangles[k].a3, triang_coords);
          if (istriangle) {
            in_shadow = true;
            break;
          }
        }
      }
    }
  }

  return in_shadow;
}

Vector return_color(vector<Sphere> spheres, vector<Light> lights, Ray ray, vector<Triangle> triangles, Object closest_object, bool in_shadow) {
  Vector color = {0,0,0};
  Vector reflective_color = {0,0,0};
 
  if (closest_object.is_object) {
    if (closest_object.material.islight > 0) {
      return {
        std::min((float)closest_object.material.color[0] * closest_object.material.islight, 255.0f),
        std::min((float)closest_object.material.color[1] * closest_object.material.islight, 255.0f),
        std::min((float)closest_object.material.color[2] * closest_object.material.islight, 255.0f),
      };
    }
    Vector coords = get_coords_from_ray(ray.start_pos, closest_object.min_time, ray.direction);
    if (closest_object.is_sphere) {
      closest_object.calc_normal(coords);
    }
  
    if (!in_shadow) {
      float total_illum;
      if (closest_object.is_sphere == false) {
          closest_object.normal = scale_vec(closest_object.normal, -1);
      }
      total_illum = illumination_equation(lights, closest_object.normal, coords);
     
      color.x = closest_object.material.color[0] * total_illum > 255 ? 255 : closest_object.material.color[0] * total_illum;
      color.y = closest_object.material.color[1] * total_illum > 255 ? 255 : closest_object.material.color[1] * total_illum;
      color.z = closest_object.material.color[2] * total_illum > 255 ? 255 : closest_object.material.color[2] * total_illum;
    } else {
      color.x = 0;
      color.y = 0;
      color.z = 0;
    }
    bool did_reflect = false;
    if (closest_object.material.reflectiveness > 0) {
      ray.bounces += 1;
      if (ray.bounces <= MAX_BOUNCE_REFLECTION) {
        Ray reflected_ray = {coords, reflect(closest_object.normal, ray.direction), 1, 0.01, ray.bounces};
        Vector shiny_vec = {(float)rand(), (float)rand(), (float)rand()};
        reflected_ray.direction = normalize_vec(add_vec(reflected_ray.direction, scale_vec(normalize_vec(shiny_vec), closest_object.material.shinniness)));
        Object co = return_intersection(spheres, triangles, reflected_ray);
        bool sha = shadow_calc(lights, spheres, triangles, co, reflected_ray);
        reflective_color = return_color(spheres, lights, reflected_ray, triangles, co, sha);
      }
      did_reflect = true;
    }
    color.x = color.x * (1-closest_object.material.reflectiveness) * ray.intensity + reflective_color.x * closest_object.material.reflectiveness;
    color.y = color.y * (1-closest_object.material.reflectiveness) * ray.intensity + reflective_color.y * closest_object.material.reflectiveness;
    color.z = color.z * (1-closest_object.material.reflectiveness) * ray.intensity + reflective_color.z * closest_object.material.reflectiveness;
    if (ray.bounces < MAX_BOUNCE_REFLECTION && did_reflect == false) {
      Vector random_dir = gen_rand_vec();
      if (dot_prod(random_dir, closest_object.normal) < 0) {
        random_dir = scale_vec(random_dir, -1);
      }
      Ray diffuse_ray = {coords, normalize_vec(add_vec(closest_object.normal, random_dir)),ray.intensity, 0.001, ray.bounces};
      Object co = return_intersection(spheres, triangles, diffuse_ray);
      bool sha = shadow_calc(lights, spheres, triangles, co, diffuse_ray);
      Vector diffuse_color = return_color(spheres, lights, diffuse_ray, triangles, co, sha);
      color.x = color.x + diffuse_color.x * closest_object.material.color[0] / 255.0f;
      color.y = color.y + diffuse_color.y * closest_object.material.color[1] / 255.0f;
      color.z = color.z + diffuse_color.z * closest_object.material.color[2] / 255.0f;
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
 
  Vector camera_pos = {0,0,-2.0};
 
  vector<Triangle> cube = {
    Triangle({-0.5f, -0.5, 2.0f},{ 0.5f, -0.5, 2.0f},{-0.5f,  0.5, 2.0f},{0, {255,0,0}, 0, 0}, {0,0,1}),
    Triangle({0.5f, 0.5, 2.0f},{ 0.5f, -0.5, 2.0f},{-0.5f,  0.5, 2.0f},{0, {255,0,0}, 0, 0}, {0,0,1}),

    Triangle({-0.5f, -0.5, 3.0f},{ 0.5f, -0.5, 3.0f},{-0.5f,  0.5, 3.0f},{0, {255,0,0}, 0, 0}, {0,0,-1}),
    Triangle({0.5f, 0.5, 3.0f},{ 0.5f, -0.5, 3.0f},{-0.5f,  0.5, 3.0f},{0, {255,0,0}, 0, 0}, {0,0,-1}),

    Triangle({-0.5f, 0.5, 2.0f},{-0.5f, 0.5, 3.0f},{-0.5f,-0.5, 3.0f},{0, {255,0,0}, 0, 0}, {1,0,0}),
    Triangle({-0.5f, -0.5, 2.0f},{-0.5f, 0.5, 2.0f},{-0.5f, -0.5, 3.0f},{0, {255,0,0}, 0, 0}, {1,0,0}),

    Triangle({0.5f, 0.5, 2.0f},{0.5f, 0.5, 3.0f},{0.5f,-0.5, 3.0f},{0, {255,0,0}, 0, 0}, {-1,0,0}),
    Triangle({0.5f, -0.5, 2.0f},{0.5f, 0.5, 2.0f},{0.5f, -0.5, 3.0f},{0, {255,0,0}, 0, 0}, {-1,0,0}),

    Triangle({-0.5f, 0.5, 2.0f},{0.5f, 0.5, 2.0f},{-0.5f,0.5, 3.0f},{0, {255,0,0}, 0, 0}, {0,1,0}),
    Triangle({-0.5f, 0.5, 3.0f},{0.5f, 0.5, 2.0f},{0.5f, 0.5, 3.0f},{0, {255,0,0}, 0, 0}, {0,1,0}),

    Triangle({-0.5f, -0.5, 2.0f},{0.5f, -0.5, 2.0f},{-0.5f,-0.5, 3.0f},{0, {255,0,0}, 0, 0}, {0,-1,0}),
    Triangle({-0.5f, -0.5, 3.0f},{0.5f, -0.5, 2.0f},{0.5f, -0.5, 3.0f},{0, {255,0,0}, 0, 0}, {0,-1,0}),
  };
  Mesh cube_mesh = Mesh(cube);
  cube_mesh.rotate_y(30);
  cube_mesh.scale({0.3, 0.5, 0.3});
  cube_mesh.translate({-0.2, -0.4, 0.5});

  Mesh roof = Mesh({
    Triangle({-0.5f, -0.5, 2.0f},{0.5f, -0.5, 2.0f},{-0.5f,-0.5, 3.0f},{0, {255,255,255}, 0, 0}, {0,-1,0}),
    Triangle({-0.5f, -0.5, 3.0f},{0.5f, -0.5, 2.0f},{0.5f, -0.5, 3.0f},{0, {255,255,255}, 0, 0}, {0,-1,0}),

    Triangle({-0.5f, 0.5, 2.0f},{-0.5f, 0.5, 3.0f},{-0.5f,-0.5, 3.0f},{0, {255,0,0}, 0, 0}, {-1,0,0}),
    Triangle({-0.5f, -0.5, 2.0f},{-0.5f, 0.5, 2.0f},{-0.5f, -0.5, 3.0f},{0, {255,0,0}, 0, 0}, {-1,0,0}),

    Triangle({0.5f, 0.5, 2.0f},{0.5f, 0.5, 3.0f},{0.5f,-0.5, 3.0f},{0, {0,255,0}, 0, 0}, {1,0,0}),
    Triangle({0.5f, -0.5, 2.0f},{0.5f, 0.5, 2.0f},{0.5f, -0.5, 3.0f},{0, {0,255,0}, 0, 0}, {1,0,0}),

    Triangle({-0.5f, -0.5, 3.0f},{ 0.5f, -0.5, 3.0f},{-0.5f,  0.5, 3.0f},{0, {255,255,255}, 0, 0}, {0,0,1}),
    Triangle({0.5f, 0.5, 3.0f},{ 0.5f, -0.5, 3.0f},{-0.5f,  0.5, 3.0f},{0, {255,255,255}, 0, 0}, {0,0,1}),

    Triangle({-0.5f, 0.5, 2.0f},{0.5f, 0.5, 2.0f},{-0.5f,0.5, 3.0f},{0, {255,255,255}, 0, 0}, {0,1,0}),
    Triangle({-0.5f, 0.5, 3.0f},{0.5f, 0.5, 2.0f},{0.5f, 0.5, 3.0f},{0, {255,255,255}, 0, 0}, {0,1,0}),
  });

  roof.scale({1.2,1.2,1.2});

  Mesh light_source = Mesh({
    Triangle({-0.5f, 0.49, 2.0f},{0.5f, 0.49, 2.0f},{-0.5f,0.49, 3.0f},{1, {255,255,255}, 0, 0}, {0,-1,0}),
    Triangle({-0.5f, 0.49, 3.0f},{0.5f, 0.49, 2.0f},{0.5f, 0.49, 3.0f},{1, {255,255,255}, 0, 0}, {0,-1,0}),
  });
  light_source.scale({0.5, 0.5, 0.5});
  light_source.translate({0,0.1, 0});

  Mesh triangles = Mesh({});
  for (int i = 0; i<roof.triangles.size(); i++) {
    triangles.triangles.push_back(roof.triangles[i]);
  }
  for (int i = 0; i<cube_mesh.triangles.size(); i++) {
    triangles.triangles.push_back(cube_mesh.triangles[i]);
  }
  //
  for (int i = 0; i<light_source.triangles.size(); i++) {
    triangles.triangles.push_back(light_source.triangles[i]);
  }
 
  vector<Light> lights = {
    {1.0, POSITIONAL, {light_source.pivot.x, light_source.pivot.y, light_source.pivot.z}},
  };
 
  vector<Sphere> spheres = {
    // {{0,0,3}, 0.3, {0, {255, 0, 0}, 0, 0}},
    // {{-0.6,0,4}, 0.3, {0, {255, 0,0}, 0.5, 0.5}},
    // {{0.0,0,4}, 0.3, {0, {255, 0,0}, 0.0, 1.00}},
    // {{0.6,0,4}, 0.3, {0, {255, 0,0}, 0.5, 0.98}},
    // {{1.2,0,4}, 0.3, {0, {255, 0,0}, 1.0, 0.4}},
    // {{0,-5001,0}, 5000, {0, {128, 128, 128}, 0, 0}},
    // {lights[0].position,0.1, {1, {255, 255,255},0,0}},
  };
 
  constexpr int ray_samples = 50;
 
  for (int y = 0; y<HEIGHT; y++) {
    std::cout << y << "/" << HEIGHT << std::endl;
    for (int x = 0; x<WIDTH; x++) {
      Vector final_color = {0,0,0};
      Vector viewport_coords = {canvas_to_viewport(x, viewport_w, WIDTH),-canvas_to_viewport(y, viewport_h, HEIGHT),d};
      Vector direction = sub_vec(viewport_coords, camera_pos);

      Ray ray = {camera_pos, direction, 1, 1, 0};
      Object closest_object = return_intersection(spheres, triangles.triangles, ray);
      bool in_shadow = shadow_calc(lights, spheres, triangles.triangles, closest_object, ray);
      // bool in_shadow = false;

      for (int i = 0; i<ray_samples; i++) {
        Vector color = return_color(spheres, lights, ray, triangles.triangles, closest_object, in_shadow);
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
