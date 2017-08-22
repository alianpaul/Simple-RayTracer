#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a = r.d.norm2();
  Vector3D oc = r.o - this->o;
  double b = 2 * dot(oc, r.d);
  double c = oc.norm2() - this->r2;

  double predicate = b*b - 4*a*c;
  if(predicate < 0) return false;
  
  double sqv = sqrt(predicate);
  t1 = (-b - sqv) / (2 * a);
  t2 = (-b + sqv) / (2 * a);
  
  return true;
}

bool Sphere::intersect(const Ray& r) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  
  double t1, t2;
  if(!test(r, t1, t2)) return false;

  if( r.min_t < t1 && t1 < r.max_t )
    {
      r.max_t = t1;
    }
  else if( r.min_t > t1 && r.min_t < t2 && t2 < r.max_t )
    {
      r.max_t = t2;
    }
  else
    {
      return false;
    }
  return true;
}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  /**/
  double t1, t2;
  if(!test(r, t1, t2)) return false;

  Vector3D hit_point;
  if( r.min_t < t1 && t1 < r.max_t )
    {
      r.max_t = t1;
      i->t = t1;
      hit_point = r.at_time(t1);
    }
  else if( r.min_t > t1 && r.min_t < t2 && t2 < r.max_t)
    {
      r.max_t = t2;
      i->t = t2;
      hit_point = r.at_time(t2);
    }
  else
    { 
      return false;
    }
  
  Vector3D normal = hit_point - (this->o);
  normal.normalize();
  i->n = normal;
  i->primitive = this;
  i->bsdf = get_bsdf();
  return true;
}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
