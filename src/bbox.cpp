#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // Part 2, Task 2:
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  //x-axis aligned slab
  double tmin,tmax;
  double invd = 1./r.d.x;
  if(invd > 0){
    tmin = (min.x - r.o.x) * invd;
    tmax = (max.x - r.o.x) * invd;
  }else{
    tmin = (max.x - r.o.x) * invd;
    tmax = (min.x - r.o.x) * invd;
  }

  //y-axis aligned slab
  double tminy, tmaxy;
  invd = 1./r.d.y;
  if(invd > 0){
    tminy = (min.y - r.o.y) * invd;
    tmaxy = (max.y - r.o.y) * invd;
  }else{
    tminy = (max.y - r.o.y) * invd;
    tmaxy = (min.y - r.o.y) * invd;
  }

  if(tminy > tmax || tmaxy < tmin) return false;

  tmin = std::max(tmin, tminy);
  tmax = std::min(tmax, tmaxy);

  //z-axis aligned slab
  double tminz, tmaxz;
  invd = 1./r.d.z;
  if(invd > 0){
    tminz = (min.z - r.o.z) * invd;
    tmaxz = (max.z - r.o.z) * invd;
  }else{
    tminz = (max.z - r.o.z) * invd;
    tmaxz = (min.z - r.o.z) * invd;
  }

  if(tminz > tmax || tmaxz < tmin) return false;

  tmin = std::max(tmin, tminz);
  tmax = std::min(tmax, tmaxz);

  //test if intersect with given range(t0, t1)
  if(tmin > t1 || tmax < t0) return false;

  t0 = std::max(t0, tmin);
  t1 = std::min(t1, tmax);
  return true;
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
	glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
	glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
	glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace XCGL
