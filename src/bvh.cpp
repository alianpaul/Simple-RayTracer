#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c);
  } else {
    draw(node->l, c);
    draw(node->r, c);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c);
  } else {
    drawOutline(node->l, c);
    drawOutline(node->r, c);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // Part 2, Task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox centroid_box, bbox;

  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }

  BVHNode *node = new BVHNode(bbox);
  
  if(prims.size() <= max_leaf_size)
    {
      //leaf node
      node->prims = new vector<Primitive *>(prims);
    }
  else
    {
      //internal node, try 3 axises
      
      //try 3 axises; 0: x, 1: y, 2: z
      double min_cost_axis = INF_D;
      vector<Primitive *> left_primitives;
      vector<Primitive *> right_primitives;
      size_t bin_cnt = 16;
      
      for(size_t axis = 0; axis < 3; ++axis)
	{
	  double                       bin_extent;
	  double                       bin_min;
	  vector<BBox>                 bin_bbox(bin_cnt);
	  vector<vector<Primitive *> > bin_prim(bin_cnt);
	  
	  if(axis == 0)
	    {
	      bin_extent = bbox.extent.x / (double)bin_cnt;
	      bin_min    = bbox.min.x;
	    }
	  else if(axis == 1)
	    {
	      bin_extent = bbox.extent.y / (double)bin_cnt;
	      bin_min    = bbox.min.y;
	    }
	  else
	    {
	      bin_extent = bbox.extent.z / (double)bin_cnt;
	      bin_min    = bbox.min.z;
	    }
	    

	  //put each primitive into its according bin
	  for (Primitive *p : prims)
	    {
	      BBox bb = p->get_bbox();
	      Vector3D c = bb.centroid();

	      double pos;
	      if(axis == 0)
		pos = c.x;
	      else if(axis == 1)
		pos = c.y;
	      else
		pos = c.z;

	      size_t bin_idx = (pos - bin_min) / bin_extent;
	      if(bin_idx == bin_cnt) bin_idx--;
	      bin_bbox[bin_idx].expand(bb);
	      bin_prim[bin_idx].push_back(p);
	    }

	  //find the best bin split
	  size_t edge = 0;
	  double min_cost_bin = INF_D;
	  //check all candidate bin split
	  for(size_t i = 0; i < bin_cnt - 1; ++i)
	    {
	      //left bin aggregate
	      BBox   bin_bbox_agg_l;
	      size_t bin_primcnt_agg_l = 0;
	      for(size_t li = 0; li <= i; ++li)
		{
		  bin_bbox_agg_l.expand(bin_bbox[li]);
		  bin_primcnt_agg_l += bin_prim[li].size();
		}
	      
	      //right bin aggreate
	      BBox   bin_bbox_agg_r;
	      size_t bin_primcnt_agg_r = 0;
	      for(size_t ri = i + 1; ri < bin_cnt; ++ri)
		{
		  bin_bbox_agg_r.expand(bin_bbox[ri]);
		  bin_primcnt_agg_r += bin_prim[ri].size();
		}

	      if(bin_primcnt_agg_r == 0|| bin_primcnt_agg_l == 0 ) continue;

	      double cost = bin_bbox_agg_l.surface_area() * bin_primcnt_agg_l +
		            bin_bbox_agg_r.surface_area() * bin_primcnt_agg_r;
	      if(cost < min_cost_bin)
		{
		  min_cost_bin = cost;
		  edge = i;
		}
	    }

	  //udpate the min cost across axis
	  if(min_cost_bin < min_cost_axis)
	    {

	      min_cost_axis = min_cost_bin;
	      left_primitives.clear();
	      right_primitives.clear();

	      for(size_t li = 0; li <= edge; ++li)
		{
		  vector<Primitive *>::iterator le = left_primitives.end();
		  left_primitives.insert(le, bin_prim[li].cbegin(), bin_prim[li].cend());
		}

	      for(size_t ri = edge + 1; ri < bin_cnt; ++ri)
		{
		  vector<Primitive *>::iterator re = right_primitives.end();
		  right_primitives.insert(re, bin_prim[ri].cbegin(), bin_prim[ri].cend());
		}
	    }
	  
	}
	
      node->l = construct_bvh(left_primitives, max_leaf_size);
      node->r = construct_bvh(right_primitives, max_leaf_size);
      
    }
  return node;
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {
  // Part 2, task 3: replace this.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0 = ray.min_t, t1 = ray.max_t;
  if(!node->bb.intersect(ray, t0, t1)) return false;
  
  if(node->isLeaf()){
    for(Primitive *p : *(node->prims)){
      total_isects++;
      if(p->intersect(ray))
	return true;
    }
    return false;
  }

  //not leaf
  if(intersect(ray, node->l) || intersect(ray, node->r))
    return true;

  return false;
  
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {
  // Part 2, task 3: replace this
  
  double t0 = ray.min_t, t1 = ray.max_t;
  if(!node->bb.intersect(ray, t0, t1)) return false;

  bool hit = false;
  if(node->isLeaf()){
    for(Primitive *p : *(node->prims)){
      total_isects++;
      if(p->intersect(ray, i))
	hit = true;
    }
    return hit;
  }

  //not leaf
  bool hit1 = intersect(ray, i, node->l);
  bool hit2 = intersect(ray, i, node->r);
  return (hit1 || hit2);
}

}  // namespace StaticScene
}  // namespace CGL
