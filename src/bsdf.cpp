#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
}


// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return reflectance / PI;
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi = sampler.get_sample(pdf);
  return reflectance / PI;
}


// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 3-2 Part 1 Task 2
  // Implement MirrorBSDF
  reflect(wo, wi);
  *pdf = 1;
  return reflectance / abs_cos_theta(*wi);
}


// Microfacet BSDF //

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: proj3-2, part 2
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  //Air-conductor Fresnel term is wave-length dependent
  //So the Fresnel term is a spectrum too.

  //Calculate the Fresnel terms for R,G,B channels repectively
  //to approximate the thererical calculation for every possible wavelength.

  double   cos_theta_i  = cos_theta(wi);
  Spectrum cos_theta_is(cos_theta_i, cos_theta_i, cos_theta_i);
  
  double   cos_theta_i2  = cos_theta_i * cos_theta_i;
  Spectrum cos_theta_i2s(cos_theta_i2, cos_theta_i2, cos_theta_i2); 

  Spectrum eta2_plus_k2 = eta * eta + k * k;

  Spectrum unit_s (1., 1., 1.);
  
  Spectrum Rs =(eta2_plus_k2 - 2*eta*cos_theta_i + cos_theta_i2s)
               /
               (eta2_plus_k2 + 2*eta*cos_theta_i + cos_theta_i2s);
  
  Spectrum Rp =
    (eta2_plus_k2*cos_theta_i2 - 2*eta*cos_theta_i + unit_s)
    /
    (eta2_plus_k2*cos_theta_i2 + 2*eta*cos_theta_i + unit_s);
  
  return (Rs + Rp) / 2.f;
}

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
    return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: proj3-2, part 2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  //return std::pow(cos_theta(h), 100.0);
  
  //double theta_h = getTheta(h);
  //double tan_theta_h = tan(theta_h);
  float sin_theta_h = sin_theta(h);
  float cos_theta_h = cos_theta(h) + EPS_F;
  float tan_theta_h = sin_theta_h / cos_theta_h;
  
  float exp_h = tan_theta_h / alpha;
  exp_h *= exp_h;
  
  return exp(-exp_h) / (PI * alpha * alpha * pow(cos_theta_h, 4));
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: proj3-2, part 2
  // Implement microfacet model here.

  //validate wo and wi
  if(wo.z < 0.+EPS_F || wi.z < 0.+EPS_F) return Spectrum();

  Vector3D h = (wo + wi); h.normalize();
  
  Spectrum Ff = F(wi); //Fresnel part
  double   Gf = G(wo, wi);
  double   Df = D(h);

  Spectrum fr = Ff * Gf * Df / (4. * wo.z * wi.z); 
  return fr;
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    // TODO: proj3-2, part 2
    // *Importance* sample Beckmann normal distribution function (NDF) here.
    // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
    //       and return the sampled BRDF value.
  /*
  if(wo.z < 0. + EPS_F)
    {
      *pdf = 0. + EPS_F;
      return Spectrum();
    }
  Vector2D r = sampler.get_sample();
  
  float theta_h = atan(sqrt( -1.f * alpha * alpha * log(1. - r.x - EPS_F)));
  float phi_h   = 2. * PI * r.y;

  Vector3D wh;
  wh.x = sin(theta_h) * cos(phi_h);
  wh.y = sin(theta_h) * sin(phi_h);
  wh.z = cos(theta_h);  
  
  //half vector reflect  
  (*wi) = (2.f * dot(wo, wh)) * wh - wo;
  if((*wi).z < 0. + EPS_F)
    {
      //sampled wi is invalid
      *pdf = 0. + EPS_F;
      return Spectrum();
    }
  
  //wi is valid, calculate pdf
  
  float e = tan(theta_h)/alpha; e *= e;
  float p_theta_exp = exp( -e );
  float p_theta_h = 2. * sin(theta_h) * p_theta_exp
                    /
                    (alpha * alpha * pow(cos(theta_h), 3));
  float p_phi_h = 1.f / (2 * PI);
  
  float p_h = p_theta_h * p_phi_h / sin(theta_h);
  *pdf = p_h / (4. * dot(*wi, wh));

  */
  
  *wi = cosineHemisphereSampler.get_sample(pdf);
  if((*wi).z < EPS_F)
    {
      *pdf = 0. + EPS_F;
      return Spectrum();
    }
  return MicrofacetBSDF::f(wo, *wi);
}


// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 3-2 Part 1 Task 4
  // Compute Fresnel coefficient and either reflect or refract based on it.
  Vector3D wi_refraction;
  if(refract(wo, &wi_refraction, ior))
    {
      //Schlick's approximation of air-dielectric interfaces
      //So the Fresnel term is wave-length independent. It's just a
      //scalar.
      float R0 = (1.f - ior)/(1.f + ior); R0 *= R0;
      float R = R0 + (1.f - R0) * pow(1 - abs_cos_theta(wo), 5);
      if(coin_flip(R))
	{
	  //reflection
	  reflect(wo, wi);
	  *pdf = R;
	  return R * reflectance / abs_cos_theta(*wi);
	}
      else
	{
	  //refraction
	  *wi = wi_refraction;
	  *pdf = 1.f - R;
	  float eta = (wo.z > 0) ? (1.f / ior) : ior;
	  return (1.f - R) * transmittance / abs_cos_theta(*wi) / (eta * eta);
	}
    }
  else
    {
      //internal reflection
      reflect(wo, wi);
      *pdf = 1;
      return reflectance / abs_cos_theta(*wi);
    }
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 3-2 Part 1 Task 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(-wo[0],-wo[1],wo[2]);
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO: 3-2 Part 1 Task 3
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When wo.z is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  float eta = (wo.z > 0) ? (1.f / ior) : ior;

  float predicate = 1 - eta * eta * (1 - wo.z * wo.z);
  
  if(predicate < 0) return false; //total internal reflection

  (*wi).x = - eta * wo.x;
  (*wi).y = - eta * wo.y;
  (*wi).z = (wo.z < 0) ? sqrt(predicate) : -sqrt(predicate);
  
  return true;
}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
