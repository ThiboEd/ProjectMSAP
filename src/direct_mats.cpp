#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>


NORI_NAMESPACE_BEGIN

class DirectMaterialSampling : public Integrator {
public:
    DirectMaterialSampling(const PropertyList& props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        //Color black as default
		Color3f Lo(0.0f); 

		//Find the intersection point that is visible in the request direction
		Intersection its;

		// If ray does not intersect with scene, assume the background
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);
        
        if (its.mesh->isEmitter()) {   // if it intersects with an emitter, return the radiance of the emitter (end of the path)
            EmitterQueryRecord emitterQR(its.p);
			emitterQR.wi = ray.d;
			emitterQR.n = its.shFrame.n;
            return its.mesh->getEmitter()->eval(emitterQR);
        }
        // if it is not an emitter
        // sample the brdf
        BSDFQueryRecord bsdfQR(its.toLocal(-ray.d), sampler->next2D());
        // BSDF intersection
        Color3f brdfSample = its.mesh->getBSDF()->sample(bsdfQR, sampler->next2D()); 
        // check if the brdf sample is valid 
        if (brdfSample.isZero() || brdfSample.hasNaN()) {   // if it is not valid, return black
            return Color3f(0.0f);
        } 
        // now create a new ray with the sampled direction
        Ray3f rayBSDF(its.p, its.toWorld(bsdfQR.wo));
        // check if the ray intersects with anything at all
        Intersection its_bsdf;

        if (scene->rayIntersect(rayBSDF, its_bsdf)) {
            if(its_bsdf.mesh->isEmitter()) {     // if the ray intersects with an emitter, we will add the radiance of the emitter to the radiance we will return
                EmitterQueryRecord emitterBSDF(its_bsdf.p);
                emitterBSDF.ref = rayBSDF.o;
                emitterBSDF.wi = rayBSDF.d;
                emitterBSDF.n = its_bsdf.shFrame.n;
                emitterBSDF.dist = its_bsdf.t;
                // calculate the radiance of the emitter to compute the contribution to the returned radiance
                Color3f Le = its_bsdf.mesh->getEmitter()->eval(emitterBSDF);
                Lo = Le * brdfSample;
                return Lo;          
            } else { //no light reaches it
			    return Color3f(0.0f);
            }
        } 
        else { 
             // if the bounced ray doesnt intersect with nothing
            Lo = scene->getBackground(rayBSDF) * brdfSample;
			return Lo;
        }      
    }

    std::string toString() const {
        return "Direct Mats Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");

NORI_NAMESPACE_END