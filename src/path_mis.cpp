#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>


NORI_NAMESPACE_BEGIN

class PathTracingMIS : public Integrator {
public:
    PathTracingMIS(const PropertyList& props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        //Color black as default
        Color3f Lo_ems(0.);
        Color3f Lo_bsdf(0.);

		//Find the intersection point that is visible in the request direction
		Intersection its;

		// If ray does not intersect with scene, assume the background
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);
        
        if (its.mesh->isEmitter()) {   // if it intersects with an emitter, return the radiance of the emitter (end of the path)
            EmitterQueryRecord emitterQR(its.mesh->getEmitter(), its.p, its.p, its.shFrame.n, its.uv);
            return its.mesh->getEmitter()->eval(emitterQR);
        }
        // if it is not an emitter

        // --- Direct Illumination via Emitter Sampling ---
        
        float ems_pdf(1.);
        float ems_wmat_pdf(1.);
        
        /// array containing all lights
		const std::vector<Emitter *>& lights = scene->getLights();
		
		size_t n_emitters = lights.size();
		
        if (n_emitters == 0) {
			return Color3f(0.0f); // No emitters, retorno negro
		}

		//Probablity of choosing a lightsource
		float pdflight_ems = 1.0f / n_emitters;
		
		//The intersection surface it is also an emitter
		EmitterQueryRecord emitterRecord_ems(its.p); 

		//Choose a random emitter light
		const Emitter* em_ems = scene->sampleEmitter(sampler->next1D(), pdflight_ems);
        emitterRecord_ems.emitter = em_ems;

		//Sample a point on the emitter and get the randiance in that direction
		Color3f Le_ems = (em_ems->sample(emitterRecord_ems, sampler->next2D(), 0.));   
        
		//emitterRecord.wi is the direction vector from the intersection point (its.p) to the light source.
		Vector3f wi_ems = emitterRecord_ems.wi;

		//We create a shadow ray (shadowRay) that starts at the intersection point its.p and goes in the direction of wi
		Ray3f shadowRay_ems(its.p, wi_ems, Epsilon, emitterRecord_ems.dist - Epsilon);

		//shadowRay.maxt is set to the distance between the intersection point its.p and the position of the light source emitterRecord.p, normalized by .norm().
		shadowRay_ems.maxt = (emitterRecord_ems.p - its.p).norm() - Epsilon;
        shadowRay_ems.mint = Epsilon;

		Intersection its_sh_ems;
		bool inter_shadow_ems = scene->rayIntersect(shadowRay_ems, its_sh_ems); //Check if the ray intersect with the scene

		if (!inter_shadow_ems) { // No occlusion

			BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord_ems.wi), its.uv, ESolidAngle);
            float den_ems = pdflight_ems * emitterRecord_ems.pdf; //We check if he denominator is dif from zero, to avoid extra calculations
			
            if(den_ems > Epsilon) {
                // In direct_ems we sample one lightsource, and we divide Lo by the emitterRecord.pdf and pdflight. 
			    emitterRecord_ems.dist = (its.p - emitterRecord_ems.p).norm(); //actualizamos el valor del emitter.dist
               
                ems_pdf = em_ems->pdf(emitterRecord_ems) *pdflight_ems;  
                ems_wmat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord_ems);
                Lo_ems += (Le_ems * its.shFrame.n.dot(emitterRecord_ems.wi) * its.mesh->getBSDF()->eval(bsdfRecord_ems)) / (den_ems*ems_pdf);
            }
        }

        // --- Indirect Illumination via BSDF Sampling ---
        float bsdf_wems_pdf(0.);
        float bsdf_pdf(0.);
        // sample the brdf
        BSDFQueryRecord bsdfQR(its.toLocal(-ray.d), sampler->next2D());
        // BSDF intersection
        Color3f brdfSample = its.mesh->getBSDF()->sample(bsdfQR, sampler->next2D()); 
        // check if the brdf sample is valid 
        if (brdfSample.isZero() || brdfSample.hasNaN()) {   // if it is not valid, return black
            return Lo_bsdf;
        } 
        // now create a new ray with the sampled direction
        Ray3f rayBSDF(its.p, its.toWorld(bsdfQR.wo));
        // check if the ray intersects with anything at all
        Intersection its_bsdf;

        if (!scene->rayIntersect(rayBSDF, its_bsdf)) {
            bsdf_pdf = its.mesh->getBSDF()->pdf(bsdfQR);
            return (scene->getBackground(rayBSDF) * brdfSample)/bsdf_pdf; }
        else if(its_bsdf.mesh->isEmitter() && bsdfQR.measure == EDiscrete) {     // if the ray intersects with an emitter, we will add the radiance of the emitter to the radiance we will return
            EmitterQueryRecord emitterBSDF(its_bsdf.mesh->getEmitter(), its_bsdf.p, its_bsdf.p, its_bsdf.shFrame.n, its_bsdf.uv);

            bsdf_pdf = its.mesh->getBSDF()->pdf(bsdfQR);
            bsdf_wems_pdf = its_bsdf.mesh->getEmitter()->pdf(emitterBSDF);
            
            // calculate the radiance of the emitter to compute the contribution to the returned radiance
            Color3f Le = its_bsdf.mesh->getEmitter()->eval(emitterBSDF);
            Lo_bsdf = Le * brdfSample;        
        }
        ////////////////////////////////////////// Direct_mis inetgrator //////////////////////////////////////////////
        // Compute MIS weights

        float w_ems(0.);
        float w_bsdf(0.);

        if ((ems_pdf + ems_wmat_pdf)>Epsilon)
        {
            w_ems = ems_pdf / (ems_pdf + ems_wmat_pdf);
        }
        else 
        {
            w_ems = 0.;
        }
        if ((bsdf_wems_pdf + bsdf_pdf)>Epsilon)
        {
            w_bsdf = bsdf_pdf / (bsdf_wems_pdf + bsdf_pdf);
        }
        else
        {
            w_bsdf = 0.;
        }

        float r = sampler->next1D();
        float p = brdfSample.maxCoeff();
        if(r > p) 
            return Lo_ems * w_ems + Lo_bsdf * w_bsdf;
        else
            return Li(scene,sampler,rayBSDF) * brdfSample / p;
   
    }

    std::string toString() const {
        return "Path Tracing []";
    };
};

NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");

NORI_NAMESPACE_END