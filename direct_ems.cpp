#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/ray.h>

NORI_NAMESPACE_BEGIN

class DirectEmitterSampling : public Integrator {
public:
    DirectEmitterSampling(const PropertyList& props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        //Color black as default
		Color3f Lo(0.); 

		//Find the surface that is visible in the request direction
		Intersection its;

		// If ray does not intersect with scene, assume the background
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		if (its.mesh->isEmitter()) { //En caso de  que el lightsource se vea desde la camara
			EmitterQueryRecord emRec; //Creamos un emisor vacio 
			emRec.wi = ray.d;
			emRec.n = its.shFrame.n; 
			emRec.ref = ray.o;
			return its.mesh->getEmitter()->eval(emRec); 
		}

		/// Return a reference to an array containing all lights
		const std::vector<Emitter *>& lights = scene->getLights();
		// Número de emisores en la escena
		size_t n_emitters = lights.size();
		if (n_emitters == 0) {
			return Lo; // No hay emisores, retorno negro
		}

		// Probabilidad uniforme de seleccionar una luz
		float pdflight = 1.0f / n_emitters;
		//float pdflight; //Probablity of choosing a lightsource

		//The intersection surface it is also an emitter
		EmitterQueryRecord emitterRecord(its.p); 

		//Choose a random emitter light
		const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);

		//Sample a point on the emitter and get the randiance in that direction
		//Color3f Le = (em->sample(emitterRecord, sampler->next2D(), 0.))/emitterRecord.pdf;
		Color3f Le = (em->sample(emitterRecord, sampler->next2D(), 0.));   

		//emitterRecord.wi is the direction vector from the intersection point (its.p) to the light source.
		Vector3f wi = emitterRecord.wi;

		//We create a shadow ray (shadowRay) that starts at the intersection point its.p and goes in the direction of wi
		Ray3f shadowRay(its.p, wi);

		//shadowRay.maxt is set to the distance between the intersection point its.p and the position of the light source emitterRecord.p, normalized by .norm().
		shadowRay.maxt = (emitterRecord.p - its.p).norm();

		Intersection its_sh;
		bool inter_shadow = scene->rayIntersect(shadowRay, its_sh); //Check if the ray intersect with the scene

		if (inter_shadow && (its_sh.t < emitterRecord.dist)) { // If it intersects, is a shadow
		}
		else { 

			BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

			// In direct_ems we sample one lightsource, and we divide Lo by the emitterRecord.pdf and pdflight. emitterRecord.pdf is already included. 
			Lo += (Le * its.shFrame.n.dot(emitterRecord.wi) * its.mesh->getBSDF()->eval(bsdfRecord)) / (pdflight * emitterRecord.pdf);
		}

		return Lo;
        
    }

    std::string toString() const {
        return "Direct Ems Integrator []";
    }
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END