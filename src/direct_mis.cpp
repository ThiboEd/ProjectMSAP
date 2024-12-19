#include <nori/frame.h>
#include <nori/reflectance.h>
#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMIS : public Integrator {
public:
    DirectMIS(const PropertyList& props) {
        
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

        Color3f Lo_ems(0.);
        Color3f Lo_mat(0.);

        // Find surface visible in that direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        if (its.mesh->isEmitter()) { // if it intersects with an emitter, return the radiance of the emitter (end of the path)
            EmitterQueryRecord emitterQR(its.p); 
            emitterQR.wi = ray.d;
            emitterQR.n = its.shFrame.n;
            return its.mesh->getEmitter()->eval(emitterQR);
        }

        ////////////////////////////////////////// Direct_ems integrator //////////////////////////////////////////////
 
        float ems_pdf(1.);
        float ems_wmat_pdf(1.);
        Intersection its_sh_ems;
        
        float pdflight_ems; //Probablity of choosing a lightsource

        //The intersection surface it is also an emitter
        EmitterQueryRecord emitterRecord_ems(its.p);

        //Choose a random lightsource, nature of direct_ems

        const Emitter* em_ems = scene->sampleEmitter(sampler->next1D(), pdflight_ems);

        //Get randiance in that direction
        Color3f Le_ems = em_ems->sample(emitterRecord_ems, sampler->next2D(), 0.);

        //emitterRecord.wi is the direction vector from the intersection point (its.p) to the light source.
        Vector3f wi_ems = emitterRecord_ems.wi;

        //We create a shadow ray (shadowRay) that starts at the intersection point its.p and goes in the direction of wi
        Ray3f shadowRay_ems(its.p, wi_ems, Epsilon, emitterRecord_ems.dist - Epsilon);

        //shadowRay.maxt is set to the distance between the intersection point its.p and the position of the light source emitterRecord.p, normalized by .norm().
        shadowRay_ems.maxt = (emitterRecord_ems.p - its.p).norm();

        bool inter_shadow_ems = scene->rayIntersect(shadowRay_ems, its_sh_ems); //Comprobamos si el rayo intersecta con scene

        if (inter_shadow_ems && (its_sh_ems.t < (emitterRecord_ems.dist - Epsilon))) { // Si intersecta, es sombra, valor 0
            Lo_ems(0.);
        }
        else { //En caso contrario, se calcula Lo

            BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord_ems.wi), its.uv, ESolidAngle);

            float den_ems = pdflight_ems * emitterRecord_ems.pdf; //We check if he denominator is dif from zero, to avoid extra calculations

            if (den_ems > Epsilon) {
                // In direct_ems we sample one lightsource, and we divide Lo by the emitterRecord.pdf and pdflight. emitterRecord.pdf is already included. 
                emitterRecord_ems.dist = (its.p - emitterRecord_ems.p).norm(); //actualizamos el valor del emitter.dist
               
                ems_pdf = em_ems->pdf(emitterRecord_ems) *pdflight_ems;  
                ems_wmat_pdf = its.mesh->getBSDF()->pdf(bsdfRecord_ems);//emitterBSDF_mat.pdf;
                Lo_ems = (Le_ems * its.shFrame.n.dot(emitterRecord_ems.wi) * its.mesh->getBSDF()->eval(bsdfRecord_ems)) / (den_ems*ems_pdf);
                //ems_wmat_pdf = em_ems->pdf(emitterRecord_ems) * scene->pdfEmitter(em_ems);
            }
        }
       
        

        ////////////////////////////////////////// Direct_mats inetgrator //////////////////////////////////////////////

        float mat_wems_pdf(0.);
        float mat_pdf(0.);
        Intersection its_bsdf_mat;
        //Vector3f wi_mat;

           
        //en caso de que no sea un emitter

        BSDFQueryRecord bsdfQR(its.toLocal(-ray.d), sampler->next2D());
        Color3f brdf_sample_mat = its.mesh->getBSDF()->sample(bsdfQR, sampler->next2D());//BRDF intersection

        // check if the brdf sample is valid (absorbed or invalid samples are not valid)
        if (brdf_sample_mat.isZero() || brdf_sample_mat.hasNaN()) {   // if it is not valid, shadow
        }else{
            // now create a new ray with the sampled direction
            Ray3f BSDFRay_mat(its.p, its.toWorld(bsdfQR.wo));

            if (scene->rayIntersect(BSDFRay_mat, its_bsdf_mat)) {
                if (its_bsdf_mat.mesh->isEmitter()) {
                    // // calculate the radiance of the emitter to compute the contribution to the returned radiance
                    EmitterQueryRecord emitterBSDF_mat(its_bsdf_mat.p);
                    // add the contribution to the returned radiance
                    emitterBSDF_mat.wi = BSDFRay_mat.d;
                    emitterBSDF_mat.n = its_bsdf_mat.shFrame.n;
                    emitterBSDF_mat.dist = (its.p - emitterBSDF_mat.p).norm();
                    Color3f Le_mat = its_bsdf_mat.mesh->getEmitter()->eval(emitterBSDF_mat);
                   
                    mat_pdf = its.mesh->getBSDF()->pdf(bsdfQR);
                    mat_wems_pdf = its_bsdf_mat.mesh->getEmitter()->pdf(emitterBSDF_mat);
                    Lo_mat = Le_mat * brdf_sample_mat;
                }
            }
            else{ //Rebota en el fondo
                mat_pdf = its.mesh->getBSDF()->pdf(bsdfQR);
                Lo_mat = (scene->getBackground(BSDFRay_mat) * brdf_sample_mat)/mat_pdf;

            }

        }
   
        ////////////////////////////////////////// Direct_mis inetgrator //////////////////////////////////////////////
        // Compute MIS weights

        float w_ems(0.);
        float w_mat(0.);

        if ((ems_pdf + ems_wmat_pdf)>Epsilon)
        {
            w_ems = ems_pdf / (ems_pdf + ems_wmat_pdf);
        }
        else 
        {
            w_ems = 0.;
        }
        if ((mat_wems_pdf + mat_pdf)>Epsilon)
        {
            w_mat = mat_pdf / (mat_wems_pdf + mat_pdf);
        }
        else
        {
            w_mat = 0.;
        }
       
        // Return the combined contribution with MIS weights
        return Lo_ems * w_ems + Lo_mat * w_mat;
    }

    std::string toString() const {
        return "DirectMisIntegrator[]";
    }
private:
    
};

NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END


