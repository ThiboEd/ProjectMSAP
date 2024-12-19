#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/ray.h>

NORI_NAMESPACE_BEGIN

class DepthIntegrator : public Integrator {
public:
    DepthIntegrator(const PropertyList& props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);
        else if (scene->rayIntersect(ray, its)) {
            // Compute the distance from the origin of the camera to the intersection point 
            float d = (ray.o - its.p).norm();
            // Return 1/distance as the intensity of the pixel 
            return Color3f(1.0f / d);
        }

    }

    std::string toString() const {
        return "DepthIntegrator []";
    }
};

NORI_REGISTER_CLASS(DepthIntegrator, "depth");

NORI_NAMESPACE_END
