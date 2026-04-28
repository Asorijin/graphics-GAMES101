//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }
void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject) const
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return Vector3f(0.0f);
    }

    if (inter.m->hasEmission()) {
        return inter.m->getEmission();
    }

    Vector3f L_dir(0.0f), L_indir(0.0f);

    Intersection lightSample;
    float pdf_light;
    sampleLight(lightSample, pdf_light);

    Vector3f x = inter.coords;
    Vector3f N = inter.normal;
    Vector3f wo = normalize(-ray.direction);
    Vector3f ws = normalize(lightSample.coords-x);
    float distanceToLight = (lightSample.coords - x).norm();

    

    Ray shadowRay(lightSample.coords, normalize(-ws));
    Intersection shadowInter = intersect(shadowRay);

    if (shadowInter.distance-distanceToLight> -0.005f) {
        Vector3f brdf = inter.m->eval(wo, ws, N);
        float cos_theta = std::max(0.f,dotProduct(N, ws));
        float cos_theta_light = std::max(0.f,dotProduct(lightSample.normal,-ws));

        L_dir = lightSample.emit * brdf * cos_theta * cos_theta_light / (distanceToLight * distanceToLight) / pdf_light;
    }

    if (get_random_float() < RussianRoulette) {
        Vector3f wi = inter.m->sample(wo, N);

        Ray indirectRay(inter.coords, wi);
        Intersection indirectInter = intersect(indirectRay);

        if (indirectInter.happened && !indirectInter.m->hasEmission()) {
            L_indir = castRay(indirectRay, depth + 1) * inter.m->eval(wi, wo, N)
                * std::max(0.f,(dotProduct(wi, N))) / inter.m->pdf(wi, wo, N) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}