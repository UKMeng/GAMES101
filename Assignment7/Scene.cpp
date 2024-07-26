//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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
    // Path Tracing

    Vector3f L_indir = Vector3f(0,0,0);
    Vector3f L_dir = Vector3f(0,0,0);

    Intersection rayToScene = intersect(ray);
    if(!rayToScene.happened) return Vector3f(0,0,0);

    if (rayToScene.m->hasEmission())
    {
        if (depth == 0)
        {
            // direct to pixel
            return rayToScene.m->getEmission();
        }
        else return Vector3f(0,0,0);
    }

    Vector3f p = rayToScene.coords;
    Vector3f N = rayToScene.normal;
    Vector3f wo = ray.direction;

    Intersection lightPos;
    float pdfLight;
    sampleLight(lightPos, pdfLight);
    Vector3f x = lightPos.coords;
    Vector3f NN = lightPos.normal;
    Vector3f ws = normalize(x - p);
    Vector3f emit = lightPos.emit;
    float distance = (x - p).norm();

    Intersection rayToLight = intersect(Ray(p, ws));

    if (rayToLight.happened && ((rayToLight.distance - distance) >= -EPSILON))
    {
        // the ray not blocked in the middle way
        L_dir = emit * rayToScene.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (distance * distance) / pdfLight;
    }

    // Russian Roulette Test
    if (get_random_float() < RussianRoulette)
    {
        Vector3f wi = rayToScene.m->sample(wo, N);
        Vector3f Li = castRay(Ray(p, wi), depth + 1);
        if (Li.norm() != 0) // the ray should hit a non-emitting object
        {
            L_indir = Li * rayToScene.m->eval(wo, wi, N) * dotProduct(wi, N) / rayToScene.m->pdf(wo, wi, N) / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}