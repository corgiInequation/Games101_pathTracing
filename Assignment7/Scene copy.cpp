//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <random>

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
        float &tNear, uint32_t &index, Object **hitObject)
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
    // TO DO Implement Path Tracing Algorithm here
    //ray.dir是wo, 然后ray碰到的点为p   
    Vector3f hit_color = {0.0f, 0.0f, 0.0f};

    //先发光，求出交点
    Intersection hit_pos = intersect(ray);
    if(!hit_pos.happened)
    {
        return hit_color;
    }

    if(dotProduct(ray.direction, normalize(hit_pos.normal)) > 0)
    {
        hit_pos.coords = hit_pos.coords - EPSILON * normalize(hit_pos.normal);
    }
    else
    {
        hit_pos.coords = hit_pos.coords + EPSILON*normalize(hit_pos.normal);
    }
    //对光源求积分
    Intersection lightPos;
    float light_pdf;
    sampleLight(lightPos, light_pdf);
    Vector3f lightDir = lightPos.coords - hit_pos.coords;
    float lightDirDis = std::sqrt(dotProduct(lightDir, lightDir));
    lightDir = normalize(lightDir);
    Ray p_to_light(hit_pos.coords, lightDir);
    Intersection block_pos = intersect(p_to_light);    
    
    if(!block_pos.happened || block_pos.distance  > lightDirDis + EPSILON)
    {    
        hit_color += lightPos.emit*hit_pos.m->eval(ray.direction, lightDir, normalize(hit_pos.normal))\
            *std::max(dotProduct(lightDir, normalize(hit_pos.normal)), 0.0f)*std::max(dotProduct(-lightDir, normalize(lightPos.normal)), 0.0f)\
            /std::pow(lightDirDis, 2.0f)/light_pdf;
    }

    //对相交的物体求积分
    float random_num = get_random_float();
    if(random_num > RussianRoulette)
    {
        return hit_color;
    }
    Vector3f dir_wi = normalize(hit_pos.m->sample(ray.direction, hit_pos.coords));
    Ray wi(hit_pos.coords, dir_wi); 
    Intersection object_pos = intersect(wi);
    if(object_pos.happened)
    {
        hit_color += castRay(wi, depth+1)*hit_pos.m->eval(ray.direction, wi.direction, normalize(hit_pos.normal))\
                    *std::max(dotProduct(wi.direction, normalize(hit_pos.normal)),0.0f)/RussianRoulette/hit_pos.m->pdf(ray.direction, wi.direction, hit_pos.normal);
    }

    return hit_color;
}