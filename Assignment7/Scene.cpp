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
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f L_dir = {0,0,0};
    Vector3f L_indir = {0,0,0};
    Intersection shadeInter = intersect(ray);
    if(!shadeInter.happened)
    {
        return L_dir;
    }

    Vector3f shade_n = normalize(shadeInter.normal);
    Vector3f shade_p = shadeInter.coords;
    shade_p = dotProduct(ray.direction, shade_n) < 0 ? shade_p + shade_n*EPSILON : shade_p - shade_n*EPSILON;

    //printf("%f %f %f normal %f %f %f\n", shadeInter.coords.x, shadeInter.coords.y, shadeInter.coords.z, shadeInter.normal.x, shadeInter.normal.y, shadeInter.normal.z);    //光源特殊处理
    //不能想着在光线积分算，因为两个面距离太近，会导致cosi接近于0，是L_dir=0
    //求出了shaderInter.m->getEmission之后，不用再算另外两个分量，应为这样子会重复
    if(shadeInter.m->hasEmission())
    {
        return shadeInter.m->getEmission();
    }
    Intersection lightInter;
    float pdf_light;
    sampleLight(lightInter, pdf_light);
    Vector3f lightDir = normalize(lightInter.coords - shade_p);
    float lightDis2 = dotProduct(lightInter.coords - shade_p, lightInter.coords - shade_p);
    Ray p_to_light(shade_p, lightDir);
    Intersection blockInter = intersect(p_to_light);
    if(!blockInter.happened || pow(blockInter.distance,2) > lightDis2)
    {
        Vector3f emit = lightInter.emit;
        Vector3f eval = shadeInter.m->eval(ray.direction, lightDir, shade_n);
        float cosi = dotProduct(lightDir, shade_n);
        float cosii = dotProduct(normalize(lightInter.normal), -lightDir);
        //L_dir = pdf_light < 1e-8 ? Vector3f(0,0,0) : emit * eval * cosi * cosii /lightDis2/pdf_light;       
        L_dir =  emit * eval * cosi * cosii /lightDis2/pdf_light;            

    }    

    float random_num = get_random_float();
    //printf("%f\n", random_num);
    if(random_num < RussianRoulette)
    {
        Vector3f wi = shadeInter.m->sample(ray.direction, shade_n);
        Ray r_wi(shade_p, wi);        
        Intersection objInter = intersect(r_wi);

        if(objInter.happened && !objInter.m->hasEmission())
        {
            Vector3f eval = shadeInter.m->eval(ray.direction, wi, shade_n);
            Vector3f emit = castRay(r_wi, depth+1);
            float cosi = dotProduct(wi, shade_n);
            float pdf = shadeInter.m->pdf(ray.direction, wi, shade_n);            
         //   L_dir = pdf < 1e-8 ? Vector3f{0,0,0} : emit*eval*cosi/RussianRoulette/pdf;
           L_dir = emit*eval*cosi/RussianRoulette/pdf;

        }
    }
    return L_dir + L_indir;
}