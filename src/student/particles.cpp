#include "../scene/particles.h"
#include "../rays/pathtracer.h"
#include "../rays/trace.h"
#include "../rays/tri_mesh.h"
#include <iostream>
#include <cstdio>
using namespace PT;

Vec3 getacc(float t, float dur, Vec3 pos, Vec3 accbypos, Vec3 accbytime) {
    Vec3 acc = Vec3{0.0f, -9.8f, 0.0f};
    if(pos.x > -0.5f && pos.x < 0.5f) {
        acc += accbypos;
    }
    if(t < dur) {
        acc += accbytime;
    }
    return acc;
}

bool Scene_Particles::Particle::update(const PT::Object& scene, float lifetime, float dt,
                                       float radius, int method, float weight, float dampexp,
                                       float forceexp, float timeforceexp, float timeforcedur) {

    float tLeft = dt;
    float eps = 1e-3;
    float t = lifetime - age;
    Ray ray;
    Trace tr;
    Vec3 qx, f_norm, f_tan;
    Vec3 acc, k1, k2, k3, k4;
    Vec3 accbypos = forceexp / weight * forcefield;
    Vec3 accbytime = timeforceexp / weight * timeforce;

    while((tLeft - eps) > 0)
    {
        t = lifetime - age + dt - tLeft;
        ray.point = pos;
        ray.dir = velocity;
        ray.r = radius / 2.0f;
        tr = scene.hit(ray);

        if(!tr.hit || (tr.hit && tr.distance > velocity.norm() * eps))
        {
            if(method == 0) {
                // forward eular
                acc = getacc(t, timeforcedur, pos, accbypos, accbytime);
                pos += eps * velocity;
                velocity += eps * acc;
            } else if(method == 1) {
                // backward eular
                acc = getacc(t, timeforcedur, pos, accbypos, accbytime);
                velocity += eps * acc;
                pos += eps * velocity;
            } else if(method == 2) {
                // semi symplectic eular
                pos += eps * velocity;
                acc = getacc(t, timeforcedur, pos, accbypos, accbytime);
                velocity += eps * acc;
            } else if(method == 3) {
                // Runge-Kutta
                Vec3 velocity_old = velocity;
                Vec3 vel0, vel1, vel2, vel3;
                acc = getacc(t, timeforcedur, pos, accbypos, accbytime);
                k1 = eps * velocity;
                vel0 = velocity + eps * acc;

                acc = getacc(t + eps / 2.0f, timeforcedur, pos + k1 / 2.0f, accbypos, accbytime);
                vel1 = velocity + eps * acc / 2.0f;
                k2 = eps * vel1;
                vel1 = velocity + eps * acc;

                acc = getacc(t + eps / 2.0f, timeforcedur, pos + k2 / 2.0f, accbypos, accbytime);
                vel2 = velocity + eps * acc / 2.0f;
                k3 = eps * vel2;
                vel2 = velocity + eps * acc;

                acc = getacc(t + eps, timeforcedur, pos + k3, accbypos, accbytime);
                vel3= velocity + eps * acc;
                k4 = eps * vel3;

                pos += (k1 + 2.0f * k2 + 2.0f * k3 + k4) / 6.0f;
                velocity = (vel0 + 2.0f * vel1 + 2.0f * vel2 + vel3) / 6.0f;
            }
        }
        else
        {
            float v0 = velocity.norm();    
            f_norm = -dot(velocity,tr.normal) * tr.normal;
            f_tan = velocity + f_norm;
            velocity = (200.0f - weight) / (200.0f + weight) * f_norm + f_tan * (1.0f - dampexp);
            pos = tr.position + (eps - tr.distance / v0) * velocity;
            velocity += eps * acc;
        }
        tLeft -= eps;
    }
    age -= dt;
    return age > 0;
}
