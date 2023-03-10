
#include "../rays/tri_mesh.h"
#include "../rays/samplers.h"
#include <iostream>
#include <cstdio>

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2 or 3
    // Compute the bounding box of the triangle.

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::hit.

    BBox box;
    return box;
}

float abs(float a) 
{
    if(a > 0)
        return a;
    else
        return -a;
}

Trace Triangle::hit(const Ray& ray) const {

    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;
    
    //Vec3 tnorm = (v_0.normal + v_1.normal + v_2.normal) / 3.0f;
    Vec3 tnorm = cross(v_0.position - v_1.position, v_0.position - v_2.position);
    if(dot(tnorm,ray.dir) > 0) tnorm = -tnorm;
    tnorm.normalize();
    float tmp = -dot(tnorm, ray.dir);
    tnorm = (-ray.dir) / tmp;
    v_0.position += tnorm * ray.r;
    v_1.position += tnorm * ray.r;
    v_2.position += tnorm * ray.r;
    
    Vec3 E1 = v_1.position - v_0.position;
    Vec3 E2 = v_2.position - v_0.position;
    Vec3 S = ray.point - v_0.position;
    Vec3 S1 = cross(ray.dir,E2);
    Vec3 S2 = cross(S,E1);

    float cr = dot(S1,E1);
    float t = dot(S2,E2) / cr;
    float b1 = dot(S1,S) / cr;
    float b2 = dot(S2,ray.dir) / cr;
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;         // was there an intersection?
    //ret.distance = 0.0f;   // at what distance did the intersection occur?
    //ret.position = Vec3{}; // where was the intersection?
    //ret.normal = Vec3{};   // what was the surface normal at the intersection?
                             // (this should be interpolated between the three vertex normals)
    if(t>=0 && b1>=0 && b2>=0 && 1-b1-b2 >=0)
    {
        ret.hit = true;
        ret.distance = t;
        ret.position = ray.point + ret.distance * ray.dir;

        Vec3 norm0 = v_0.position - ret.position;
        Vec3 norm1 = v_1.position - ret.position;
        Vec3 norm2 = v_2.position - ret.position;
        float s_01 = cross(norm0, norm1).norm();
        float s_12 = cross(norm1, norm2).norm();
        float s_20 = cross(norm2, norm0).norm();
        float sum = s_01 + s_12 + s_20;
        ret.normal = v_0.normal * (s_12 / sum) + v_1.normal * (s_20 / sum) + v_2.normal * (s_01 / sum);
        ret.normal.normalize();
    }
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

Vec3 Triangle::sample(Vec3 from) const {
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    Samplers::Triangle sampler(v_0.position, v_1.position, v_2.position);
    Vec3 pos = sampler.sample();
    return (pos - from).unit();
}

float Triangle::pdf(Ray wray, const Mat4& T, const Mat4& iT) const {

    Ray tray = wray;
    tray.transform(iT);

    Trace trace = hit(tray);
    if(trace.hit) {
        trace.transform(T, iT.T());
        Vec3 v_0 = T * vertex_list[v0].position;
        Vec3 v_1 = T * vertex_list[v1].position;
        Vec3 v_2 = T * vertex_list[v2].position;
        float a = 2.0f / cross(v_1 - v_0, v_2 - v_0).norm();
        float g =
            (trace.position - wray.point).norm_squared() / std::abs(dot(trace.normal, wray.dir));
        return a * g;
    }
    return 0.0f;
}

void Tri_Mesh::build(const GL::Mesh& mesh, bool bvh) {

    use_bvh = bvh;
    verts.clear();
    triangle_bvh.clear();
    triangle_list.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    if(use_bvh) {
        triangle_bvh.build(std::move(tris), 4);
    } else {
        triangle_list = List<Triangle>(std::move(tris));
    }
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh, bool use_bvh) {
    build(mesh, use_bvh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangle_bvh = triangle_bvh.copy();
    ret.triangle_list = triangle_list.copy();
    ret.use_bvh = use_bvh;
    return ret;
}

BBox Tri_Mesh::bbox() const {
    if(use_bvh) return triangle_bvh.bbox();
    return triangle_list.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    if(use_bvh) return triangle_bvh.hit(ray);
    return triangle_list.hit(ray);
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    if(use_bvh) return triangle_bvh.visualize(lines, active, level, trans);
    return 0;
}

Vec3 Tri_Mesh::sample(Vec3 from) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.sample(from);
}

float Tri_Mesh::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.pdf(ray, T, iT);
}

} // namespace PT
