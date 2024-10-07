#ifndef HITTABLE_H
#define HITTABLE_H

#include "rtweekend.h"
#include "aabb.h"

class material;

class hit_record
{
public:
    point3 p;
    vec3 normal;
    shared_ptr<material> mat;
    double t;
    double u;
    double v;
    bool front_face;

    void set_face_normal(const ray &r, const vec3 &outward_normal)
    {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.

        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class hittable
{
public:
    virtual ~hittable() = default;
    virtual bool hit(const ray &r, interval ray_t, hit_record &rec) const = 0;
    virtual aabb bounding_box() const = 0;
    virtual vec3 center() const = 0;
};

class translate : public hittable
{
public:
    translate(shared_ptr<hittable> object, const vec3 &offset)
        : object(object), offset(offset)
    {
        bbox = object->bounding_box() + offset;
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        // Move the ray backwards by the offset
        ray offset_r(r.origin() - offset, r.direction(), r.time());

        // Determine whether an intersection exists along the offset ray (and if so, where)
        if (!object->hit(offset_r, ray_t, rec))
            return false;

        // Move the intersection point forwards by the offset
        rec.p += offset;

        return true;
    }

    aabb bounding_box() const override { return bbox; }
    vec3 center() const override { return object->center() + offset; }

private:
    shared_ptr<hittable> object;
    vec3 offset;
    aabb bbox;
};

class rotate : public hittable
{
public:
    rotate(shared_ptr<hittable> object, const vec3 &euler_xyz) : object(object)
    {
        bbox = object->bounding_box();
        trans = matrix(euler_xyz, vec3(0, 0, 0));
        trans_inv = trans.inverse();
        point3 min(infinity, infinity, infinity);
        point3 max(-infinity, -infinity, -infinity);

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                for (int k = 0; k < 2; k++)
                {
                    auto x = i * bbox.x.max + (1 - i) * bbox.x.min;
                    auto y = j * bbox.y.max + (1 - j) * bbox.y.min;
                    auto z = k * bbox.z.max + (1 - k) * bbox.z.min;

                    vec3 a = (trans * matrix(vec3(x, y, z), true)).to_vec3();

                    for (int c = 0; c < 3; c++)
                    {
                        min[c] = std::fmin(min[c], a[c]);
                        max[c] = std::fmax(max[c], a[c]);
                    }
                }
            }
        }

        bbox = aabb(min, max);
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        // Transform the ray from world space to object space.
        ray object_r((trans_inv * matrix(r.origin(), true)).to_vec3(), (trans_inv * matrix(r.direction(), false)).to_vec3(), r.time());

        // Determine whether an intersection exists in object space (and if so, where).
        if (!object->hit(object_r, ray_t, rec))
            return false;

        // Transform the intersection from object space back to world space.
        rec.p = (trans * matrix(rec.p, true)).to_vec3();
        rec.normal = (trans * matrix(rec.normal, false)).to_vec3();

        return true;
    }

    aabb bounding_box() const override { return bbox; }
    vec3 center() const override { return (trans * matrix(object->center(), true)).to_vec3(); }

private:
    shared_ptr<hittable> object;
    matrix trans, trans_inv;
    aabb bbox;
};

class scale : public hittable
{
public:
    scale(std::shared_ptr<hittable> object, const vec3 &scaling_factors) : object(object)
    {
        bbox = object->bounding_box();
        trans = matrix(
            vec3(scaling_factors.x(), 0, 0),
            vec3(0, scaling_factors.y(), 0),
            vec3(0, 0, scaling_factors.z())
        );
        trans_inv = matrix(
            vec3(1 / scaling_factors.x(), 0, 0),
            vec3(0, 1 / scaling_factors.y(), 0),
            vec3(0, 0, 1 / scaling_factors.z())
        );
        
        // Calculate the scaled bounding box
        point3 min = (trans * matrix(bbox.min, true)).to_vec3();
        point3 max = (trans * matrix(bbox.max, true)).to_vec3();
        bbox = aabb(min, max);
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        // Transform the ray from world space to object space.
        ray object_r((trans_inv * matrix(r.origin(), true)).to_vec3(), 
                      (trans_inv * matrix(r.direction(), false)).to_vec3(), 
                      r.time());

        // Determine whether an intersection exists in object space (and if so, where).
        if (!object->hit(object_r, ray_t, rec))
            return false;

        // Transform the intersection from object space back to world space.
        rec.p = (trans * matrix(rec.p, true)).to_vec3();
        
        vec3 rec_normal = (trans * matrix(rec.normal, false)).to_vec3();
        
        // Normalize the normal to ensure it's a unit normal after scaling the object
        rec.normal = unit_vector(rec_normal);

        return true;
    }

    aabb bounding_box() const override { return bbox; }
    vec3 center() const override { return (trans * matrix(object->center(), true)).to_vec3(); }

private:
    std::shared_ptr<hittable> object;
    matrix trans, trans_inv;
    aabb bbox;
    vec3 scaling_factors; // Store the scaling factors for later use if needed
};

class transform: public hittable
{
public:
    transform(shared_ptr<hittable> object, const vec3 &sca, const vec3 &rot, const vec3 &trans) : object(object)
    {
        scaling = make_shared<scale>(object, sca);
        rotation = make_shared<rotate>(scaling, rot);
        translation = make_shared<translate>(rotation, trans);
        // The order of transformations is assumed to be:
        // 1. Scaling
        // 2. Rotation
        // 3. Translation
        bbox = translation->bounding_box();
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        if(!translation->hit(r, ray_t, rec))
            return false;
        return true;
    }

    aabb bounding_box() const override { return bbox; }
    vec3 center() const override { return translation->center(); }

private:
    shared_ptr<hittable> object; 
    shared_ptr<hittable> rotation, translation, scaling;  
    aabb bbox; 
};

#endif