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

private:
    shared_ptr<hittable> object;
    matrix trans, trans_inv;
    aabb bbox;
};

#endif