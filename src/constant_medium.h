#ifndef CONSTANT_MEDIUM_H
#define CONSTANT_MEDIUM_H

#include "hittable.h"
#include "material.h"
#include "texture.h"

class constant_medium : public hittable
{
public:
    constant_medium(shared_ptr<hittable> boundary, double density, shared_ptr<texture> tex)
        : boundary(boundary), neg_inv_density(-1 / density),
          phase_function(make_shared<isotropic>(tex))
    {
    }

    constant_medium(shared_ptr<hittable> boundary, double density, const color &aldebo)
        : boundary(boundary), neg_inv_density(-1 / density),
          phase_function(make_shared<isotropic>(aldebo))
    {
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        hit_record rec1, rec2;

        if (!boundary->hit(r, interval::universe, rec1))
            return false;

        if (!boundary->hit(r, interval(rec1.t + 0.0001, infinity), rec2))
            return false;

        if (rec1.t < 0)
            rec1.t = 0;

        auto ray_length = r.direction().length();
        auto distance_inside_boundary = (rec2.t - rec1.t) * ray_length;
        auto hit_distance = neg_inv_density * std::log(random_double());

        if (hit_distance > distance_inside_boundary)
            return false;

        rec.t = rec1.t + hit_distance / ray_length;
        rec.p = r.at(rec.t);

        rec.normal = vec3(1, 0, 0); // arbitary
        rec.front_face = true;      // also arbitary
        rec.mat = phase_function;

        return true;
    }

    aabb bounding_box() const override { return boundary->bounding_box(); }
    vec3 center() const override { return boundary->center(); }

private:
    shared_ptr<hittable> boundary;
    double neg_inv_density;
    shared_ptr<material> phase_function;
};

#endif