#ifndef MATRIX_H
#define MATRIX_H

#include "rtweekend.h"

class matrix // orthogonal matrix
{
public:
    double m[4][4];

    matrix() : m{0} {}
    matrix(vec3 p, bool is_point) : m{0}
    {
        m[0][0] = p.x();
        m[0][1] = p.y();
        m[0][2] = p.z();
        m[0][3] = is_point;
    }

    matrix(vec3 euler_xyz, vec3 offset) : m{0} // return the orthogonal matrix of rotation and translation
    {
        matrix translate;
        translate.m[3][0] = offset.x();
        translate.m[3][1] = offset.y();
        translate.m[3][2] = offset.z();
        translate.m[3][3] = 1;
        translate.m[0][0] = 1;
        translate.m[1][1] = 1;
        translate.m[2][2] = 1;

        double alpha, beta, theta;
        alpha = degrees_to_radians(euler_xyz[0]);
        beta = degrees_to_radians(euler_xyz[1]);
        theta = degrees_to_radians(euler_xyz[2]);

        matrix rotation[3];
        rotation[0].m[0][0] = 1;
        rotation[0].m[1][1] = std::cos(alpha);
        rotation[0].m[2][1] = -std::sin(alpha);
        rotation[0].m[1][2] = std::sin(alpha);
        rotation[0].m[2][2] = std::cos(alpha);

        rotation[1].m[0][0] = std::cos(beta);
        rotation[1].m[1][1] = 1;
        rotation[1].m[2][0] = std::sin(beta);
        rotation[1].m[0][2] = -std::sin(beta);
        rotation[1].m[2][2] = std::cos(beta);

        rotation[2].m[0][0] = std::cos(theta);
        rotation[2].m[1][0] = -std::sin(theta);
        rotation[2].m[0][1] = std::sin(theta);
        rotation[2].m[1][1] = std::cos(theta);
        rotation[2].m[2][2] = 1;

        matrix rotate = rotation[2] * rotation[1] * rotation[0];
        rotate.m[3][3] = 1;
        matrix result = translate * rotate;

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                m[i][j] = result.m[i][j];
            }
        }
    }

    inline vec3 to_vec3()
    {
        return vec3(m[0][0], m[0][1], m[0][2]);
    }

    matrix transpose()
    {
        matrix result;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result.m[j][i] = m[i][j];
            }
        }
        return result;
    }

    matrix inverse()
    {
        // can only solve the inverse of a orthogonal transform matrix
        // cannot solve the inverse of any arbitary 4x4 matrix

        matrix rotate = *this;
        matrix trans = *this;

        for (int i = 0; i < 3; i++)
        {
            rotate.m[3][i] = 0;
        }
        rotate = rotate.transpose();

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (i != j)
                {
                    trans.m[i][j] = 0;
                }
                else
                {
                    trans.m[i][j] = 1;
                }
            }
            trans.m[3][i] = -trans.m[3][i];
        }
        return rotate * trans;
    }

    matrix operator*(const matrix &a)
    {
        matrix result;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result.m[i][j] = 0;
                for (int k = 0; k < 4; k++)
                {
                    result.m[i][j] += m[k][j] * a.m[i][k];
                }
            }
        }
        return result;
    }
};

inline matrix operator*(const matrix &a, const matrix &b)
{
    matrix result;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result.m[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                result.m[i][j] += a.m[k][j] * b.m[i][k];
            }
        }
    }
    return result;
}

#endif