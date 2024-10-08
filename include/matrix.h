#ifndef MATRIX_H
#define MATRIX_H

#include "utils.h"

// orthogonal matrix
class matrix
{
public:
    double m[4][4];

    matrix()
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                m[i][j] = (i == j) ? 1 : 0; // Identity matrix
            }
        }
    }
    
    matrix(vec3 p, bool is_point) : m{0}
    {
        m[0][0] = p.x();
        m[0][1] = p.y();
        m[0][2] = p.z();
        m[0][3] = is_point;
        // in orthogonal matrix, the fourth row canbe 0 or 1, used to indicate whether it is a point or a vector
    }

    matrix(vec3 x, vec3 y, vec3 z) : m{0}
    {
        for(int i = 0; i < 3; i++)
        {
            m[0][i] = x[i];
            m[1][i] = y[i];
            m[2][i] = z[i];
        }
        m[3][3] = 1;
    }

    // euler_xyz is in degrees
    // offset is the translation vector
    matrix(vec3 euler_xyz, vec3 offset) : m{0}
    {
        matrix translate;
        translate.m[3][0] = offset.x();
        translate.m[3][1] = offset.y();
        translate.m[3][2] = offset.z();
        for (int i = 0; i < 4; i++)
        {
            m[i][i] = 1;
        }

        double alpha, beta, theta;
        alpha = degrees_to_radians(euler_xyz[0]);
        beta = degrees_to_radians(euler_xyz[1]);
        theta = degrees_to_radians(euler_xyz[2]);

        matrix rotation[3];
        for (int i = 0; i < 3; i++)
        {
            rotation[i].m[i][i] = 1;
        }
        rotation[0].m[1][1] = std::cos(alpha);
        rotation[0].m[2][1] = -std::sin(alpha);
        rotation[0].m[1][2] = std::sin(alpha);
        rotation[0].m[2][2] = std::cos(alpha);

        rotation[1].m[0][0] = std::cos(beta);
        rotation[1].m[2][0] = std::sin(beta);
        rotation[1].m[0][2] = -std::sin(beta);
        rotation[1].m[2][2] = std::cos(beta);

        rotation[2].m[0][0] = std::cos(theta);
        rotation[2].m[1][0] = -std::sin(theta);
        rotation[2].m[0][1] = std::sin(theta);
        rotation[2].m[1][1] = std::cos(theta);

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

    vec3 operator[](int i)
    {
        return vec3(m[i][0], m[i][1], m[i][2]);
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