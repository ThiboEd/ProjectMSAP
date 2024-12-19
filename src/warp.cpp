/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {

    float x = sample.x();
    float y = sample.y();

    float r = sqrt(x);
    float theta = 2 * M_PI * y;

    return Point2f(r*cos(theta) , r*sin(theta));

}

float Warp::squareToUniformDiskPdf(const Point2f &p) {

    if (pow(p.x() , 2 ) + pow(p.y() , 2) > 1)
    {return 0.0f;}

    else {
        float area = M_PI; //Since r = 1
        return 1/area;
    }
}

Point2f Warp::squareToUniformTriangle(const Point2f& sample){

    float x = sample.x();
    float y = sample.y();

    if (sample.x() + sample.y() <= 1)
    {
        Point2f sample (x , y);
        return sample;
    }
    else 
    {
        Point2f sample(1.0f - y , 1.0f -x);
        return sample;
    }
}

float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    return (p.x() >= 0 && p.y() >= 0 && (p.x() + p.y() <= 1)) ? 2.0f : 0.0f ;
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    
    float x = sample.x();
    float y = sample.y();

    float theta = acos(1 - 2*x);
    float phi = 2* M_PI * y;

    return Vector3f (sin(theta)*cos(phi) , sin(theta)*sin(phi) , cos(theta));
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {

    float area = 4 * M_PI;
    return 1/area;    
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {

    float x = sample.x();
    float y = sample.y();

    float theta = acos(1 - 2*x);
    float phi = 2* M_PI * y;

    float new_x = sin(theta)*cos(phi);
    float new_y = sin(theta)*sin(phi);
    float z = cos(theta);

    if (z >= 0)
    {
        return Vector3f (new_x , new_y , z);
    }

    else
    {
        return Vector3f(new_x , new_y , -z);
    }
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if(v.z() < 0.0f)
    {
        return 0.0f;
    }

    else
    {return 1 / (2.0f * M_PI);}

}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {

    float x = sample.x();
    float y = sample.y();

    float phi = 2 * M_PI * x;
    float theta = asin(sqrt(y)); //Why not asin(M_PI * y) ? 

    float new_x = sin(theta)*cos(phi);
    float new_y = sin(theta)*sin(phi);
    float z = cos(theta);

    if (z >= 0)
    {
        return Vector3f (new_x , new_y , z);
    }

    else
    {
        return Vector3f(new_x , new_y , -z);
    }

}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {

    if (v.z() < 0.0f)
    {
        return 0.0f;
    }
    else
    {
        return v.z() / M_PI;
    }
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float x = sample.x();
    float y = sample.y();

    float phi = 2 * M_PI * x;
    float theta = atan(sqrt(-alpha * alpha *log(1 - y))) ;

    float new_x = sin(theta)*cos(phi);
    float new_y = sin(theta)*sin(phi);
    float z = cos(theta);
    
    return Vector3f(new_x,new_y,z);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() <= 0)
            return 0;
    
    float cosTheta = m.z();
    float tanTheta2 = (m.x() * m.x() + m.y() * m.y()) / (cosTheta * cosTheta);
    float cosTheta4 = cosTheta * cosTheta * cosTheta * cosTheta;
    float D = exp(-tanTheta2 / pow(alpha,2)) / (M_PI * pow(alpha,2) * cosTheta4);
    return D * cosTheta;
    
}

NORI_NAMESPACE_END
