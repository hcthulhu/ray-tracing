#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <fstream>
#include "geometry.h"

const float EPSILON = 0.0000001;
const float FLARE_COEFF = 0.5, FLARE_POW = 100;
const float LIGHT_BASE = 100, LIGHT_COEFF = 0.5;
const unsigned int DEPTH = 4;
const float TRANSPARENT_COEFF = 0.2;
const float SPECULAR_COEFF = 0.4;

typedef struct PointLight {
    Vec3f point;
    Vec3f color;

    PointLight(const Vec3f &coords, const Vec3f &col) {
        point = coords;
        color = col;
    }
} PointLight;

typedef struct Triangle {
	Vec3f v0;
	Vec3f v1;
	Vec3f v2;
	Vec3f normal;

	Triangle(const Vec3f &p0, const Vec3f &p1, const Vec3f &p2) {
		v0 = p0;
		v1 = p1;
		v2 = p2;
		normal = cross(v1 - v0, v2 - v0).normalize();
	};

	Triangle() {
	    v0 = Vec3f (0, 0, 0);
	    v1 = Vec3f (0, 0, 0);
	    v2 = Vec3f (0, 0, 0);
	    normal = Vec3f (0, 0, 0);
	}
} Triangle;

typedef struct Intersection {
    bool isIntersected;
    char side;
    Vec3f point;
    Triangle triangle;

    Intersection(const bool &isInt, const char &s, const Vec3f &p, const Triangle tr) {
        isIntersected = isInt;
        side = s;
        point = p;
        triangle = tr;
    };

    Intersection() {
        isIntersected = false;
        side = 0;
        point = Vec3f (0, 0, 0);
        triangle = Triangle();
    }
} Intersection;

void readTrianglesFromFile(std::string fileName, std::vector<Triangle> &trs, int amount);
void saveImage(char* filename, int w, int h, unsigned char* data);
Intersection rayTriangleIntersect(const Vec3f &orig, const Vec3f &dir, const Triangle &tr);
Vec3f castRayColor(const Vec3f &origin, const Vec3f &dir, const std::vector<PointLight> &lightSources, const std::vector<Triangle> &trs, int stepOfRecursion);

void readTrianglesFromFile(std::string fileName, std::vector<Triangle> &trs, int amount=1) {
	std::ifstream inf(fileName);
	if (!inf)
	{
		std::cout << "Can't read!" << std::endl;
		exit(1);
	}

	Vec3f p0, p1, p2;

	while (inf && amount--)
	{
		inf >> p0.x >> p0.y >> p0.z;
		inf >> p1.x >> p1.y >> p1.z;
		inf >> p2.x >> p2.y >> p2.z;
		trs.push_back(Triangle(p0, p1, p2));
	}
}

Vec3f castRayColor(const Vec3f &origin, const Vec3f &dir, const std::vector<PointLight> &lightSources, const std::vector<Triangle> &trs, int stepOfRecursion) {
    Vec3f color(0, 0, 0);
    if (stepOfRecursion >= DEPTH)
        return Vec3f(0, 0, 0);

    stepOfRecursion++;

    std::vector<Intersection> intersections;
    Intersection inter;

    for (int i = 0; i < trs.size(); i++) {
        inter = rayTriangleIntersect(origin, dir, trs[i]);
        if (inter.isIntersected) {
            intersections.push_back(inter);
        }
    }
    if (!intersections.empty()) {
        Intersection nearest = intersections[0];
        float len_nearest = (nearest.point - origin).norm();
        float len_tmp;
        for (auto x : intersections) {
            len_tmp = (x.point - origin).norm();
            if (len_tmp < len_nearest) {
                nearest = x;
                len_nearest = (nearest.point - origin).norm();
            }
        }

        for (auto lightSource : lightSources) {
            float lum;
            float light_signed = nearest.triangle.normal * (nearest.point - lightSource.point).normalize();
            float origin_signed = nearest.triangle.normal * (nearest.point - origin).normalize();
            if (light_signed * origin_signed <= 0) {
                lum = 0;
            } else {
                lum = fabs(light_signed);
            }

            Vec3f flare_color(powf(lum * lightSource.color[0], FLARE_POW),
                              powf(lum * lightSource.color[1], FLARE_POW),
                              powf(lum * lightSource.color[2], FLARE_POW));
            Vec3f light_color(lum * lightSource.color[0],
                              lum * lightSource.color[1],
                              lum * lightSource.color[2]);

            switch (nearest.side) {
                case 1: { // transparent
                    Vec3f newOrigin = nearest.point + dir * 0.001;
                    Vec3f trans_color = castRayColor(newOrigin, dir, lightSources, trs, stepOfRecursion);
                    color = color + flare_color * FLARE_COEFF + light_color * LIGHT_COEFF
                            + trans_color * TRANSPARENT_COEFF;
                    break;
                }
                case -1: { // specular and transparent
                    Vec3f newOriginSpec = nearest.point - dir * 0.001;
                    Vec3f newOriginTrans = nearest.point + dir * 0.001;
                    Vec3f newDir = dir - nearest.triangle.normal * 2.0 * (nearest.triangle.normal * dir);
                    Vec3f spec_color = castRayColor(newOriginSpec, newDir, lightSources, trs, stepOfRecursion);
                    Vec3f trans_color = castRayColor(newOriginTrans, dir, lightSources, trs, stepOfRecursion);
                    color = color + flare_color * FLARE_COEFF + light_color * LIGHT_COEFF
                            + trans_color * TRANSPARENT_COEFF + spec_color * SPECULAR_COEFF;
                    break;
                }
            }
        }
    }

    return color;
}

void saveImage(char* filename, int w, int h, int channels_num, unsigned char* data) {
	stbi_write_jpg(filename, w, h, channels_num, data, 100);
}

Intersection rayTriangleIntersect(const Vec3f &orig, const Vec3f &dir, const Triangle &tr) {
	float u, v, t;
	Vec3f v0v1 = tr.v1 - tr.v0;
    Vec3f v0v2 = tr.v2 - tr.v0;
    Vec3f pvec = cross(dir, v0v2);
    float det = v0v1 * pvec;
	Intersection ret;

    // if the determinant is close to 0, the ray misses the triangle
	if (fabs(det) < EPSILON)
		return ret;

    float invDet = 1 / det;

    Vec3f tvec = orig - tr.v0;
    u = tvec * pvec * invDet;
    if (u < 0 || u > 1)
    	return ret;

    Vec3f qvec = cross(tvec, v0v1);
    v = dir * qvec * invDet;
    if (v < 0 || u + v > 1)
    	return ret;

    t = v0v2 * qvec * invDet;
    if (t > 0.0) {
		ret.isIntersected = true;
		float s = (dir - orig) * tr.normal;
    	ret.side = s < 0.0 ? -1 : 1;
    	ret.point = orig + dir * t;
    	ret.triangle = tr;
	}

	return ret;
}

std::vector<Vec3f> find_basis (Vec3f dir_0) {
    float x0 = dir_0.x;
    float y0 = dir_0.y;
    float z0 = dir_0.z;

    Vec3f p(0, 0, 0);
    Vec3f q(0, 0, 0);

    if (x0 == 0 && y0 == 0 && z0 == 0) {
        p.x = 1;
        p.y = 1;
        p.z = 1;
    } else if ((x0 != 0 || y0 != 0) && z0 == 0) {
        p.x = 0;
        p.y = 0;
        p.z = 1;
    } else if (x0 == 0 && y0 == 0 && z0 != 0) {
        p.y = -z0;
        p.x = 0;
        p.z = 0;
    } else if (x0 == 0 && y0 != 0 && z0 != 0) {
        p.x = 0;
        p.z = 1;
        p.y = -z0 / y0 * p.z;
    } else if (x0 != 0 && y0 == 0 && z0 != 0) {
        p.y = 0;
        p.z = 1;
        p.x = -z0 / x0 * p.z;
    } else {
        p.z = 1;
        p.x = -z0 * x0 / (x0 * x0 + y0 * y0) * p.z;
        p.y = y0 / x0 * p.x;
    }

    q = cross(dir_0, p);
    p.normalize();
    q.normalize();

    return std::vector<Vec3f>({q, p});
}

#endif // RAY_TRACING_H
