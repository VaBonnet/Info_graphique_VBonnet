#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <sys/time.h>

#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <list>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <random>


std::default_random_engine engine;
std::uniform_real_distribution<double> distrib(0,1);

// Définition structure timeval pour affichage du temps
typedef struct timeval tval;

/**
 * Calculates the elapsed time between two time intervals (in milliseconds).
 */
double get_elapsed(tval t0, tval t1)
{
    return (double)(t1.tv_sec - t0.tv_sec) * 1000.0L + (double)(t1.tv_usec - t0.tv_usec) / 1000.0L;
}

class Vector {
	private:
		double coords[3];

	public:
        explicit Vector(double x=0, double y=0, double z=0) {
            coords[0] = x;
            coords[1] = y;
            coords[2] = z;
        };
        double operator[](int i) const {
            return coords[i];
        };
        
        double &operator[](int i) {
            return coords[i];
        };

        Vector& operator+=(const Vector& a) {
            coords[0] += a[0];
            coords[1] += a[1];
            coords[2] += a[2];
            return *this;
        };

        double Norme() const {
            return coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2];
        };

        Vector get_normalized() {
            double n = sqrt(Norme());
            return Vector(coords[0]/n, coords[1]/n, coords[2]/n);
        };

};

Vector operator+(const Vector& a, const Vector& b){
    return Vector(a[0]+b[0],a[1]+b[1],a[2]+b[2]);
}
Vector operator-(const Vector& a, const Vector& b){
    return Vector(a[0]-b[0],a[1]-b[1],a[2]-b[2]);
}
Vector operator-(const Vector& b){
    return Vector(-b[0],-b[1],-b[2]);
}
Vector operator*(const Vector& a, double b){
    return Vector(a[0]*b,a[1]*b,a[2]*b);
}
Vector operator*(double b, const Vector& a){
    return Vector(a[0]*b,a[1]*b,a[2]*b);
}
Vector operator*(const Vector& a, const Vector& b){
    return Vector(a[0]*b[0],a[1]*b[1],a[2]*b[2]);
}
Vector operator/(const Vector& a, double b){
    return Vector(a[0]/b,a[1]/b,a[2]/b);
}
double dot(const Vector& a, const Vector& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
double sqr(double a) {
    return a*a;
}
Vector cross_product(const Vector& a, const Vector& b) {
    return Vector(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
}

Vector random_cos(const Vector& N) {
    double u1 = distrib(engine);
    double u2 = distrib(engine);
    double x = cos(2 * M_PI * u1) * sqrt(1 - u2);
    double y = sin(2 * M_PI * u1) * sqrt(1 - u2);
    double z = sqrt(u2);
    Vector T1;
    // On cherche la composante la plus faible, la met à 0, inverse les deux autres en met une des deux négatif
    if(N[0]<N[1] && N[0]<N[2]) {
        T1 = Vector(0,-N[2],N[1]);
    } else {
        if(N[1]<N[2] && N[1]<N[0]) {
            T1 = Vector(-N[2],0,N[0]);
        } else {
            T1 = Vector(-N[1],N[0],0);
        }
    }
    T1 = T1.get_normalized();
    Vector T2 = cross_product(N,T1);
    return z*N + x * T1 + y * T2;
}

class Ray {
    public:
        Vector C;
        Vector u;

        Ray(const Vector& C, const Vector& u) : C(C), u(u) {};

};

// Classe bounding box pour accélérer le calcul global
class Bounding_box {
public:
    Vector coord_min, coord_max;
    
    bool intersect(const Ray& r) {
        // Intersections sur x
        double inter1_x = (coord_min[0] - r.C[0]) / r.u[0];
        double inter2_x = (coord_max[0] - r.C[0]) / r.u[0];
        double interx_min = std::min(inter1_x,inter2_x);
        double interx_max = std::max(inter1_x,inter2_x);

        // Intersections sur y
        double inter1_y = (coord_min[1] - r.C[1]) / r.u[1];
        double inter2_y = (coord_max[1] - r.C[1]) / r.u[1];
        double intery_min = std::min(inter1_y,inter2_y);
        double intery_max = std::max(inter1_y,inter2_y);

        // Intersections sur z
        double inter1_z = (coord_min[2] - r.C[2]) / r.u[2];
        double inter2_z = (coord_max[2] - r.C[2]) / r.u[2];
        double interz_min = std::min(inter1_z,inter2_z);
        double interz_max = std::max(inter1_z,inter2_z);

        // Minimum des maximums
        double inter_max = std::min(std::min(interx_max,intery_max),interz_max); // point où sort boîte (si existe)
        // Maximum des minimums
        double inter_min = std::max(std::max(interx_min,intery_min),interz_min); // point où on rentre dans la boîte (si existe)

        if (inter_max <0) return false;
        return inter_max > inter_min;
    }
};

// Classe utile pour la construction de bvh
class Node { 
public:
    Node *fg, *fd;
    Bounding_box bb;
    int debut,fin; 
};


// Classe abstraite object
class Object {
    public :
        Object(){};
        // Méthode virtuelle : toutes les classes qui héritent de cette classe doivent avoir cette fonction
        virtual bool intersect(const Ray& r, Vector& P, Vector& v_normal, double& t, Vector& color) = 0;

        Vector albedo;
        bool isMirror, isTransparent;
};

// Classes obj reader
class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
    };
    int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
    int uvi, uvj, uvk;  // indices within the uv coordinates array
    int ni, nj, nk;  // indices within the normals array
    int group;       // face group
};
 

class TriangleMesh : public Object {
public:
  ~TriangleMesh() {}
    TriangleMesh(const Vector& albedo, bool isMirror = false, bool isTransparent = false) {
        this->albedo = albedo;
		this->isMirror = isMirror;
		this->isTransparent = isTransparent;
    };


    void build_bvh(int debut, int fin, Node* n) {
        n->debut = debut;
        n->fin = fin;
        n->bb = build_bouding(n->debut, n->fin);
        Vector diag = n->bb.coord_max - n->bb.coord_min;
        int dim;
        if(diag[0] >= diag[1] && diag[0] >= diag[2]) {
            dim = 0;
        } else {
            if (diag[1] >= diag[0] && diag[1] >= diag[2]) {
                dim = 1;
            } else {
                dim = 2;
            }
        }
        double milieu = (n->bb.coord_min[dim] + n->bb.coord_max[dim]) *0.5;
        int pivot = n->debut;
        for (int i = n->debut; i < n->fin; i++) {
            double middle = (vertices[indices[i].vtxi][dim] + vertices[indices[i].vtxj][dim] + vertices[indices[i].vtxk][dim]) /3.; // barycentre
            if (middle < milieu) {
                std::swap(indices[i],indices[pivot]);
                pivot++;
            }
        }

        n->fg = NULL;
        n->fd = NULL;

        if (pivot == n->debut || pivot == n->fin || n->fin - n->debut < 5) return;
        
        n->fg = new Node;
        n->fd = new Node;

        build_bvh(n->debut, pivot, n->fg);
        build_bvh(pivot, n->fin, n->fd);
        
        
    };

    Bounding_box build_bouding (int debut, int fin) { // debut, fin : indices triangles, pas sommets
        Bounding_box bb;
        double infinite = 1E10;
        bb.coord_min = Vector(infinite,infinite,infinite);
        bb.coord_max = Vector(-infinite,-infinite,-infinite);
        for (int i = debut; i < fin; i++) {
            for (int j = 0; j < 3; j++) {
                // Composante sur x
                bb.coord_min[j] = std::min(bb.coord_min[j],vertices[indices[i].vtxi][j]);
                bb.coord_max[j] = std::max(bb.coord_max[j],vertices[indices[i].vtxi][j]);
                
                // Composante sur y
                bb.coord_min[j] = std::min(bb.coord_min[j],vertices[indices[i].vtxj][j]);
                bb.coord_max[j] = std::max(bb.coord_max[j],vertices[indices[i].vtxj][j]);
                
                // Composante sur z
                bb.coord_min[j] = std::min(bb.coord_min[j],vertices[indices[i].vtxk][j]);
                bb.coord_max[j] = std::max(bb.coord_max[j],vertices[indices[i].vtxk][j]);
            }
        }
        return bb;
    };

    bool intersect(const Ray& r, Vector& P, Vector& v_normal, double& t, Vector& color){
        t = 1E10;
        bool has_inter = false;
        if (!bvh->bb.intersect(r)) return false; // Cas rayon n'intersecte pas plus grand boîte

        std::list<Node*> l;
        l.push_back(bvh);
        while (!l.empty()) {
            Node*c = l.front();
            l.pop_front();

            if(c->fg){ // test seulement sur côté gauche, car si pas de fg, pas de fd
                if(c->fg->bb.intersect(r)) {
                    l.push_front(c->fg);

                }
                if(c->fd->bb.intersect(r)) {
                    l.push_front(c->fd);
                    
                }
            } else {
                for (int i = c->debut; i < c->fin; i++) {
                    // Sommets du triangle
                    const Vector &A = vertices[indices[i].vtxi];
                    const Vector &B = vertices[indices[i].vtxj];
                    const Vector &C = vertices[indices[i].vtxk];

                    // Test intersection triangle
                    Vector e1 = B - A;
                    Vector e2 = C - A;
                    Vector N = cross_product(e1, e2);
                    Vector AO = r.C - A;
                    Vector AO_u = cross_product(AO,r.u);
                    double invUN = 1./dot(r.u,N);
                    double beta = -dot(e2,AO_u) * invUN;
                    double gamma = dot(e1,AO_u) * invUN;
                    double alpha = 1 - beta - gamma;
                    double localt = -dot(AO, N) * invUN;

                    if(beta >= 0 && gamma >= 0 && alpha >=0 && beta <= 1 && gamma <= 1 && localt > 0) {
                        has_inter = true;
                        if (localt < t) {
                            t = localt;
                            // Vecteur normal au centre triangle
                            v_normal = (alpha * normals[indices[i].ni] + beta * normals[indices[i].nj] + gamma * normals[indices[i].nk]).get_normalized();
                            P = r.C + r.u * t;

                            // Recuperation texture
                            int W = Wtexture[indices[i].group];
                            int H = Htexture[indices[i].group];
                            Vector UVp = alpha * uvs[indices[i].uvi] + beta * uvs[indices[i].uvj] + gamma * uvs[indices[i].uvk];
                            UVp = UVp * Vector(W,H,0);
                            int uvx = UVp[0] + 0.5;
                            int uvy = UVp[1] + 0.5;
                            uvx = uvx % W;
                            uvy = uvy % H;
                            if (uvx<0) uvx += W;
                            if (uvy<0) uvy += H;

                            // Recale à l'origine de la texture
                            uvy = H - uvy - 1;

                            // Met puissance 2,2 car correction gamma déjà prise compte dans texture
                            color = Vector(
                                std::pow(textures[indices[i].group][(uvy*W+uvx)*3] / 255.,2.2),
                                std::pow(textures[indices[i].group][(uvy*W+uvx)*3 + 1] / 255.,2.2),
                                std::pow(textures[indices[i].group][(uvy*W+uvx)*3 + 2] / 255.,2.2));

                        }
                    }
                }
            }
        }
        return has_inter;
    };
    
    void readOBJ(const char* obj) {
 
        char matfile[255];
        char grp[255];
 
        FILE* f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;
 
            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());
 
            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }
 
            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;
 
                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));
 
                    vertices.push_back(vec);
                    vertexcolors.push_back(col);
 
                } else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;
 
                char* consumedline = line + 1;
                int offset;
 
                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                    indices.push_back(t);
                } else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                        indices.push_back(t);
                    } else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            indices.push_back(t);
                        } else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }
 
                consumedline = consumedline + offset;
 
                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    } else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        } else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;                             
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            } else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                } else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }
 
            }
 
        }
        fclose(f);
 
    }
 
    void loadTexture(const char* texture_file) {
        int W, H, C;
        unsigned char* texture = stbi_load(texture_file, &W, &H, &C, 3);
        Wtexture.push_back(W);
        Htexture.push_back(H);
        textures.push_back(texture);
    }

    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;
    std::vector<unsigned char*> textures;
    std::vector<int> Wtexture, Htexture;
    Bounding_box bb; // Bounding box simple
    Node *bvh = new Node; // Bounding box hierarchy
    
};


class Sphere : public Object {
    public:
        Sphere(const Vector& O, double R, const Vector& albedo, bool isMirror = false, bool isTransparent = false) {
            this->O = O;
            this->R = R;
            this->albedo = albedo;
            this->isMirror = isMirror;
            this->isTransparent = isTransparent;
        };
        bool intersect(const Ray& r, Vector& P, Vector& N, double& t, Vector &color){
            // Résout a * t² + b*t + c = 0
            double a = 1 ;
            double b = 2 * dot(r.u, r.C - O);
            double c = (r.C-O).Norme() - R*R;
            double D = b*b - 4 * a * c;
            if (D<0) {
                double t = 1E10;
                return false;
            }

            double s_D = sqrt(D);
            double t2 = (-b+s_D)/(2*a);
            if (t2<0) return false;
            
            double t1 = (-b-s_D)/(2*a);
            if (t1>0) {
                t = t1;
            } else {
                t = t2;
            }

            P = r.C + r.u*t;
            N = (P - O).get_normalized();

            color = this->albedo;
            return true;
        };

        Vector O;
		double R;
};

class Scene {
    public:
        std::vector<Object*> objects;
        Vector L;
        double I = 1E10;

        Scene(){};
        bool intersect(const Ray& r, Vector& P, Vector& N, Vector& albedo, bool& mirror, bool& transparent, double& t, int& objectid){
            t=1E10;
            bool has_inter = false;
            for (int i = 0; i<objects.size(); i++) {
                Vector temp_P, temp_N, temp_albedo;
                double temp_t;
                bool inter = objects[i]->intersect(r, temp_P, temp_N, temp_t, temp_albedo);
                if(inter && temp_t<t){
                    t = temp_t;
                    has_inter = true;
                    albedo = temp_albedo;
                    P = temp_P;
                    N = temp_N;
                    mirror = objects[i]->isMirror;
                    transparent = objects[i]->isTransparent;
                    objectid = i;
                }
            }
            return has_inter;
        };

        Vector getColor(Ray& r, const int& rebond, bool lastDiffuse) {
            if (rebond > 5) { // Condition d'arrêt
                return Vector(0.,0.,0.);
            }
            // Centre et rayon de la sphère lumière
            double R = dynamic_cast<Sphere*>(objects[0])->R;
            Vector O = dynamic_cast<Sphere*>(objects[0])->O;
            double t;
            Vector P, N, albedo;
            bool mirror = false;
            bool transparent = false;
            int objectid;
            bool inter = intersect(r, P, N, albedo, mirror, transparent, t, objectid);
            Vector color(0,0,0);
            if (inter){
                if (objectid==0) { // id = 0 => source de lumière
                    if (rebond == 0 || !lastDiffuse) { // Cas où rayon arrive directement sur lumière, ou vient d'une surface miroire/transparente
                        return Vector(1.,1.,1.)*I/(4 * M_PI * M_PI * R * R);
                    } else {
                        return Vector(0.,0.,0.);
                    }
                }
                else {
                    if(mirror) { // Rayon réfléchi
                        Vector reflectedDir = r.u - 2 * dot(r.u,N)*N;
                        Ray reflectedRay(P + 0.001 * N, reflectedDir);
                        return getColor(reflectedRay, rebond + 1, false);
                    } 
                    else { 
                        if (transparent) { // Boule transparente
                            // Calcul rayon refracté
                            double n1 = 1, n2 = 1.4;
                            Vector N2 = N;
                            if (dot(r.u,N)>0) { // cas où on sort de la sphère : inverse normale et deux indices
                                std::swap(n1,n2);
                                N2 = -N;
                            }
                            Vector Tt = n1/n2 * (r.u - dot(r.u,N2) * N2);
                            double norm = 1 - (sqr(n1/n2)) * (1 - sqr(dot(r.u,N2)));
                            if (norm < 0) {
                                Vector reflectedDir = r.u - 2 * dot(r.u,N)*N;
                                Ray reflectedRay(P + 0.001 * N, reflectedDir);
                                return getColor(reflectedRay, rebond + 1, false);                        
                            }
                            Vector Tn = -sqrt(norm) * N2;
                            Vector transmittedDir = Tt + Tn;
                            Ray transmittedRay(P - 0.001 * N2,transmittedDir);
                            return getColor(transmittedRay, rebond + 1, false);
                        }
                        else {  // Pas transparence ni miroir  

                            //eclairage direct
                            Vector PL = O - P;
                            PL = PL.get_normalized();
                            Vector wi = random_cos(-PL).get_normalized();
                            Vector xp = wi * R + O;
                            Vector P_xp = xp - P;
                            double d = sqrt(P_xp.Norme());
                            P_xp = P_xp / d;

                            // Gestion des ombres
                            Vector shadowP, shadowN, shadowAlbedo;
                            double shadowt = 1E10;
                            bool shadow_mirror, shadow_transp;
                            int shadow_id;
                            Ray shadowRay(P+0.0001*N, P_xp);
                            bool shadowInter = intersect(shadowRay,shadowP, shadowN, shadowAlbedo, shadow_mirror, shadow_transp, shadowt, shadow_id);
                            if(shadowInter && shadowt < d-0.001) { // Cas obstacle entre P et la lumière
                                color = Vector(0.,0.,0.);
                            } else { // Contribution directe de lumière
                                double r_carre = sqr(R);
                                double prob = std::max(0.,dot(-PL, wi))/(M_PI * r_carre);
                                double Jacobien = std::max(0.,dot(wi,-P_xp))/(d*d);
                                color = I/(4*M_PI*M_PI*r_carre) * albedo/M_PI * std::max(0., dot(N, P_xp)) * Jacobien / prob;
                            }
                            // eclairage indirect
                            Vector reflectedDir = random_cos(N);
                            Ray reflectedRay(P + 0.001 * reflectedDir, reflectedDir.get_normalized());
                            color += albedo * getColor(reflectedRay, rebond + 1, true);
                        }
                    }
                }
            }
        return color;
        };


};


int main() {
    // Variable stockant les temps de début et de fin pour calcul de durée
    tval t[2] = { 0 };
    double elapsed = 0;
    // Prise du temps de début
    gettimeofday(&t[0], NULL);

    // Taille de l'image
	int W = 256*2;
	int H = 256*2;

    // Création des objets
    Vector C(0,0,55);
    Sphere principal(Vector(0,0,0),10,Vector(1.,0.,0.), false, false);
    Sphere sol(Vector(0,-1000,0),975,Vector(1.,1.,1.));
    Sphere plafond(Vector(0,1050,0),975,Vector(0.5,0.5,0.5));
    Sphere mur_gauche(Vector(-1000,0,0),950,Vector(0.5,0.5,0.5));
    Sphere mur_droite(Vector(1000,0,0),950,Vector(0.5,0.5,0.5));
    Sphere fond(Vector(0,0,-1050),950,Vector(0.,1.,1.));
    Sphere arriere(Vector(0,0,1050),950,Vector(0.,0.,1.));
    Sphere lumiere(Vector(-10,20,40),5,Vector(1.,1.,1.));
    TriangleMesh maillage(Vector(1.,1.,1.),false,false);
    // Lecture du maillage
    maillage.readOBJ("dog/13463_Australian_Cattle_Dog_v3.obj");
    maillage.loadTexture("dog/Australian_Cattle_Dog_dif.jpg");
    
    // Réarrangement du maillage à la scene
    for (int i=0; i < maillage.vertices.size();i++) {
        maillage.vertices[i][1] += +10;
        std::swap(maillage.vertices[i][1],maillage.vertices[i][2]);
        std::swap(maillage.vertices[i][2],maillage.vertices[i][0]);
        maillage.vertices[i][1] += -15;
        maillage.vertices[i][0] += -10;
        maillage.vertices[i][2] = -maillage.vertices[i][2];
        maillage.vertices[i][0] = -maillage.vertices[i][0];
    }
    for (int i=0; i < maillage.normals.size();i++) {
        std::swap(maillage.normals[i][1],maillage.normals[i][2]);
        std::swap(maillage.normals[i][2],maillage.normals[i][0]);
        maillage.normals[i] = -maillage.normals[i];
    }

    // Création bvh
    maillage.build_bvh(0, maillage.indices.size(),maillage.bvh); // Construction bounding box

    // Création scene et insertion objets
    Scene scene;
    scene.objects.push_back(&lumiere);
    //scene.objects.push_back(&principal);
    scene.objects.push_back(&sol);
    scene.objects.push_back(&plafond);
    scene.objects.push_back(&mur_gauche);
    scene.objects.push_back(&mur_droite);
    scene.objects.push_back(&fond);
    scene.objects.push_back(&arriere);
    scene.objects.push_back(&maillage);

    double fov = 60 * M_PI / 180;
    int nbrays = 20;
/*
    Vector up(0,cos(-10 * M_PI / 180),sin(-10 * M_PI / 180));
    Vector right(0, 1, 0);
    Vector viewDirection = cross_product(-right,up);
*/

    double taille_objectif = 0.0001; // Valeur faible = effet désactivé
    double focale = 55;
	
	std::vector<unsigned char> image(W*H * 3, 0);
    #pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
            //Vector u(j-W/2, i-H/2, -W/(2.*tan(fov / 2))); // Laisser commenté dans cas veut enlver anti-aliasing
            Vector color = Vector(0,0,0);
            
            for (int k = 0; k<nbrays; k++) {
                // Effet flou optique
                double u1 = distrib(engine);
                double u2 = distrib(engine);
                double x_objectif = taille_objectif * cos(2 * M_PI * u1) * sqrt(1 - u2);
                double y_objectif = taille_objectif * sin(2 * M_PI * u1) * sqrt(1 - u2);
				while( (sqr(x_objectif) + sqr(y_objectif)) > taille_objectif){ // Boucle pour retirer des variables aléatoires si le point est hors du cercle de l'objectif
					u1 = distrib(engine);
					u2 = distrib(engine);
					x_objectif = u1 * 2 * taille_objectif - taille_objectif; 
					y_objectif = u1* 2 * taille_objectif - taille_objectif;
				}

                // anti-aliasing
				double u3 = distrib(engine);
				double u4 = distrib(engine);
				double x = sqrt(1 - u3) * cos(2 * M_PI * u4);
				double y = sqrt(1 - u3) * sin(2 * M_PI * u4);

				Vector u(j - W / 2 + x, i - H / 2 + y, -W / (2 * tan(fov / 2)));
				u = u.get_normalized();

                // Modification de u pour mouvement caméra
                //u = u[0] * right + u[1] * up + u[2] * viewDirection;


                Vector target = C + focale*u;
                Vector new_C = C + Vector(x_objectif,y_objectif,0);
                Vector new_u = (target - new_C).get_normalized();
                Ray rayon(new_C,new_u);
                // Calcul de la couleur
                color += scene.getColor(rayon,0,false);
            }
            color = color / nbrays; // Moyenne de couleur obtenue pour tous les rayons lancés

            // Ecriture du pixel 
            image[((H-i-1)*W + j) * 3 + 0] = std::min(255.,std::pow(color[0],0.45)); // puissance 0,45 : correction gamma
            image[((H-i-1)*W + j) * 3 + 1] = std::min(255.,std::pow(color[1],0.45));
            image[((H-i-1)*W + j) * 3 + 2] = std::min(255.,std::pow(color[2],0.45));
		}
	}
	stbi_write_png("images/no_bb.png", W, H, 3, &image[0], 0);

    
    // Prise du temps de fin
    gettimeofday(&t[1], NULL);
    elapsed = get_elapsed(t[0], t[1]) / 1000;
    std::cout << "Done ! Temps pour générer l'image : " << elapsed << " sec" << std::endl;


	return 0;
}