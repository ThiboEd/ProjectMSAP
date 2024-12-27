#include <nori/texture.h>
#include <math.h>

NORI_NAMESPACE_BEGIN

class PerlinNoise {
private:
    int p[512];
    int permutation[256] = { 151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
        190,6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
        74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,65,25,63,
        161,1,216,80,73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,250,124,
        123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,119,248,152,2,44,154,163,70,
        221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,97,228,251,34,242,193,238,
        210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,
        150,254,138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180 };

public:
    PerlinNoise() {
        for (int i = 0; i < 256; i++) {
            p[256 + i] = p[i] = permutation[i];
        }
    }

    double fade(double t) const {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    double lerp(double t, double a, double b) const {
        return a + t * (b - a);
    }

    double grad(int hash, double x, double y, double z) const {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }

    double noise(double x, double y, double z) const {
        int X = (int)floor(x) & 255;
        int Y = (int)floor(y) & 255;
        int Z = (int)floor(z) & 255;

        x -= floor(x);
        y -= floor(y);
        z -= floor(z);

        double u = fade(x);
        double v = fade(y);
        double w = fade(z);

        int A = p[X] + Y;
        int AA = p[A] + Z;
        int AB = p[A + 1] + Z;
        int B = p[X + 1] + Y;
        int BA = p[B] + Z;
        int BB = p[B + 1] + Z;

        return lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z),
            grad(p[BA], x - 1, y, z)),
            lerp(u, grad(p[AB], x, y - 1, z),
                grad(p[BB], x - 1, y - 1, z))),
            lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1),
                grad(p[BA + 1], x - 1, y, z - 1)),
                lerp(u, grad(p[AB + 1], x, y - 1, z - 1),
                    grad(p[BB + 1], x - 1, y - 1, z - 1))));
    }
};

class WoodTexture : public Texture {
public:
    WoodTexture(const PropertyList& props) {
        m_baseColor = props.getColor("baseColor", Color3f(0.6f, 0.3f, 0.1f));  // Brown
        m_grainColor = props.getColor("grainColor", Color3f(0.4f, 0.2f, 0.07f));  // Darker brown
        m_scale = props.getFloat("scale", 50.0f);
        m_turbulence = props.getFloat("turbulence", 5.0f);
        m_ringScale = props.getFloat("ringScale", 0.1f);
    }

    virtual Color3f eval(const Point2f& uv) const override {
        // Calculate wood grain pattern
        Point2f scaledUV = uv * m_scale;
        
        // Add some turbulence to make the wood grain more natural
        // double xyValue = scaledUV.x() + 
        //     m_turbulence * m_perlin.noise(scaledUV.x() * 0.1, 
        //                                 scaledUV.y() * 0.1, 
        //                                 0.0);
        double xyValue = scaledUV.x() + 
            m_turbulence * m_perlin.noise(scaledUV.x() * 0.1, scaledUV.y() * 0.1, 0.0) +
            fbm(scaledUV, 3) * 0.2;

        
        // Create ring pattern
        // double ringValue = sin(xyValue * m_ringScale);
        double ringValue = sin(xyValue * m_ringScale + fbm(scaledUV, 5));

        ringValue = (ringValue + 1.0) * 0.5;  // Normalize to [0,1]
        
        // Add fine grain noise
        double grainNoise = m_perlin.noise(scaledUV.x() * 2.0, scaledUV.y() * 2.0, 0.0) * 0.1;
            ringValue = std::max(0.0, std::min(1.0, ringValue + grainNoise));

                                         
        ringValue = std::max(0.0, std::min(1.0, ringValue + grainNoise));

        // Interpolate between base color and grain color
        return m_baseColor.cwiseProduct(Color3f(ringValue)) + 
               m_grainColor.cwiseProduct(Color3f(1.0f - ringValue));
    }

    virtual std::string toString() const override {
        return tfm::format(
            "WoodTexture[\n"
            "  baseColor = %s,\n"
            "  grainColor = %s,\n"
            "  scale = %f,\n"
            "  turbulence = %f,\n"
            "  ringScale = %f\n"
            "]",
            m_baseColor.toString(),
            m_grainColor.toString(),
            m_scale,
            m_turbulence,
            m_ringScale);
    }

    double fbm(Point2f p, int octaves) const {
    double total = 0.0;
    double amplitude = 1.0;
    double frequency = 1.0;
    for (int i = 0; i < octaves; ++i) {
        total += amplitude * m_perlin.noise(p.x() * frequency, p.y() * frequency, 0.0);
        amplitude *= 0.5;  // Diminuer l'impact des octaves supérieures
        frequency *= 2.0;  // Doubler la fréquence à chaque octave
    }
    return total;
}


protected:
    Color3f m_baseColor;    // Base wood color
    Color3f m_grainColor;   // Wood grain color
    float m_scale;          // Overall scaling of the pattern
    float m_turbulence;     // Amount of turbulence in the grain
    float m_ringScale;      // Scale of the wood rings
    PerlinNoise m_perlin;   // Perlin noise generator
};

NORI_REGISTER_CLASS(WoodTexture, "wood")

NORI_NAMESPACE_END