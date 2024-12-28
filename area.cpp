/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
	
	v1 - Dec 01 2020
	Copyright (c) 2020 by Adrian Jarabo

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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/mesh.h>
#include <nori/texture.h>
#include <Eigen/Dense>


NORI_NAMESPACE_BEGIN

struct SphQuad {
    Vector3f origin, x, y, z; // Sistema de referencia local
    float x0, y0, z0, z0sq, y0sq;         // Coordenadas locales
    float x1, y1, y1sq;       // Coordenadas del rectángulo
    float b0, b1, b0sq, solidAngle, k; // Constantes precomputadas
};

void SphQuadInit(SphQuad& squad, const Vector3f& s, const Vector3f& ex, const Vector3f& ey, const Vector3f& o) {
    float exl = ex.norm();
    float eyl = ey.norm();

    // Calcular el sistema de referencia local
    squad.x = ex / exl;
    squad.y = ey / eyl;
    squad.z = squad.x.cross(squad.y).normalized();

    // Calcular las coordenadas del rectángulo en el sistema de referencia local
    Vector3f d = s - o;
    squad.z0 = d.dot(squad.z);

    // Asegurar que z apunte hacia afuera del rectángulo
    if (squad.z0 > 0) {
        squad.z *= -1;
        squad.z0 *= -1;
    }

	squad.z0sq = squad.z0 * squad.z0;
    squad.x0 = d.dot(squad.x);
    squad.y0 = d.dot(squad.y);
    squad.x1 = squad.x0 + exl;
    squad.y1 = squad.y0 + eyl;
	squad.y0sq = squad.y0 * squad.y0; 
	squad.y1sq = squad.y1 * squad.y1;

	// create vectors to four vertices
	Vector3f v00 = {squad.x0, squad.y0, squad.z0}; 
	Vector3f v01 = {squad.x0, squad.y1, squad.z0}; 
	Vector3f v10 = {squad.x1, squad.y0, squad.z0}; 
	Vector3f v11 = {squad.x1, squad.y1, squad.z0}; // compute normals to edges
	Vector3f n0 = (v00.cross(v10)).normalized(); 
	Vector3f n1 = (v10.cross(v11)).normalized(); 
	Vector3f n2 = (v11.cross(v01)).normalized(); 
	Vector3f n3 = (v01.cross(v00)).normalized(); // compute internal angles (gamma_i) 
	float g0 = acos(-n0.dot(n1));
	float g1 = acos(-n1.dot(n2)); 
	float g2 = acos(-n2.dot(n3)); 
	float g3 = acos(-n3.dot(n0)); // compute predefined constants squad.b0 = n0.z;
	squad.b1 = n2.z();
	squad.b0sq = squad.b0 * squad.b0;
	squad.k = 2*M_PI - g2 - g3;
	// compute solid angle from internal angles 
	squad.solidAngle = g0 + g1 - squad.k;
}

Vector3f SphQuadSample(const SphQuad& squad, float u, float v) {
    // 1. Calcular `cu`
    float au = u * squad.solidAngle + squad.k; // Ángulo ajustado
    float fu = (cos(au) * squad.b0 - squad.b1) / sin(au);
    float cu = 1 / sqrt(fu * fu + squad.b0sq) * (fu > 0 ? 1 : -1);
    cu = clamp(cu, -1.0f, 1.0f); // Evitar NaN o valores fuera de rango

    // 2. Calcular `xu`
    float xu = -(cu * squad.z0) / sqrt(1 - cu * cu);
    xu = clamp(xu, squad.x0, squad.x1);

    // 3. Calcular `yv`
    float d = sqrt(xu * xu + squad.z0sq);
    float h0 = squad.y0 / sqrt(d * d + squad.y0sq);
    float h1 = squad.y1 / sqrt(d * d + squad.y1sq);
    float hv = h0 + v * (h1 - h0), hv2 = hv * hv;
    float yv = (hv2 < 1.0f - 1e-6) ? (hv * d) / sqrt(1 - hv2) : squad.y1;

    // 4. Convertir a coordenadas globales
    return squad.origin + xu * squad.x + yv * squad.y + squad.z0 * squad.z;
}


class AreaEmitter : public Emitter {
public:
	AreaEmitter(const PropertyList &props) {
		m_type = EmitterType::EMITTER_AREA;
		m_radiance = new ConstantSpectrumTexture(props.getColor("radiance", Color3f(1.f)));
		m_scale = props.getFloat("scale", 1.);
	}

	virtual std::string toString() const {
		return tfm::format(
			"AreaLight[\n"
			"  radiance = %s,\n"
			"  scale = %f,\n"
			"]",
			m_radiance->toString(), m_scale);
	}

	// We don't assume anything about the visibility of points specified in 'ref' and 'p' in the EmitterQueryRecord.
	virtual Color3f eval(const EmitterQueryRecord & lRec) const {
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");

		// This function call can be done by bsdf sampling routines.
		// Hence the ray was already traced for us - i.e a visibility test was already performed.
		// Hence just check if the associated normal in emitter query record and incoming direction are not backfacing
		if (lRec.n.dot(-lRec.wi) < 0.0f) { // check if backfacing, lRec.wi is the direction emitter-receptor (-lRec.wi is receptor emitter)
        	return Color3f(0.0f);
		}
		return m_radiance->eval(lRec.uv);
	}

	virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample, float optional_u) const {
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");
		if (lRec.n.dot(-lRec.wi) < 0.0f) { // check if backfacing, lRec.wi is the direction emitter-receptor (-lRec.wi is receptor emitter)
        	return Color3f(0.0f);
		}
		// sample a point on the mesh and update the value of the records
		//m_mesh->samplePosition(sample, lRec.p, lRec.n, lRec.uv);
		// Usa el sistema local R para generar una muestra
		Vector3f sampledPoint = SphQuadSample(m_sphQuad, sample.x(), sample.y());
		lRec.p = sampledPoint;

		
		// update the values on the record
		lRec.dist = (lRec.p - lRec.ref).norm();
		lRec.wi = (lRec.p - lRec.ref) / lRec.dist;
		lRec.pdf = pdf(lRec);
		if (lRec.pdf < 1e-3) {	// if pdf is too small, assume it is black
			return Color3f(0.0f);
		}
		return m_radiance->eval(lRec.uv)/ pow(lRec.dist, 1);
	}

	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
	// Assumes all information about the intersection point is already provided inside.
	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior. 
	//			Plus no visibility is considered.
	virtual float pdf(const EmitterQueryRecord &lRec) const {
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");
		//float m_pdf = m_mesh->pdf(lRec.p);
		//float dist2 = static_cast<float>(pow(lRec.dist, 2));
		//float cosFactor = lRec.n.dot(-lRec.wi);

		//return m_pdf * dist2 / cosFactor;
		
		//Accede al ángulo sólido precomputado en el sistema local R
		float solidAngle = m_sphQuad.solidAngle;
		if (solidAngle <= 0.0f)
        	return 0.0f; // Caso degenerado: no hay ángulo sólido

		return 1.0f / solidAngle;
	}


	// Get the parent mesh
	void setParent(NoriObject *parent)
	{
		auto type = parent->getClassType();
		if (type == EMesh)
			m_mesh = static_cast<Mesh*>(parent);
			
			// Precomputar el sistema local R
			Vector3f lightCorner = m_mesh->getVertexPositions().col(0);
			Vector3f ex = m_mesh->getVertexPositions().col(1) - lightCorner;
			Vector3f ey = m_mesh->getVertexPositions().col(2) - lightCorner;
			Vector3f observer = Vector3f(0, 0, 0); // Puedes cambiar esto si es necesario

			SphQuadInit(m_sphQuad, lightCorner, ex, ey, observer);
	}

	// Set children
	void addChild(NoriObject* obj, const std::string& name = "none") {
		switch (obj->getClassType()) {
		case ETexture:
			if (name == "radiance")
			{
				delete m_radiance;
				m_radiance = static_cast<Texture*>(obj);
			}
			else
				throw NoriException("AreaEmitter::addChild(<%s>,%s) is not supported!",
					classTypeName(obj->getClassType()), name);
			break;

		default:
			throw NoriException("AreaEmitter::addChild(<%s>) is not supported!",
				classTypeName(obj->getClassType()));
		}
	}
protected:
	Texture* m_radiance;
	float m_scale;
	SphQuad m_sphQuad; // Sistema local para el muestreo basado en ángulo sólido
};

NORI_REGISTER_CLASS(AreaEmitter, "project_area")
NORI_NAMESPACE_END
