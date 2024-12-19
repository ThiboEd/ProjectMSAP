#include <nori/emitter.h>
public :
	m_type = EmitterType::EMITTER_POINT;
	m_radiance = props.getColor ( "radiance" , Color3f ( 1.f ) ) ;
	return tfm::format(
		"  position = %s,\n"
		m_radiance.toString ( ) ) ;
// This function assumes that a ray have been traced towards
// the light source. However, since the probability of randomly
	return 0.;

virtual Color3f sample(EmitterQueryRecord & lRec,
	lRec.dist = (lRec.p - lRec.ref).norm(); 
	lRec.wi = (lRec.p - lRec.ref) / lRec.dist;

	// reasons it is more convenient to just leave as 1
	lRec.pdf = 1.;

	// that visibility should be taken care of in the integrator.
	return m_radiance/(lRec.dist*lRec.dist);
// it is more convenient to just leave as 1
	return 1.;
Point3f m_position; 
Color3f m_radiance;