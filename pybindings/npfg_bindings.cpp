#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <lib/npfg/npfg.hpp>

namespace py = pybind11;

// wrapper class
class NPFGWrapper : public NPFG
{
public:
	std::vector<float> wrapped_bearingVec(const std::vector<float> &unit_path_tangent, const float look_ahead_ang,
				    const float signed_track_error) const
    {
        matrix::Vector2f unit_path_tangent_vector2f{unit_path_tangent[0], unit_path_tangent[1]};
        matrix::Vector2f bearing_vec = bearingVec(unit_path_tangent_vector2f, look_ahead_ang, signed_track_error);
        return {bearing_vec(0), bearing_vec(1)};
    }
};

PYBIND11_MODULE(npfg_lib, m)
{
    py::class_<NPFGWrapper>(m, "NPFG")
        .def(py::init<>())
        .def("trackErrorBound", &NPFGWrapper::trackErrorBound)
        .def("pGain", &NPFGWrapper::pGain)
        .def("timeConst", &NPFGWrapper::timeConst)
        .def("lookAheadAngle", &NPFGWrapper::lookAheadAngle)
        .def("normalizedTrackError", &NPFGWrapper::normalizedTrackError)
        .def("bearingVec", &NPFGWrapper::wrapped_bearingVec);
}