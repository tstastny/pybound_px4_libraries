#include <pybind11/pybind11.h>
#include <filter/second_order_reference_model.hpp>

namespace py = pybind11;

PYBIND11_MODULE(pybindings, m)
{
    py::class_<SecondOrderReferenceModel<float>>(m, "SecondOrderReferenceModel")
        .def(py::init<>())
        .def(py::init<const float &, const float &>())
        .def("setParameters", &SecondOrderReferenceModel<float>::setParameters)
        .def("getState", &SecondOrderReferenceModel<float>::getState)
        .def("getRate", &SecondOrderReferenceModel<float>::getRate)
        .def("getAccel", &SecondOrderReferenceModel<float>::getAccel)
        .def("update", &SecondOrderReferenceModel<float>::update, py::arg("time_step"), py::arg("state_sample"), py::arg("rate_sample") = 0)
        .def("reset", &SecondOrderReferenceModel<float>::reset);
}