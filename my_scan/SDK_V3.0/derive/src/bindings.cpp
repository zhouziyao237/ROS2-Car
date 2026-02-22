#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "base/hchead.h"
#include "base/HcSDK.h"
#include "base/HcData.h"
#include "base/lidar.h"
#include "lidar/LidarTest.h"

namespace py = pybind11;

PYBIND11_MODULE(sdk_py, m) {
    m.doc() = "Python bindings for the entire derive library";

    // --- Base Module: Dev Class ---
    py::class_<Dev>(m, "Dev")
        .def(py::init<>())
        .def("Initialize", &Dev::Initialize, "Initialize the device",
             py::arg("port"), py::arg("baudrate"), py::arg("isGetLoopData") = true)
        .def("GetScanData", [](Dev &dev, bool isReverse) {
            std::list<node_info> dataList;
            dev.GetScanData(dataList, isReverse);
            return dataList;
        }, "Get scan data", py::arg("isReverse") = true)
        .def("Uninit", &Dev::Uninit, "Uninitialize the device")
        .def("GetLastErrCode", &Dev::GetLastErrCode, "Get last error code");

    // --- PointCloud Structure ---
    py::class_<tsPointCloud>(m, "PointCloud")
        .def(py::init<>())
        .def_readwrite("valid", &tsPointCloud::bValid)
        .def_readwrite("angle", &tsPointCloud::dAngle)
        .def_readwrite("angle_raw", &tsPointCloud::dAngleRaw)
        .def_readwrite("distance", &tsPointCloud::u16Dist)
        .def_readwrite("distance_raw", &tsPointCloud::u16DistRaw)
        .def_readwrite("speed", &tsPointCloud::u16Speed)
        .def_readwrite("gray", &tsPointCloud::u16Gray)
        .def_readwrite("timestamp_ns", &tsPointCloud::u64TimeStampNs)
        .def("__repr__", [](const tsPointCloud &data) -> std::string {
            return "<PointCloud angle=" + std::to_string(data.dAngle) +
                   ", distance=" + std::to_string(data.u16Dist) + 
                   ", valid=" + (data.bValid ? "True" : "False") + ">";
        });

    // --- Lidar Module ---
    py::class_<LidarTest>(m, "LidarTest")
        .def(py::init<>())
        .def("initLidar", &LidarTest::initLidar, "Initialize lidar");

    // --- Node Info Structure ---
    py::class_<node_info>(m, "NodeInfo")
        .def(py::init<>())
        .def_readwrite("syn_quality", &node_info::syn_quality)
        .def_readwrite("angle_q6_checkbit", &node_info::angle_q6_checkbit)
        .def_readwrite("distance_q2", &node_info::distance_q2)
        .def_readwrite("is_valid", &node_info::isValid)
        .def_readwrite("speed", &node_info::speed)
        .def("__repr__", [](const node_info &info) -> std::string {
            return "<NodeInfo angle=" + std::to_string(info.angle_q6_checkbit/64.0) +
                   ", distance=" + std::to_string(info.distance_q2) + 
                   ", valid=" + (info.isValid ? "True" : "False") + ">";
        });

    // --- High-Level Functions ---
    m.def("hc_sdk_initialize", [](const std::string &port, const std::string &model, int baudrate, 
                                  int timeout, bool dist_q2, bool loop_data, bool poll_mode) {
        return hcSDKInitialize(port.c_str(), model.c_str(), baudrate, timeout, dist_q2, loop_data, poll_mode);
    }, "Initialize HC SDK",
    py::arg("port"), py::arg("model") = "X1B", py::arg("baudrate") = 115200, 
    py::arg("timeout") = 1000, py::arg("dist_q2") = false, 
    py::arg("loop_data") = false, py::arg("poll_mode") = false);

    m.def("hc_sdk_uninit", []() {
        return hcSDKUnInit();
    }, "Uninitialize HC SDK");

    m.def("get_last_error", []() {
        return getSDKLastErrCode();
    }, "Get last error code");

    m.def("get_lidar_info", []() {
        return getSDKLidarInfo();
    }, "Get lidar information");

    m.def("get_status", []() {
        return getSDKStatus();
    }, "Get SDK status");

    m.def("get_version", []() {
        return getSDKVersion();
    }, "Get SDK version");

    m.def("get_lidar_id", []() {
        return getSDKLidarID();
    }, "Get lidar ID");

    m.def("get_factory_info", []() {
        return getSDKFactoryInfo();
    }, "Get factory information");

    m.def("get_lidar_model", []() {
        return getSDKLidarModel();
    }, "Get lidar model");

    m.def("get_firmware_version", []() {
        return getSDKFirmwareVersion();
    }, "Get firmware version");

    m.def("get_hardware_version", []() {
        return getSDKHardwareVersion();
    }, "Get hardware version");
}