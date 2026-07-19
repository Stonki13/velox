from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
import re


class VeloxConan(ConanFile):
    name = "velox"
    license = "MIT"
    url = "https://github.com/Stonki13/velox"
    description = "C++17 rigid-body physics engine"
    topics = ("physics", "rigid-body", "game-development", "cuda")
    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False], "with_cuda": [True, False]}
    default_options = {"shared": False, "fPIC": True, "with_cuda": False}
    exports_sources = "CMakeLists.txt", "cmake/*", "include/*", "src/*", "LICENSE*"

    def set_version(self):
        with open("include/velox/version.h", encoding="utf-8") as header:
            source = header.read()
        components = []
        for component in ("Major", "Minor", "Patch"):
            match = re.search(r"kVersion%s = ([0-9]+)" % component, source)
            if not match:
                raise ValueError("Velox version header is malformed")
            components.append(match.group(1))
        self.version = ".".join(components)

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        toolchain = CMakeToolchain(self)
        toolchain.variables["VELOX_BUILD_EXAMPLES"] = False
        toolchain.variables["VELOX_ENABLE_CUDA"] = bool(self.options.with_cuda)
        toolchain.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
        toolchain.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "Velox")
        self.cpp_info.set_property("cmake_target_name", "velox::velox")
        self.cpp_info.libs = ["velox"]
