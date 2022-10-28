cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(rust_ackerman)

find_package(Corrosion REQUIRED)

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#============================================================================
# Rust Business Logic
corrosion_import_crate(MANIFEST_PATH src/rust/Cargo.toml)

# Wrapper stuff
add_library(RustAckermannSteeringSystem SHARED
  src/AckermannSteering.cc)
set_property(TARGET RustAckermannSteeringSystem PROPERTY CXX_STANDARD 17)

target_link_libraries(RustAckermannSteeringSystem
  PUBLIC
    rust_ackerman
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(TARGETS RustAckermannSteeringSystem DESTINATION lib)
