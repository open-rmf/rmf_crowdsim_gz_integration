# Use RUST in Gazebo plugins

This tutorial will explain how to use Rust in Gazebo plugins. It covers how to create the folder structure and it tackles three different scenarios:
  - function arguments
  - function returns
  - Shared structs between Gazebo plugin and Rust

## Requeriments
 - [Gazebo](https://gazebosim.org/docs/garden/install_ubuntu).
 - [Corrosion](https://github.com/corrosion-rs/corrosion#installation). Corrosion,
 formerly known as cmake-cargo, is a tool for integrating Rust into an existing CMake project.
 Corrosion is capable of importing executables, static libraries, and dynamic libraries from a crate.

 - [cargo](https://doc.rust-lang.org/cargo/). Cargo is the Rust package manager.
 Cargo downloads your Rust package's dependencies, compiles your packages,
 makes distributable packages, and uploads them to crates.io, the Rust community’s package registry.
 Follow this instructions to [install cargo](https://doc.rust-lang.org/cargo/getting-started/installation.html).


## Project tree

We are going to create a new Gazebo package with the following structure

```
rust_hello_world
├── rust                     Rust files
│    ├── src                 Rust code
│    ├── rust_interface.h    Rust bindings
│    ├── RustHelloWorld.cc   Gazebo plugin implementation
│    ├── RustHelloWorld.hh   Gazebo plugin header
├── README.md                Description of the project.
└── CMakeLists.txt           CMake build script.
```

## Create the workspace

Create the following workspace:

```bash
mkdir ~/rust_ws/src -p
cd ~/rust_ws
```

## Create a simple Gazebo Plugin

Create the Gazebo plugin inside the workspace `~/rust_ws/src/rust_hello_world`:

```bash
mkdir ~/rust_ws/src/rust_hello_world
```

Add the file `src/RustHelloWorld.hh`:

```cpp
#ifndef RUST_HELLO_WORLD__HH_
#define RUST_HELLO_WORLD__HH_

#include <memory>

#include <gz/sim/System.hh>

namespace rust_hello_world
{
  class RustHelloWorldSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: RustHelloWorldSystem();

    /// \brief Destructor
    public: ~RustHelloWorldSystem();

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;
  };
}

#endif
```

Add the file `src/RustHelloWorld.cc` to code the implementation of the plugin:

```cpp
#include <memory>

#include "RustHelloWorld.hh"
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EventManager.hh>

#include <sdf/Element.hh>

using namespace gz;
using namespace gz::sim;

namespace rust_hello_world
{

RustHelloWorldSystem::RustHelloWorldSystem()
{
}

RustHelloWorldSystem::~RustHelloWorldSystem()
{
}

void RustHelloWorldSystem::Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr)
{
  gzmsg << "RustHelloWorldSystem::Configure" << std::endl;
}

void RustHelloWorldSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
{
}
}
GZ_ADD_PLUGIN(rust_hello_world::RustHelloWorldSystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
```

Finally create the `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(rust_hello_world)

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Wrapper stuff
add_library(RustHelloWorldSystem SHARED
   src/RustHelloWorld.cc)
set_property(TARGET RustHelloWorldSystem PROPERTY CXX_STANDARD 17)

target_link_libraries(RustHelloWorldSystem
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(TARGETS RustHelloWorldSystem DESTINATION lib)
```

This is, so far, the same workflow that we use to create a Gazebo plugin.

Now you can compile your workspace:

```bash
colcon build
```

### Including the plugin in a sdf file:

When the plugin is compiled you can use it within Gazebo.
Create a file called `rust_hello_world.sdf`:

```bash
touch ~/rust_ws/src/rust_hello_world/worlds/rust_hello_world.sdf
```

Edit the file:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <plugin filename="RustHelloWorldSystem" name="rust_hello_world::RustHelloWorldSystem">
    </plugin>
  </world>
</sdf>
```

Lauch the simulation:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=`echo $HOME`/rust_ws/install/lib/
gz sim -v 2 ~/rust_ws/src/rust_hello_world/worlds/rust_hello_world.sdf
```

You should be able to see the following trace:

```bash
[Msg] RustHelloWorldSystem::Configure
```

## Including Rust

 - Let's include some Rust code in our plugin. We need to use
[Corrosion](https://github.com/corrosion-rs/corrosion#installation), there are two
different ways to use `Corrosion` in our `CMakeLists.txt`.

  1. If [corrosion is installed in your system](https://github.com/corrosion-rs/corrosion#cmake-install). (*We will use this way in the tutorial*).
  ```cmake
  find_package(Corrosion REQUIRED)
  ```

  2. If you are using `CMake >= 3.19` or installation is difficult or not feasible in your environment,
  you can use the `FetchContent` module to include `Corrosion`. This will download `Corrosion` and use
  it as if it were a subdirectory at configure time.
  Include in your `CMakeLists.txt`:
  ```cmake
    include(FetchContent)

    FetchContent_Declare(
     Corrosion
     GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
     GIT_TAG v0.2.1 # Optionally specify a commit hash, version tag or branch here
    )

    FetchContent_MakeAvailable(Corrosion)
  ```

- Include the two following lines in your `CMakeLists.txt`:
  ```cmake
  ...
  find_package(Corrosion REQUIRED)
  ...
  # Rust Business Logic
  corrosion_import_crate(MANIFEST_PATH src/rust/Cargo.toml)

  target_link_libraries(RustHelloWorldSystem
    PUBLIC
      rust_hello_world
  ...
  ```

- Create the file `src/rust/Cargo.toml` with at least this basic data:
  ```toml
    [package]
    name = "rust_hello_world"
    version = "0.0.1"
    edition = "2022"

    # See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html
    [dependencies]

    [lib]
    crate-type=["staticlib"]
  ```

### Create a function that accepts arguments

- Now we should create two new files:

  - `src/rust_interface.h`: This file contains the bindings between Rust and C.
  - `src/rust/src/lib.rs`: This file contains the Rust code that we want to run in the Gazebo plugin.

  Let create a function that includes a string as an argument:

   1. Include the header in the `rust_interface.h` file:
   ```c
   #ifndef RUST_INTERFACE__H_
   #define RUST_INTERFACE__H_

   #include <cstdint>
   extern "C" void print_string(char const *name);
   #endif
   ```
   2. Write the Rust code in `lib.rs`:
   ```rust
   #[no_mangle]
   pub extern "C" fn print_string(str: *const c_char)
   {
       let name = unsafe { std::ffi::CStr::from_ptr(str).to_str().unwrap() };
       println!("Hello, {}! I'm Rust!", name);
   }
   ```
   3. Use this code in the Gazebo plugin. Modify `src/RustHelloWorld.cc`:
   ```cpp
   ...
   #include "rust_interface.h"
   ...
   void RustHelloWorldSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                              gz::sim::EntityComponentManager &_ecm)
   {
     std::string msg = "Hello, world! Simulation is ";
     if (!_info.paused)
       msg += "not ";
     msg += "paused.";
     print_string(msg.c_str());
   ```
   4. Compile the code again:
   ```bash
   colcon build
   ```
   You can see that the Rust code is compiled properly. If there is any error in the Rust
   code you will be able to see the errors in this prompt:
   ```bash
   Compiling rust_hello_world v0.0.1 (/home/ahcorde/rust_ws/src/rust_hello_world/src/rust)
   Finished dev [unoptimized + debuginfo] target(s) in 0.27s
   ```

   5. Run Gazebo with the plugin
   ```bash
   export GZ_SIM_SYSTEM_PLUGIN_PATH=`echo $HOME`/rust_ws/install/lib/
   gz sim -v 2 rust_hello_world.sdf
   ```

   You will see two traces:
     - If the simulation is paused:
     ```bash
     Hello, Hello, world! Simulation is paused.! I'm Rust!
     Hello, Hello, world! Simulation is paused.! I'm Rust!
     ```
     - or if the simulation is running:
     ```
     Hello, Hello, world! Simulation is not paused.! I'm Rust!
     Hello, Hello, world! Simulation is not paused.! I'm Rust!
     ```

### Create a function that returns a string

This example will show how to get a string from Rust and how to free the memory:

  1. Include this functions in the `rust_interface.h` file:
  ```c
  ...
  extern "C" const char * get_string();
  extern "C" void free_string(const char * str);
  ...
  ```
  2. Write the Rust code in `lib.rs`:
  ```rust
  #[no_mangle]
  pub extern "C" fn get_string() -> *const c_char
  {
      // string
      let c_string = std::ffi::CString::new("This string is generated in Rust").expect("CString::new failed");
      return c_string.into_raw();
  }

  /// # Safety
  /// The ptr should be a valid pointer to the string allocated by rust
  #[no_mangle]
  pub unsafe extern fn free_string(ptr: *const c_char) {
      // Take the ownership back to rust and drop the owner
      let _ = std::ffi::CString::from_raw(ptr as *mut _);
  }
  ```

  3. Use this code in the Gazebo plugin. Modify `src/RustHelloWorld.cc`:
  ```cpp
  void RustHelloWorldSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
  {
  ...
    const char* rust_string = get_string();

    // Messages printed with gzmsg only show when running with verbosity 3 or
    // higher (i.e. gz sim -v 3)
    gzmsg << rust_string << std::endl;
    free_string(rust_string);
    ...
  }
  ```

  4. Compile the code again:
  ```bash
  colcon build
  ```

  5. Run Gazebo with the plugin
  ```bash
  export GZ_SIM_SYSTEM_PLUGIN_PATH=`echo $HOME`/rust_ws/install/lib/
  gz sim -v 2 rust_hello_world.sdf
  ```
  You will see:
  ```
  [Msg] This string is generated in Rust
  ```

### Sharing a struct between the Gazebo plugins and Rust

  1. Include this functions in the `rust_interface.h` file:
  ```c
  ...
  // struct shared between the Gazebo plugins and Rust
  extern "C" typedef struct SimulationBinding
  {
    const char* text;
  } simulation_binding_t;

  /// \brief Create a new simulation_binding instance
  extern "C" simulation_binding_t * simulation_binding_new();
  /// \brief Release simulation_binding instance
  extern "C" void simulation_binding_free(simulation_binding_t *t);
  ...
  ```

  2. Write the Rust code in `lib.rs`:
  ```rust
  ...

  #[repr(C)]
  pub struct SimulationBinding
  {
      text: std::ffi::CString
  }

  #[no_mangle]
  pub extern "C" fn simulation_binding_new() -> *mut SimulationBinding
  {
      Box::into_raw(Box::new(SimulationBinding
      {
        text: std::ffi::CString::new("String defined in the struct").expect("CString::new failed")
      }))
  }

  #[no_mangle]
  pub extern "C" fn simulation_binding_free(ptr: *mut SimulationBinding)
  {
      if ptr.is_null() {
          return;
      }
      unsafe {
          Box::from_raw(ptr);
      }
  }
  ...
  ```
  3. Add this line in the `src/RustHelloWorld.hh`
  ```cpp
      #include "rust_interface.h"
      ...
      class RustHelloWorldSystem:
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
      {
        ...
        private: simulation_binding_t* simulationBindings;
        ...
      };
  ```

  4. Use this new code in the Gazebo plugin. Modify `src/RustHelloWorld.cc`:

  ```cpp
  ...
  RustHelloWorldSystem::RustHelloWorldSystem()
  {
    this->simulationBindings = simulation_binding_new();
  }

  RustHelloWorldSystem::~RustHelloWorldSystem()
  {
    simulation_binding_free(this->simulationBindings);
  }
  ...
  void RustHelloWorldSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
  {
    ...
    gzmsg << this->simulationBindings->text << std::endl;
    ...
  }
  ```

  5. Compile the code again:
  ```bash
  colcon build
  ```

  6. Run Gazebo with the plugin
  ```bash
  export GZ_SIM_SYSTEM_PLUGIN_PATH=`echo $HOME`/rust_ws/install/lib/
  gz sim -v 2 rust_hello_world.sdf
  ```
  You will see:
  ```bash
  [Msg] String defined in the struct
  ```
