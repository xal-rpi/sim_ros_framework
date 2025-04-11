{
  description = "BeamNG ROS 2 Development Environment (Fish Only)";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nixgl.url = "github:nix-community/nixGL";
  };

  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      nixgl,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default nixgl.overlay ];
        };
        rosDistro = pkgs.rosPackages.humble;

        # Define the custom python environment with beamngpy
        pythonWithBeamNGpy = pkgs.python3.withPackages (
          ps: with ps; [
            numpy
            msgpack
            pillow
            jinja2
            scipy
            matplotlib
            seaborn
            ipython
            black
            flake8
            mypy
            pytest
            pydocstyle
            # Build beamngpy from PyPI source
            (buildPythonPackage rec {
              pname = "beamngpy";
              version = "1.30";
              src = pkgs.fetchPypi {
                inherit pname version;
                sha256 = "sha256-2lsuxJ6zCx2jFEx3t7UbT6RORd7D82YjK7jIk0tfD0w=";
              };
              # Dependencies needed by beamngpy at runtime
              propagatedBuildInputs = [
                numpy
                msgpack
                pillow
                jinja2
                scipy
                matplotlib
                seaborn
              ];
              doCheck = false;
            })
          ]
        );

        # Combine ROS packages into a single environment derivation
        rosEnv = rosDistro.buildEnv {
          paths = with rosDistro; [
            ros-core
            ros-base
            # Select the desired RMW implementation explicitly
            rmw-fastrtps-cpp
            # Dependencies for rmw_fastrtps_cpp
            rmw-fastrtps-shared-cpp
            fastrtps
            fastcdr
            # Core build/ament tools
            ament-cmake-core
            ament-lint-auto
            ament-lint-common
            python-cmake-module
            # User requested tools
            plotjuggler
            plotjuggler-ros
          ];
        };

      in
      {
        devShells.default = pkgs.mkShell {
          name = "bng-ros-dev-fish";

          packages = [
            # Build tools
            pkgs.colcon
            pkgs.cmake
            pkgs.ninja
            pkgs.pkg-config
            pkgs.gnumake
            pkgs.gcc
            pkgs.gdb
            pkgs.binutils

            # Custom Python environment
            pythonWithBeamNGpy

            # Python build/lint tools (available in the shell PATH)
            pkgs.pyright
            pkgs.python3.pkgs.setuptools # Often needed by colcon/build systems
            pkgs.python3.pkgs.wheel # Often needed by colcon/build systems
            pkgs.python3.pkgs.pip
            pkgs.python3.pkgs.cython # If needed for building certain packages

            # ROS environment
            rosEnv

            # Graphics/Simulation deps
            pkgs.nixgl.auto.nixGLDefault
            pkgs.qt5.qtbase

            # Fish plugin needed for the 'cb' function sourcing bash env
            pkgs.fishPlugins.bass
          ];

          # Environment variables set directly by mkShell
          ROS_DOMAIN_ID = "42";
          RMW_IMPLEMENTATION = "rmw_fastrtps_cpp";

          # shellHook runs in bash/sh before fish starts
          # Use POSIX syntax here
          shellHook = ''
            echo "Entering ROS2 + BeamNG Nix shell..."
            # Unset PYTHONHOME/PYTHONPATH if they exist, ROS/colcon sets its own
            unset PYTHONHOME
            unset PYTHONPATH

            # Add ROS paths
            export AMENT_PREFIX_PATH="${rosEnv}:$AMENT_PREFIX_PATH"
            export CMAKE_PREFIX_PATH="${rosEnv}:$CMAKE_PREFIX_PATH"

            # Add custom Python environment's bin to PATH and site-packages maybe needed
            export PATH="${pythonWithBeamNGpy}/bin:$PATH"
            export PYTHONPATH="${pythonWithBeamNGpy}/${pkgs.python3.sitePackages}:$PYTHONPATH"

            # Add libgl for graphic support
            export LD_LIBRARY_PATH=${pkgs.libGL}/lib:${pkgs.libGLU}/lib:$LD_LIBRARY_PATH

            # Python headers
            export CFLAGS="-I${pythonWithBeamNGpy}/include/python${pythonWithBeamNGpy.pythonVersion}"

            echo "Use colcon build && source ./install/setup.bash to builf the dev environment."
          '';
        };
      }
    );

  # Configure Nix to use the ROS binary cache
  nixConfig = {
    extra-substituters = [ "https://attic.iid.ciirc.cvut.cz/ros" ];
    extra-trusted-public-keys = [ "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q=" ];
  };
}
