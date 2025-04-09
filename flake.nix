{
  description = "BeamNG ROS 2 Development Environment";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT: Use the nixpkgs version from nix-ros-overlay
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        # Define the ROS distribution to use (humble for ROS 2)
        rosDistro = pkgs.rosPackages.humble;

        # Create BeamNGpy Python environment
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

            # Install BeamNGpy from PyPI with provided SHA
            (buildPythonPackage rec {
              pname = "beamngpy";
              version = "1.30";

              src = fetchPypi {
                inherit pname version;
                sha256 = "sha256-2lsuxJ6zCx2jFEx3t7UbT6RORd7D82YjK7jIk0tfD0w=";
              };

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
      in
      {
        devShells.default = pkgs.mkShell {
          name = "bng-ros-dev";

          packages = [
            # Build tools
            pkgs.colcon
            pkgs.cmake
            pkgs.ninja
            pkgs.pkg-config

            # Python with BeamNGpy
            pythonWithBeamNGpy
            pkgs.pyright

            # Fish shell
            pkgs.fish
            pkgs.fishPlugins.bass # For sourcing bash scripts in fish

            # Create ROS environment with all required packages
            (rosDistro.buildEnv {
              paths = with rosDistro; [
                # Core ROS packages
                ros-core
                ros-base

                # RMW implementation (explicit inclusion for runtime)
                rmw-implementation
                rmw-fastrtps-cpp
                rmw-fastrtps-shared-cpp

                # Development tools
                ament-cmake-core
                ament-lint-auto
                ament-lint-common
                python-cmake-module

                plotjuggler
                plotjuggler-ros
              ];
            })
            pkgs.libGL
            pkgs.libGLU
            pkgs.glfw
            pkgs.qt5.qtbase
          ];

          # Environment variables
          ROS_DOMAIN_ID = "42"; # Can be changed to any number from 0-101
          RMW_IMPLEMENTATION = "rmw_fastrtps_cpp";

          # Set path variables for proper ROS integration - using shellHook instead of direct setting

          shellHook = ''
            # Set ROS environment variables properly
            export LD_LIBRARY_PATH="${rosDistro.rmw-fastrtps-cpp}/lib:${rosDistro.fastrtps}/lib:${rosDistro.fastcdr}/lib:$LD_LIBRARY_PATH"
            export AMENT_PREFIX_PATH="${rosDistro.ros-core}:$(dirname $(dirname $(which ros2)))"
            export CMAKE_PREFIX_PATH="${rosDistro.ros-core}"

            export PYTHONPATH=${pythonWithBeamNGpy}/${pkgs.python3.sitePackages}:$PYTHONPATH

            export LD_LIBRARY_PATH=${pkgs.libGL}/lib:${pkgs.libGLU}/lib:$LD_LIBRARY_PATH
          '';
        };
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
