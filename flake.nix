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
          overlays = [
            nix-ros-overlay.overlays.default
            nixgl.overlay
          ];
        };
        rosDistro = pkgs.rosPackages.humble;

        # Define beamngpy with its propagated dependencies using the proper attribute.
        beamngpy = pkgs.python3Packages.buildPythonPackage rec {
          pname = "beamngpy";
          version = "1.32";
          src = pkgs.fetchPypi {
            inherit pname version;
            sha256 = "sha256-A1uV2/F5fnGK3MoAtW+cXGUDMbhqDebVfHb9Q1gPj0s=";
          };
          propagatedBuildInputs = [
            pkgs.python3Packages.numpy
            pkgs.python3Packages.msgpack
            pkgs.python3Packages.pillow
            pkgs.python3Packages.jinja2
            pkgs.python3Packages.scipy
            pkgs.python3Packages.matplotlib
            pkgs.python3Packages.seaborn
          ];
          doCheck = false;
        };

        # Define a custom Python environment that includes BeamNGpy.
        pythonWithBeamNGpy = pkgs.python3.withPackages (
          ps: with ps; [
            beamngpy
            ipython
            black
            flake8
            mypy
            pytest
            pydocstyle
          ]
        );

        # Combine ROS packages into a single environment derivation.
        rosEnv = rosDistro.buildEnv {
          paths = with rosDistro; [
            ros-core
            ros-base
            rmw-fastrtps-cpp
            rmw-fastrtps-shared-cpp
            fastrtps
            fastcdr
            ament-cmake-core
            ament-lint-auto
            ament-lint-common
            python-cmake-module
            plotjuggler
            plotjuggler-ros
          ];
        };

      in
      {
        devShells.default = pkgs.mkShell {
          name = "bng-ros-dev-fish";

          packages = with pkgs; [
            colcon
            cmake
            ninja
            pkg-config
            gnumake
            gcc
            gdb
            binutils

            pythonWithBeamNGpy

            pyright
            python3Packages.setuptools
            python3Packages.wheel
            python3Packages.pip
            python3Packages.cython

            rosEnv

            qt5.qtbase

            fishPlugins.bass
          ];

          ROS_DOMAIN_ID = "42";

          shellHook = ''
            echo "Entering ROS2 + BeamNG Nix shell..."
            unset PYTHONHOME
            unset PYTHONPATH

            # export AMENT_PREFIX_PATH="${rosEnv}:$AMENT_PREFIX_PATH"
            export CMAKE_PREFIX_PATH="${rosEnv}:$CMAKE_PREFIX_PATH"

            export PATH="${pythonWithBeamNGpy}/bin:$PATH"
            export PYTHONPATH="${pythonWithBeamNGpy}/${pkgs.python3.sitePackages}:$PYTHONPATH"

            export LD_LIBRARY_PATH="${rosEnv}/lib:${rosEnv}/opt/rmw_fastrtps_cpp/lib:${rosEnv}/opt/fastrtps/lib:$LD_LIBRARY_PATH"

            export CFLAGS="-I${pythonWithBeamNGpy}/include/python${pythonWithBeamNGpy.pythonVersion}"

            echo "Use colcon build && source ./install/setup.bash to build the dev environment."
          '';
        };
      }
    );

  nixConfig = {
    extra-substituters = [ "https://attic.iid.ciirc.cvut.cz/ros" ];
    extra-trusted-public-keys = [ "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q=" ];
  };
}
