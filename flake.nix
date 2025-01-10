{
  description = "Kalman filter implementation in Java";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    nixpkgs,
    flake-utils,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = nixpkgs.legacyPackages.${system};
    in {
      devShells.default = pkgs.mkShell {
        packages = builtins.attrValues {
          inherit
            (pkgs)
            jdk
            maven
            ;
        };
        shellHook = ''
          echo "Welcome to Java DevShell"
        '';
      };
    });
}
