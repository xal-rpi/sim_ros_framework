{
  pkgs ? import <nixpkgs> { },
}:

pkgs.mkShell {
  buildInputs = with pkgs; [
    # Lua interpreter and package manager
    lua
    luajitPackages.luarocks

    # Lua LSP
    sumneko-lua-language-server

    # Lua formatter
    stylua

    # Additional development tools
    gcc
    gnumake

    # Required for certain Lua modules
    openssl
    readline

    # Documentation
    luajitPackages.ldoc
  ];

}
