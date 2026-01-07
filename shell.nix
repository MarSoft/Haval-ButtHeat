let
  nixpkgs-esp-dev = builtins.fetchGit {
    url = "https://github.com/mirrexagon/nixpkgs-esp-dev.git";

    # Optionally pin to a specific commit of `nixpkgs-esp-dev`.
    # rev = "<commit hash>";
    #ref = "v5.5.2";
    rev = "b7a2e7caf5b730948fbffeae10edd30d7ede1f83";  # just before v5.5.2, see #113
  };

  pkgs = import <nixpkgs> { overlays = [ (import "${nixpkgs-esp-dev}/overlay.nix") ]; };
in
pkgs.mkShell {
  name = "esp-project";

  buildInputs = with pkgs; [
    esp-idf-full
  ];

  shellHook = ''
    export PATH=$(echo $PATH | tr : '\n' | sort -u | grep riscv32-esp-elf-esp-idf | sed s:bin:riscv32-esp-elf/bin: | tr '\n' :):"$PATH"
    # FIXME: drop this after #110 is merged
  '';
}
