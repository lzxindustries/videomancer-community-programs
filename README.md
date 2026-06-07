# Videomancer Community Programs

[![Forgejo](https://img.shields.io/badge/host-Forgejo-blue)](https://git.lzxindustries.net/lzx/videomancer-community-programs)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

A community repository for FPGA-based video processing programs for the [Videomancer](https://github.com/lzxindustries/videomancer-sdk) platform.

## 🎨 What is Videomancer?

Videomancer is an open-source FPGA video synthesis platform by LZX Industries. It allows you to create real-time video effects by writing VHDL code that runs directly on FPGA hardware.

**This repository** collects community-created programs that extend the capabilities of Videomancer hardware with new effects, processors, and creative tools.

## 🚀 Quick Start

If you want to build the community programs for your Videomancer hardware:

```bash
# 1. Clone the repository
git clone git@git.lzxindustries.net:lzx/videomancer-community-programs.git
cd videomancer-community-programs

# 2. Initialize submodules
git submodule update --init --recursive

# 3. Run setup (one-time)
./scripts/setup.sh

# 4. Build all programs
./build_programs.sh
```

Your compiled `.vmprog` files will be in the `out/` directory, ready to load onto your Videomancer hardware.

## 🛠️ Building Programs

The `build_programs.sh` script compiles VHDL programs into `.vmprog` packages for Videomancer hardware.

### Build All Programs

Build everything in the repository:

```bash
./build_programs.sh
```

### Build Programs from a Vendor/Author

Build all programs from a specific vendor (e.g., `lzx`):

```bash
./build_programs.sh lzx
```

### Build a Specific Program

Build one specific program:

```bash
./build_programs.sh lzx passthru
```

### Output Directory Structure

Built programs are placed in `out/`, organized by hardware revision and vendor:

```
out/
└── rev_b/
    ├── lzx/
    │   ├── passthru.vmprog
    │   └── yuv_amplifier.vmprog
    └── [yourname]/
        └── [yourprogram].vmprog
```

## 📚 Documentation

- **[Newcomer's Guide](docs/newcomer-guide.md)** - Step-by-step walkthrough for absolute beginners: GitHub, cloning, building, creating a program from `passthru`, loading it onto hardware, and submitting a pull request
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - How to contribute programs to this repository
- **[Program Development Guide](videomancer-sdk/docs/program-development-guide.md)** - Complete VHDL development workflow
- **[TOML Configuration Guide](videomancer-sdk/docs/toml-config-guide.md)** - Program configuration format
- **[Videomancer SDK](https://github.com/lzxindustries/videomancer-sdk)** - Core SDK documentation

## 🤝 Contributing

We welcome community contributions! To contribute your own video processing programs:

1. Read **[CONTRIBUTING.md](CONTRIBUTING.md)** for detailed guidelines
2. Create your program under `programs/yourname/programname/`
3. Follow the [Program Development Guide](videomancer-sdk/docs/program-development-guide.md)
4. Test thoroughly on hardware
5. Submit a pull request

All contributions are licensed under GPL-3.0.

## 🎯 Example Programs

Reference programs in `programs/lzx/`:

- **passthru** - Minimal passthrough (1 cycle latency) - use as a template
- **yuv_amplifier** - Color amplifier with 8 controls (14 cycle latency) - demonstrates multi-stage pipeline

## ⚖️ License

This repository is licensed under **GPL-3.0-only**.

- All community contributions must be GPL-3.0 compatible
- Programs retain original author copyright
- See [LICENSE](LICENSE) for complete terms
