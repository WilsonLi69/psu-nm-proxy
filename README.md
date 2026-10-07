# psu-nm-proxy

PSU telemetry proxy for OpenBMC. It speaks the Intel ME / Node Manager `D9h`
"Send Raw PMBUS command" proxy over the Ipmb D-Bus channel and publishes the
decoded PSU telemetry onto D-Bus as `xyz.openbmc_project.Sensor.Value` /
inventory objects.

Canonical specification:
<https://github.com/WilsonLi69/openbmc_private/blob/main/docs/specs/psu-telemetry.md>

Repository topology decision:
<https://github.com/WilsonLi69/openbmc_private/blob/main/docs/adr/0002-psu-nm-proxy-own-repository.md>

## Repository split

This repository holds the **daemon source only**. Following ADR-0002 (and the
`virtual-led-ctrl` precedent), the integration layer `meta-custom` carries:

- the BitBake recipe (`recipes-phosphor/psu-nm-proxy/`), pinned by `SRCREV`,
- the JSON configuration,
- any board-specific patches.

No functional PSU behaviour is implemented yet; this repository exists so later
daemon slices have a repo, a build, and a pipeline to land in.

## Building

### OpenBMC cross SDK

```
source /usr/local/oecore-x86_64/environment-setup-arm1176jzs-openbmc-linux-gnueabi

meson setup build
ninja -C build
```

This produces the `psu-nm-proxy` binary for the target BMC.

### Unit tests

```
./run-ci
```

`run-ci` is the repository-level CI entry point (the interface OpenBMC's
`unit-test.py` looks for). It configures with `-Dwerror=true
-Dwarning_level=3 -Dtests=enabled`, builds, and runs `meson test`.

## CI

`.github/workflows/ci.yml` runs `run-ci` on a native x86_64 runner on every
push and pull request, provisioning `sdbusplus` (and its dependencies) from
source.
