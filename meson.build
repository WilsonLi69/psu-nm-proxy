project(
    'psu-nm-proxy',
    'cpp',
    version: '0.1',
    meson_version: '>=1.1.1',
    default_options: [
        'warning_level=3',
        'cpp_std=c++20',
    ],
)

sdbusplus_dep = dependency('sdbusplus')
systemd_dep = dependency('systemd')

executable(
    'psu-nm-proxy',
    'src/main.cpp',
    dependencies: [
        sdbusplus_dep,
        systemd_dep,
    ],
    install: true,
    install_dir: get_option('bindir'),
)

systemd_system_unit_dir = systemd_dep.get_variable(
    pkgconfig: 'systemdsystemunitdir',
    pkgconfig_define: ['prefix', get_option('prefix')],
)

install_data(
    'psu-nm-proxy.service',
    install_dir: systemd_system_unit_dir,
)

if get_option('tests').enabled()
    subdir('test')
endif
