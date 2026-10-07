project(
    'psu-nm-proxy',
    'cpp',
    version: '0.1',
    meson_version: '>=1.1.1',
    default_options: [
        'warning_level=3',
        # sdbusplus (asio/connection.hpp) uses std::move_only_function and
        # requires C++23; see the pinned sdbusplus meson.build cpp_std.
        'cpp_std=c++23',
    ],
)

sdbusplus_dep = dependency('sdbusplus')
systemd_dep = dependency('systemd')

# Boost.Asio headers only (sdbusplus::asio::object_server is header-only).
boost_dep = dependency('boost', required: false)

src_inc = include_directories('src')

executable(
    'psu-nm-proxy',
    'src/main.cpp',
    'src/ipmb_client.cpp',
    dependencies: [
        sdbusplus_dep,
        systemd_dep,
        boost_dep,
    ],
    include_directories: src_inc,
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
