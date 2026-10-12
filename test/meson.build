gtest_dep = dependency('gtest', main: true)

test_exe = executable(
    'psu-nm-proxy-test',
    'linear11_test.cpp',
    'nm_command_test.cpp',
    'pmbus_test.cpp',
    'sensor_test.cpp',
    'inventory_test.cpp',
    'failure_tracker_test.cpp',
    'config_test.cpp',
    '../src/config.cpp',
    dependencies: [
        gtest_dep,
        nlohmann_json_dep,
    ],
    include_directories: src_inc,
    install: false,
)

test('psu-nm-proxy', test_exe)
