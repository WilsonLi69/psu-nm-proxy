gtest_dep = dependency('gtest', main: true)

test_exe = executable(
    'psu-nm-proxy-test',
    'linear11_test.cpp',
    'nm_command_test.cpp',
    'pmbus_test.cpp',
    'sensor_test.cpp',
    'inventory_test.cpp',
    'poll_cycle_test.cpp',
    'failure_tracker_test.cpp',
    'operational_state_test.cpp',
    'presence_tracker_test.cpp',
    'config_test.cpp',
    'threshold_test.cpp',
    'slot_discovery_test.cpp',
    'event_logger_test.cpp',
    '../src/config.cpp',
    '../src/inventory.cpp',
    '../src/ipmb_client.cpp',
    '../src/ipmb_reader.cpp',
    '../src/slot_discovery.cpp',
    dependencies: [
        gtest_dep,
        nlohmann_json_dep,
        sdbusplus_dep,
        systemd_dep,
        boost_dep,
    ],
    include_directories: src_inc,
    install: false,
)

test('psu-nm-proxy', test_exe)
