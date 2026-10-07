gtest_dep = dependency('gtest', main: true)

test_exe = executable(
    'psu-nm-proxy-test',
    'linear11_test.cpp',
    'nm_command_test.cpp',
    dependencies: [
        gtest_dep,
    ],
    include_directories: src_inc,
    install: false,
)

test('psu-nm-proxy', test_exe)
