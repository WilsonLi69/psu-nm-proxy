gtest_dep = dependency('gtest', main: true)

test_exe = executable(
    'psu-nm-proxy-test',
    'basic_test.cpp',
    dependencies: [
        gtest_dep,
    ],
    install: false,
)

test('psu-nm-proxy', test_exe)
