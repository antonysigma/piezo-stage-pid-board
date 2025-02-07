project('encoder', 'cpp',
    version: '1.4.4',
)

arduinocore_proj = subproject('arduinocore-avr')
encoder_lib = static_library('encoder',
    sources: 'Encoder.cpp',
    cpp_args: [
        '-std=c++17',
        '-DARDUINO=182',
    ],
    dependencies: [
        arduinocore_proj.get_variable('arduinocore_dep'),
    ],
)

encoder_dep = declare_dependency(
    link_with:encoder_lib,
    include_directories: '.',
)